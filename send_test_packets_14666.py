"""
MAVLink 測試封包發送器（預設送往 UDP 14666，供 mavlink_gui.py 使用）

模擬無人機在 (24.772413, 121.047530) 附近繞圈飛行：
  2Hz: HEARTBEAT, GLOBAL_POSITION_INT
  1Hz: SYS_STATUS, GPS_RAW_INT
"""

import argparse
import math
import os
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

CENTER_LAT = 24.772413
CENTER_LON = 121.047530
EARTH_RADIUS_M = 6378137.0
AUTO_CUSTOM_MODE = 3
FLIGHT_ALT_M = 20.0
MSL_BASE_ALT_M = 100.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MAVLink 測試封包發送器 (14666)")
    parser.add_argument("--host", default="127.0.0.1", help="接收端 IP，預設 127.0.0.1")
    parser.add_argument("--port", type=int, default=14666, help="接收端 UDP Port，預設 14666")
    parser.add_argument("--sysid", type=int, default=1, help="來源 system id，預設 1")
    parser.add_argument("--compid", type=int, default=1, help="來源 component id，預設 1")
    parser.add_argument("--radius", type=float, default=20.0, help="繞圈半徑（公尺），預設 20")
    parser.add_argument("--period", type=float, default=20.0, help="繞一圈秒數，預設 20")
    return parser


def circle_state(elapsed: float, radius: float, period: float):
    """回傳繞圈（順時針）的 lat, lon, vx(北), vy(東), heading(度)。"""
    angle = 2 * math.pi * elapsed / period
    north = radius * math.cos(angle)
    east = radius * math.sin(angle)
    lat = CENTER_LAT + math.degrees(north / EARTH_RADIUS_M)
    lon = CENTER_LON + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(CENTER_LAT))))
    speed = 2 * math.pi * radius / period
    vx = -speed * math.sin(angle)
    vy = speed * math.cos(angle)
    heading = math.degrees(math.atan2(vy, vx)) % 360
    return lat, lon, vx, vy, heading


def send_2hz_packets(master: mavutil.mavfile, boot_ms: int, elapsed: float, args) -> None:
    lat, lon, vx, vy, heading = circle_state(elapsed, args.radius, args.period)

    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        AUTO_CUSTOM_MODE,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )

    master.mav.global_position_int_send(
        boot_ms,
        int(lat * 1e7),
        int(lon * 1e7),
        int((MSL_BASE_ALT_M + FLIGHT_ALT_M) * 1000),
        int(FLIGHT_ALT_M * 1000),
        int(vx * 100),
        int(vy * 100),
        0,
        int(heading * 100),
    )


def send_1hz_packets(master: mavutil.mavfile, unix_us: int, battery_remaining: int) -> None:
    master.mav.sys_status_send(
        0,
        0,
        0,
        220,
        int((14.0 + battery_remaining * 0.028) * 1000),
        int(12.5 * 100),
        battery_remaining,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    master.mav.gps_raw_int_send(
        unix_us,
        6,
        int(CENTER_LAT * 1e7),
        int(CENTER_LON * 1e7),
        int((MSL_BASE_ALT_M + FLIGHT_ALT_M) * 1000),
        int(1.2 * 100),
        int(1.5 * 100),
        0,
        0,
        12,
    )


def main() -> None:
    args = build_parser().parse_args()
    endpoint = f"udpout:{args.host}:{args.port}"
    master = mavutil.mavlink_connection(
        endpoint,
        source_system=args.sysid,
        source_component=args.compid,
    )

    print(f"[TX] start sending to {endpoint} (sysid={args.sysid}, compid={args.compid})")
    print(f"[TX] 繞圈中心 ({CENTER_LAT}, {CENTER_LON})，半徑 {args.radius}m，週期 {args.period}s")
    print("[TX]  2Hz: HEARTBEAT, GLOBAL_POSITION_INT")
    print("[TX]  1Hz: SYS_STATUS, GPS_RAW_INT")

    start = time.time()
    next_2hz = start
    next_1hz = start
    battery_remaining = 100

    while True:
        now = time.time()
        elapsed = now - start
        boot_ms = int(elapsed * 1000)

        if now >= next_2hz:
            send_2hz_packets(master, boot_ms, elapsed, args)
            next_2hz += 0.5

        if now >= next_1hz:
            send_1hz_packets(master, int(now * 1e6), battery_remaining)
            battery_remaining = max(0, battery_remaining - 1) if int(elapsed) % 10 == 0 else battery_remaining
            next_1hz += 1.0

        time.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[TX] stopped")
