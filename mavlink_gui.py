"""
MAVLink Web GUI

被動監聽 MAVLink UDP 遙測，透過網頁呈現 Dashboard 數據，並在右側地圖上以無人機 icon 顯示即時位置與軌跡。

    uv run mavlink_gui.py [--udp-port 14666] [--http-port 3333]
    瀏覽器開啟 http://<host>:3333

地圖使用 Leaflet 與 OSM / Esri 圖磚，瀏覽器端需可連外網。
"""

import argparse
import json
import math
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

UDP_PORT = 14666
HTTP_PORT = 3333
TRACK_MAX_POINTS = 2000

FIX_TYPE = {
    0: "No GPS",
    1: "No Fix",
    2: "2D Fix",
    3: "3D Fix",
    4: "DGPS",
    5: "RTK Float",
    6: "RTK Fixed",
    7: "Static",
    8: "PPP",
}

ARDU_COPTER_MODES = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
}

MESSAGE_TYPES = ["HEARTBEAT", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "SYS_STATUS"]


class TelemetryStore:
    """線程安全的最新遙測狀態與飛行軌跡。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._data = {
            "lat": None,
            "lon": None,
            "relative_altitude": None,
            "velocity_vx": None,
            "velocity_vy": None,
            "velocity_vz": None,
            "ground_speed": None,
            "heading": None,
            "flight_mode": None,
            "armed": None,
            "battery_voltage": None,
            "battery_current": None,
            "battery_percentage": None,
            "rtk_status": None,
            "eph": None,
            "epv": None,
        }
        self._last_heartbeat = None
        self._last_position = None
        self._track = deque(maxlen=TRACK_MAX_POINTS)
        self._track_total = 0

    def update(self, msg) -> None:
        msg_type = msg.get_type()
        now = time.monotonic()
        with self._lock:
            if msg_type == "HEARTBEAT":
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    return
                self._last_heartbeat = now
                self._data["flight_mode"] = ARDU_COPTER_MODES.get(msg.custom_mode, f"UNKNOWN({msg.custom_mode})")
                self._data["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif msg_type == "GLOBAL_POSITION_INT":
                lat, lon = msg.lat / 1e7, msg.lon / 1e7
                vx, vy, vz = msg.vx / 100, msg.vy / 100, msg.vz / 100
                self._last_position = now
                self._data.update(
                    lat=lat,
                    lon=lon,
                    relative_altitude=msg.relative_alt / 1000,
                    velocity_vx=vx,
                    velocity_vy=vy,
                    velocity_vz=vz,
                    ground_speed=math.hypot(vx, vy),
                    heading=None if msg.hdg == 65535 else msg.hdg / 100,
                )
                if not (lat == 0 and lon == 0):
                    self._track.append([lat, lon])
                    self._track_total += 1
            elif msg_type == "GPS_RAW_INT":
                self._data.update(
                    rtk_status=FIX_TYPE.get(msg.fix_type, str(msg.fix_type)),
                    eph=None if msg.eph == 65535 else msg.eph / 100,
                    epv=None if msg.epv == 65535 else msg.epv / 100,
                )
            elif msg_type == "SYS_STATUS":
                self._data.update(
                    battery_voltage=None if msg.voltage_battery == 65535 else msg.voltage_battery / 1000,
                    battery_current=None if msg.current_battery < 0 else msg.current_battery / 100,
                    battery_percentage=None if msg.battery_remaining < 0 else msg.battery_remaining,
                )

    def snapshot(self, since: int = 0) -> dict:
        now = time.monotonic()
        with self._lock:
            new_count = min(max(self._track_total - since, 0), len(self._track))
            new_points = list(self._track)[len(self._track) - new_count :]
            return {
                **self._data,
                "heartbeat_age": None if self._last_heartbeat is None else now - self._last_heartbeat,
                "position_age": None if self._last_position is None else now - self._last_position,
                "track": new_points,
                "track_total": self._track_total,
            }


store = TelemetryStore()


def mavlink_loop(udp_port: int) -> None:
    conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{udp_port}", source_system=254)
    print(f"[MAVLink GUI] Listening MAVLink on udp:{udp_port}")
    while True:
        try:
            msg = conn.recv_match(type=MESSAGE_TYPES, blocking=True, timeout=1)
            if msg is not None:
                store.update(msg)
        except Exception as exc:
            print(f"[MAVLink GUI] MAVLink error: {exc}")
            time.sleep(0.5)


class Handler(BaseHTTPRequestHandler):
    def _send(self, code: int, content_type: str, body: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path, _, query = self.path.partition("?")
        if path == "/":
            self._send(200, "text/html; charset=utf-8", HTML.encode("utf-8"))
        elif path == "/api/state":
            params = dict(p.split("=", 1) for p in query.split("&") if "=" in p)
            try:
                since = int(params.get("since", 0))
            except ValueError:
                since = 0
            self._send(200, "application/json", json.dumps(store.snapshot(since)).encode("utf-8"))
        else:
            self._send(404, "text/plain; charset=utf-8", b"Not Found")

    def log_message(self, format, *args) -> None:
        pass


HTML = r"""<!DOCTYPE html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<title>MAVLink Dashboard</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  * { box-sizing: border-box; }
  html, body { height: 100%; margin: 0; background: #111; color: #ddd; font-family: "Noto Sans TC", "Microsoft JhengHei", sans-serif; }
  #app { display: flex; height: 100%; }
  #panel { width: 340px; min-width: 340px; background: #1b1b1b; overflow-y: auto; padding: 10px; border-right: 2px solid #000; }
  #map-wrap { flex: 1; position: relative; }
  #map { position: absolute; inset: 0; }
  h1 { font-size: 15px; margin: 0 0 8px; display: flex; justify-content: space-between; align-items: center; }
  #conn { font-size: 12px; padding: 2px 8px; border-radius: 10px; background: #555; }
  #conn.ok { background: #1e7d32; }
  #conn.stale { background: #b26a00; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-bottom: 8px; }
  .card { background: #262626; border-radius: 6px; padding: 8px; text-align: center; }
  .card.wide { grid-column: span 2; }
  .label { font-size: 12px; color: #aaa; }
  .value { font-size: 30px; font-weight: 600; margin-top: 2px; line-height: 1.2; }
  .value.small { font-size: 20px; }
  .unit { font-size: 12px; color: #888; margin-left: 2px; font-weight: 400; }
  .c-alt { color: #b9a4ff; } .c-spd { color: #ffa35c; } .c-hdg { color: #62d69a; }
  .c-vx { color: #ff8a8a; } .c-vy { color: #8ac8ff; } .c-vz { color: #ffe27a; }
  .badge { display: inline-block; padding: 2px 10px; border-radius: 4px; font-size: 18px; font-weight: 600; background: #444; }
  .armed { background: #c62828; } .disarmed { background: #2e7d32; }
  .bar { height: 14px; background: #444; border-radius: 7px; overflow: hidden; margin: 6px 0 2px; }
  .bar > div { height: 100%; width: 0; background: #4caf50; transition: width .3s; }
  .stale .value { opacity: .35; }
  #compass { width: 90px; height: 90px; }
  .map-ctl { position: absolute; top: 10px; right: 60px; z-index: 1000; background: rgba(0,0,0,.65); padding: 6px 10px; border-radius: 6px; font-size: 13px; }
  .map-ctl button { margin-left: 6px; }
  #bottom { position: absolute; left: 0; right: 0; bottom: 0; z-index: 1000; background: rgba(0,0,0,.7); font-size: 13px; padding: 4px 10px; display: flex; gap: 18px; }
  .uav-icon { transition: transform .4s linear; filter: drop-shadow(0 0 3px #000); }
</style>
</head>
<body>
<div id="app">
  <div id="panel">
    <h1><span>UAV Dashboard</span><span id="conn">等待連線</span></h1>
    <div class="grid">
      <div class="card"><div class="label">飛行模式</div><div class="value small" id="mode">--</div></div>
      <div class="card"><div class="label">解鎖狀態</div><div class="value small"><span class="badge" id="armed">--</span></div></div>
      <div class="card"><div class="label">相對高度 (m)</div><div class="value c-alt" id="alt">--</div></div>
      <div class="card"><div class="label">地速 (m/s)</div><div class="value c-spd" id="gs">--</div></div>
      <div class="card wide" style="display:flex;align-items:center;justify-content:space-around;">
        <div><div class="label">航向 Heading (deg)</div><div class="value c-hdg" id="hdg">--</div></div>
        <svg id="compass" viewBox="-50 -50 100 100">
          <circle r="46" fill="#1b1b1b" stroke="#555" stroke-width="2"/>
          <g id="compass-rot">
            <text y="-33" text-anchor="middle" font-size="12" fill="#ff6b6b">N</text>
            <text y="42" text-anchor="middle" font-size="12" fill="#999">S</text>
            <text x="38" y="4" text-anchor="middle" font-size="12" fill="#999">E</text>
            <text x="-38" y="4" text-anchor="middle" font-size="12" fill="#999">W</text>
          </g>
          <polygon points="0,-26 7,10 0,4 -7,10" fill="#62d69a"/>
        </svg>
      </div>
      <div class="card"><div class="label">Vx 北向 (m/s)</div><div class="value small c-vx" id="vx">--</div></div>
      <div class="card"><div class="label">Vy 東向 (m/s)</div><div class="value small c-vy" id="vy">--</div></div>
      <div class="card wide"><div class="label">Vz 垂直 (m/s，向上為正)</div><div class="value small c-vz" id="vz">--</div></div>
      <div class="card wide">
        <div class="label">電池</div>
        <div class="value small"><span id="bpct">--</span><span class="unit">%</span></div>
        <div class="bar"><div id="bbar"></div></div>
        <div class="label"><span id="bv">--</span> V &nbsp;/&nbsp; <span id="bi">--</span> A</div>
      </div>
      <div class="card wide"><div class="label">GPS / RTK</div><div class="value small" id="rtk">--</div></div>
      <div class="card"><div class="label">EPH / HDOP (m)</div><div class="value small" id="eph">--</div></div>
      <div class="card"><div class="label">EPV / VDOP (m)</div><div class="value small" id="epv">--</div></div>
    </div>
  </div>
  <div id="map-wrap">
    <div id="map"></div>
    <div class="map-ctl">
      <label><input type="checkbox" id="follow" checked> 自動平移地圖</label>
      <button id="clear">清除軌跡</button>
    </div>
    <div id="bottom">
      <span>Lat <b id="b-lat">--</b></span>
      <span>Lon <b id="b-lon">--</b></span>
      <span>Alt <b id="b-alt">--</b> m</span>
    </div>
  </div>
</div>
<script>
const $ = id => document.getElementById(id);
const fmt = (v, d = 2) => (v === null || v === undefined) ? '--' : Number(v).toFixed(d);

const osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '&copy; OpenStreetMap' });
const sat = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', { maxZoom: 19, attribution: 'Esri World Imagery' });
const map = L.map('map', { layers: [sat], zoomControl: true }).setView([24.772413, 121.04753], 18);
L.control.layers({ '衛星': sat, '街道': osm }).addTo(map);

const uavSvg = `<svg class="uav-icon" id="uav-svg" width="56" height="56" viewBox="-28 -28 56 56">
  <g stroke="#fff" stroke-width="2" fill="#222">
    <line x1="-14" y1="-14" x2="14" y2="14"/><line x1="14" y1="-14" x2="-14" y2="14"/>
  </g>
  <g fill="rgba(255,235,59,.85)" stroke="#000" stroke-width="1">
    <circle cx="-14" cy="-14" r="9"/><circle cx="14" cy="-14" r="9"/>
    <circle cx="-14" cy="14" r="9" fill="rgba(255,255,255,.7)"/><circle cx="14" cy="14" r="9" fill="rgba(255,255,255,.7)"/>
  </g>
  <polygon points="0,-18 5,2 0,-2 -5,2" fill="#e53935" stroke="#fff" stroke-width="1"/>
  <circle r="4" fill="#222" stroke="#fff"/>
</svg>`;
const icon = L.divIcon({ className: '', html: uavSvg, iconSize: [56, 56], iconAnchor: [28, 28] });
let marker = null;
const trackLine = L.polyline([], { color: '#ffeb3b', weight: 3 }).addTo(map);

let since = 0, rot = 0, centered = false;

function setHeading(h) {
  if (h === null || h === undefined) return;
  // 累加最短角度差，避免 359°→1° 時 CSS 轉場繞整圈
  rot += ((h - rot) % 360 + 540) % 360 - 180;
  const svg = document.getElementById('uav-svg');
  if (svg) svg.style.transform = `rotate(${rot}deg)`;
  $('compass-rot').setAttribute('transform', `rotate(${-h})`);
}

function render(s) {
  const hb = s.heartbeat_age, pos = s.position_age;
  const conn = $('conn');
  if (hb === null && pos === null) { conn.textContent = '等待連線'; conn.className = ''; }
  else if ((hb === null || hb > 3) && (pos === null || pos > 3)) { conn.textContent = 'STALE'; conn.className = 'stale'; }
  else { conn.textContent = '已連線'; conn.className = 'ok'; }
  document.getElementById('panel').classList.toggle('stale', conn.className === 'stale');

  $('mode').textContent = s.flight_mode ?? '--';
  const armed = $('armed');
  armed.textContent = s.armed === null ? '--' : (s.armed ? 'ARMED' : 'DISARMED');
  armed.className = 'badge ' + (s.armed === null ? '' : (s.armed ? 'armed' : 'disarmed'));
  $('alt').textContent = fmt(s.relative_altitude);
  $('gs').textContent = fmt(s.ground_speed);
  $('hdg').textContent = fmt(s.heading, 1);
  $('vx').textContent = fmt(s.velocity_vx);
  $('vy').textContent = fmt(s.velocity_vy);
  // MAVLink vz 為 NED（向下為正），取反後以向上為正顯示
  $('vz').textContent = fmt(s.velocity_vz === null ? null : -s.velocity_vz);
  $('bpct').textContent = s.battery_percentage ?? '--';
  $('bv').textContent = fmt(s.battery_voltage, 2);
  $('bi').textContent = fmt(s.battery_current, 2);
  const pct = s.battery_percentage ?? 0;
  const bar = $('bbar');
  bar.style.width = pct + '%';
  bar.style.background = pct > 50 ? '#4caf50' : (pct > 20 ? '#ff9800' : '#e53935');

  const rtk = $('rtk');
  rtk.textContent = s.rtk_status ?? '--';
  const colors = { 'RTK Fixed': '#4caf50', 'RTK Float': '#ff9800', 'DGPS': '#cddc39', '3D Fix': '#ffeb3b' };
  rtk.style.color = colors[s.rtk_status] || (s.rtk_status ? '#ef5350' : '#ddd');
  $('eph').textContent = fmt(s.eph);
  $('epv').textContent = fmt(s.epv);

  $('b-lat').textContent = fmt(s.lat, 7);
  $('b-lon').textContent = fmt(s.lon, 7);
  $('b-alt').textContent = fmt(s.relative_altitude);

  if (s.track_total < since) { trackLine.setLatLngs([]); }
  s.track.forEach(p => trackLine.addLatLng(p));
  since = s.track_total;

  if (s.lat !== null && s.lon !== null && !(s.lat === 0 && s.lon === 0)) {
    const ll = [s.lat, s.lon];
    if (!marker) marker = L.marker(ll, { icon, interactive: false, zIndexOffset: 1000 }).addTo(map);
    else marker.setLatLng(ll);
    setHeading(s.heading);
    if (!centered) { map.setView(ll, 18); centered = true; }
    else if ($('follow').checked) map.panTo(ll, { animate: true, duration: 0.4 });
  }
}

async function poll() {
  try {
    const r = await fetch('/api/state?since=' + since, { cache: 'no-store' });
    render(await r.json());
  } catch (e) {
    $('conn').textContent = 'GUI 伺服器離線';
    $('conn').className = 'stale';
  }
  setTimeout(poll, 200);
}

$('clear').onclick = () => trackLine.setLatLngs([]);
poll();
</script>
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description="MAVLink Web GUI")
    parser.add_argument("--udp-port", type=int, default=UDP_PORT, help=f"MAVLink UDP 輸入埠，預設 {UDP_PORT}")
    parser.add_argument("--http-port", type=int, default=HTTP_PORT, help=f"Web 伺服器埠，預設 {HTTP_PORT}")
    args = parser.parse_args()

    threading.Thread(target=mavlink_loop, args=(args.udp_port,), daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)
    print(f"[MAVLink GUI] Open http://localhost:{args.http_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
