# MAVLink GUI

被動監聽 MAVLink UDP 遙測，透過網頁顯示 Dashboard 數據，並在地圖上以無人機 icon 呈現即時位置與軌跡。

- `mavlink_gui.py`：接收 MAVLink UDP（預設 14666）並提供 Web GUI（預設 3333）
- `send_test_packets_14666.py`：測試封包發送器，模擬無人機繞圈飛行（HEARTBEAT、GLOBAL_POSITION_INT、SYS_STATUS、GPS_RAW_INT）

> 地圖使用 Leaflet 與 OSM / Esri 圖磚，瀏覽器端需可連外網。

## 環境需求

- Ubuntu 22.04 或以上
- [uv](https://docs.astral.sh/uv/)（會自動下載並管理 Python 與相依套件，不需手動建立虛擬環境）

## 安裝 uv

```bash
sudo apt update
sudo apt install -y curl
curl -LsSf https://astral.sh/uv/install.sh | sh
```

安裝完成後重新開啟終端機（或執行 `source $HOME/.local/bin/env`），確認：

```bash
uv --version
```

也可使用 pipx 安裝：`sudo apt install -y pipx && pipx install uv`。

## 安裝專案相依套件

```bash
git clone <repo-url> mavlink-gui
cd mavlink-gui
uv sync
```

`uv sync` 會依 `pyproject.toml` 與 `.python-version` 建立 `.venv` 並安裝 `pymavlink`。

## 執行

### 啟動 GUI

```bash
uv run mavlink_gui.py
# 或
uv run mavlink-gui
```

瀏覽器開啟 <http://localhost:3333>。

參數：

| 參數 | 預設 | 說明 |
|------|------|------|
| `--udp-port` | 14666 | MAVLink UDP 輸入埠 |
| `--http-port` | 3333 | Web 伺服器埠 |

範例：`uv run mavlink_gui.py --udp-port 14550 --http-port 8080`

### 以測試封包驗證

另開一個終端機：

```bash
uv run send_test_packets_14666.py
# 或
uv run mavlink-test-sender
```

參數：

| 參數 | 預設 | 說明 |
|------|------|------|
| `--host` | 127.0.0.1 | 接收端 IP |
| `--port` | 14666 | 接收端 UDP Port |
| `--sysid` | 1 | 來源 system id |
| `--compid` | 1 | 來源 component id |
| `--radius` | 20.0 | 繞圈半徑（公尺） |
| `--period` | 20.0 | 繞一圈秒數 |

## 新增套件

```bash
uv add <package>
```
