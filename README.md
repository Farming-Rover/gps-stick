# GPS Stick — RTK Live Map Guide

Field navigation tool for laying out and walking a survey grid with centimeter-level RTK GNSS. A Raspberry Pi Zero runs a Streamlit app that reads position from a SparkFun ZED-F9P, applies NTRIP corrections, and guides you on a phone browser: place a grid on the map, then follow relative forward/back/left/right cues to each point.

## Hardware

### Overview

| Role | Device |
|------|--------|
| Computer / Wi‑Fi AP host | Raspberry Pi Zero |
| RTK GNSS receiver | SparkFun ZED-F9P |
| Antenna | GPS/GNSS antenna on the ZED-F9P |
| Field UI | Phone or tablet browser on the Pi’s network |

Power and data paths:

- The **Pi Zero** is powered from a USB supply wired into the Pi’s **5 V** and **GND** pins.
- The **ZED-F9P** is powered from the Pi’s **5 V** and **GND**.
- The Pi talks to the ZED-F9P over **UART** on the module’s **UART2** pins (**TX2** / **RX2**).
- The GNSS antenna connects to the ZED-F9P (antenna port on the board).

### Wiring

```
USB power ──► Pi Zero 5V / GND
                 │
                 ├── 5V  ──► ZED-F9P 5V
                 ├── GND ──► ZED-F9P GND
                 │
                 ├── UART TX (Pi) ──► ZED-F9P RX2
                 └── UART RX (Pi) ──► ZED-F9P TX2

GNSS antenna ──► ZED-F9P antenna connector
```

Notes:

- Cross TX/RX: Pi TX → ZED-F9P **RX2**, Pi RX → ZED-F9P **TX2**.
- Use a common ground between the Pi and the ZED-F9P.
- Default serial device in the app is `/dev/serial0` at **115200** baud (Pi hardware UART). Enable the serial interface and disable the login shell on that UART in `raspi-config` if needed.
- Confirm pin labels on your specific SparkFun ZED-F9P breakout; UART2 is the port used for this rover link.

### Software stack on the Pi

- Raspberry Pi OS
- Python 3 with a virtualenv (recommended)
- Streamlit app (`app.py`) serving the UI on the LAN
- Optional: NTRIP client (built into the app) for RTK corrections from a caster such as RTK2GO

## Features

- **Grid placement** — pan, zoom, and rotate the map under a screen-aligned preview grid; finalize when origin and orientation look right
- **Save / load grids** — JSON under `saved_grids/` including origin, spacing, dimensions, and orientation
- **Live field view** — RTK position from the stick; phone compass for facing direction and relative guidance to the nearest grid point
- **NTRIP** — stream RTCM corrections to the ZED-F9P over the same UART (see in-app sidebar)

## Project layout

| Path | Purpose |
|------|---------|
| `app.py` | Main Streamlit UI and serial / NTRIP orchestration |
| `gps_runtime.py` | Shared GNSS fix quality state across sessions |
| `ntrip_runtime.py` | Shared NTRIP client state |
| `placement_map_html.py` | Placement map Streamlit component bridge |
| `placement_map_component/` | Leaflet placement map (with local Leaflet assets) |
| `static/` | Live field map / nav panel HTML |
| `saved_grids/` | Saved grid JSON files (created at runtime) |
| `ntrip-rover.py` | Standalone NTRIP helper (reference / testing) |
| `serial_monitor.py` | Serial debugging utility |

## Setup

### Dependencies

On the Pi (example):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install streamlit pyserial pynmea2 pandas
```

Adjust versions as needed for your Pi OS / Python build. PyArrow is not required for the placement map path used here.

### Run

From the project directory:

```bash
streamlit run app.py --server.address 0.0.0.0 --server.headless true
```

Open the printed URL from a phone on the same network (or via the Pi’s hotspot). For compass heading, use **HTTPS** (or localhost); browsers block orientation sensors on plain HTTP.

### Serial config

Defaults in `app.py`:

```python
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 115200
```

Change these if your UART device path or baud rate differs.

## Typical field workflow

1. Power the stick (USB → Pi 5 V/GND); confirm the ZED-F9P has a fix.
2. Start Streamlit and open the app on a phone.
3. Connect NTRIP in the sidebar if you need RTK float/fixed.
4. Place the grid (pan / rotate / zoom), then **Finalize grid location**.
5. Optionally **Save Grid Location** (orientation is stored with the grid).
6. Tap **Enable compass** on the live map and walk points using the guidance HUD.

## License / attribution

Map tiles: © OpenStreetMap contributors. Leaflet and leaflet-rotate are used for interactive maps.
