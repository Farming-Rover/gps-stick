import base64
import json
import math
import time

import pandas as pd
import pynmea2
import serial
import streamlit as st
import streamlit.components.v1 as components

from placement_map_html import MAP_MESSAGE_TYPE, post_map_message, render_placement_map
# Serial Port Configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Base Station Configuration
BASE_IP = "192.168.1.62"
BASE_PORT = "2101"

# st.map size values are in meters (not pixels)
GRID_MARKER_SIZE_M = 0.8
TARGET_MARKER_SIZE_M = 1.2
USER_MARKER_SIZE_M = 0.6

# --- CORE GEOMETRIC MATH ---
def get_distance_and_bearing(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(delta_lambda)
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
    return distance, bearing

def generate_grid(origin_lat, origin_lon, rows, cols, spacing_meters):
    grid = []
    lat_degree_meters = 111132.92
    lon_degree_meters = 40075000 * math.cos(math.radians(origin_lat)) / 360
    for r in range(rows):
        offset = 0
        if r % 2 == 1:
            offset = spacing_meters / 2
        for c in range(cols):
            dn = r * spacing_meters
            de = c * spacing_meters
            point_lat = origin_lat + (dn / lat_degree_meters)
            point_lon = origin_lon + (de / lon_degree_meters) + offset
            grid.append({
                "Point": f"R{r+1}C{c+1}",
                "lat": point_lat,
                "lon": point_lon,
                "color": "#1E90FF",
                "size": GRID_MARKER_SIZE_M,
            })
    return pd.DataFrame(grid)

def sync_placement_from_query_params():
    """Apply map pan/zoom updates sent via URL query params (no PyArrow needed)."""
    qp = st.query_params
    if "origin_lat" not in qp or "origin_lon" not in qp:
        return False

    new_lat = float(qp["origin_lat"])
    new_lon = float(qp["origin_lon"])
    new_zoom = int(qp.get("map_zoom", st.session_state.map_zoom))

    origin_moved = (
        abs(new_lat - st.session_state.preview_origin_lat) > 1e-8
        or abs(new_lon - st.session_state.preview_origin_lon) > 1e-8
    )
    zoom_changed = new_zoom != st.session_state.map_zoom

    if origin_moved or zoom_changed:
        st.session_state.preview_origin_lat = new_lat
        st.session_state.preview_origin_lon = new_lon
        st.session_state.map_view_center = [new_lat, new_lon]
        st.session_state.map_zoom = new_zoom

    del st.query_params["origin_lat"]
    del st.query_params["origin_lon"]
    if "map_zoom" in qp:
        del st.query_params["map_zoom"]

    return origin_moved or zoom_changed

_LIVE_FIELD_MAP_HTML = """<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <link
      rel="stylesheet"
      href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
      integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY="
      crossorigin=""
    />
    <style>
      html,
      body {
        margin: 0;
        padding: 0;
        width: 100%;
        height: 100%;
        overflow: hidden;
      }

      #map {
        width: 100%;
        height: 100%;
      }

      .map-shell {
        position: relative;
        width: 100%;
        height: 520px;
      }
    </style>
  </head>
  <body>
    <div class="map-shell">
      <div id="map-config" data-config-b64="__MAP_CONFIG_B64__" hidden></div>
      <div id="map"></div>
    </div>

    <script
      src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
      integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
      crossorigin=""
    ></script>
    <script>
      const MAP_MESSAGE_TYPE = __MAP_MESSAGE_TYPE__;
      const MAP_CONFIG = JSON.parse(
        atob(document.getElementById("map-config").dataset.configB64)
      );

      function drawGridMarkers() {
        gridLayer.clearLayers();
        MAP_CONFIG.grid_points.forEach((point) => {
          const isTarget = point.point === MAP_CONFIG.target_point;
          L.circleMarker([point.lat, point.lon], {
            radius: isTarget ? 9 : 6,
            color: isTarget ? "#00FF00" : "#1E90FF",
            fillColor: isTarget ? "#00FF00" : "#1E90FF",
            fillOpacity: isTarget ? 0.9 : 0.45,
            weight: 2,
          })
            .bindPopup(point.point)
            .addTo(gridLayer);
        });
      }

      function drawUserMarker() {
        userMarker.clearLayers();
        L.circleMarker([MAP_CONFIG.user_lat, MAP_CONFIG.user_lon], {
          radius: 7,
          color: "#FF0000",
          fillColor: "#FF0000",
          fillOpacity: 1.0,
          weight: 2,
        })
          .bindPopup("Your position")
          .addTo(userMarker);
      }

      function handleMapMessage(data) {
        if (!data || data.type !== MAP_MESSAGE_TYPE || !map) {
          return;
        }

        if (data.action === "updateTarget") {
          if (data.target_point != null) {
            MAP_CONFIG.target_point = data.target_point;
            drawGridMarkers();
          }
          return;
        }

        if (data.action === "updateUser") {
          const nextLat = Number(data.user_lat);
          const nextLon = Number(data.user_lon);
          if (
            !Number.isFinite(nextLat) ||
            !Number.isFinite(nextLon) ||
            (nextLat === MAP_CONFIG.user_lat && nextLon === MAP_CONFIG.user_lon)
          ) {
            return;
          }
          MAP_CONFIG.user_lat = nextLat;
          MAP_CONFIG.user_lon = nextLon;
          drawUserMarker();
        }
      }

      const map = L.map("map", {
        center: [MAP_CONFIG.center_lat, MAP_CONFIG.center_lon],
        zoom: MAP_CONFIG.zoom,
        zoomControl: true,
        maxZoom: 21,
      });

      L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        attribution: "&copy; OpenStreetMap contributors",
        maxZoom: 21,
        maxNativeZoom: 19,
      }).addTo(map);

      const gridLayer = L.layerGroup().addTo(map);
      const userMarker = L.layerGroup().addTo(map);

      drawGridMarkers();
      drawUserMarker();

      try {
        if (typeof BroadcastChannel !== "undefined") {
          const gridChannel = new BroadcastChannel(MAP_MESSAGE_TYPE);
          gridChannel.onmessage = (event) => {
            handleMapMessage(event.data);
          };
        }
      } catch (error) {
        // Ignore channel errors on older browsers.
      }

      try {
        if (window.parent.__gpsStickMapHandler) {
          window.parent.removeEventListener(
            "message",
            window.parent.__gpsStickMapHandler
          );
        }
        window.parent.__gpsStickMapHandler = (event) => {
          handleMapMessage(event.data);
        };
        window.parent.addEventListener(
          "message",
          window.parent.__gpsStickMapHandler
        );
      } catch (error) {
        // Ignore parent listener errors in restricted iframes.
      }
    </script>
  </body>
</html>
"""


def _encode_map_payload(payload):
    return base64.b64encode(json.dumps(payload).encode("utf-8")).decode("ascii")


def render_live_field_map(
    grid_points,
    target_point,
    user_lat,
    user_lon,
    center_lat,
    center_lon,
    zoom=19,
    height=520,
):
    config = {
        "grid_points": grid_points,
        "target_point": target_point,
        "user_lat": user_lat,
        "user_lon": user_lon,
        "center_lat": center_lat,
        "center_lon": center_lon,
        "zoom": zoom,
    }
    html = (
        _LIVE_FIELD_MAP_HTML.replace("__MAP_CONFIG_B64__", _encode_map_payload(config))
        .replace("__MAP_MESSAGE_TYPE__", json.dumps(MAP_MESSAGE_TYPE))
    )
    components.html(html, height=height, scrolling=False)


def post_user_position_update(user_lat, user_lon, session_key):
    """Push a GPS position to mounted maps without remounting them."""
    position = (user_lat, user_lon)
    if st.session_state.get(session_key) == position:
        return
    st.session_state[session_key] = position
    post_map_message({
        "action": "updateUser",
        "user_lat": user_lat,
        "user_lon": user_lon,
    })

# --- LIVE HARDWARE GPS ---
def _iter_nmea_sentences(line):
    """Split glued NMEA lines like '$GNGGA,...0$GNGSA,...' into separate sentences."""
    if not line or "$" not in line:
        return
    if line.count("$") == 1:
        yield line
        return
    for part in line.lstrip("$").split("$"):
        if part:
            yield "$" + part

def _parse_gga_sentence(sentence):
    if not (sentence.startswith("$GNGGA") or sentence.startswith("$GPGGA")):
        return None
    try:
        return pynmea2.parse(sentence)
    except (pynmea2.ParseError, pynmea2.ChecksumError):
        return None

def _ensure_gps_serial():
    if "last_gps_msg" not in st.session_state:
        st.session_state.last_gps_msg = None

    serial_conn = st.session_state.get("gps_serial")
    if serial_conn is not None:
        try:
            if serial_conn.is_open:
                return
        except Exception:
            pass

    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        serial_conn.reset_input_buffer()
        st.session_state.gps_serial = serial_conn
    except Exception:
        st.session_state.gps_serial = None

def poll_gps(block_until_fix=False, timeout_s=5.0):
    """Read buffered serial data and return the latest valid GGA fix."""
    _ensure_gps_serial()
    ser = st.session_state.get("gps_serial")
    if ser is None:
        return st.session_state.get("last_gps_msg")

    deadline = time.time() + (timeout_s if block_until_fix else 0)

    while True:
        try:
            line = ser.readline().decode("ascii", errors="replace").strip()
        except Exception:
            break

        if line:
            for sentence in _iter_nmea_sentences(line):
                msg = _parse_gga_sentence(sentence)
                if msg is not None:
                    st.session_state.last_gps_msg = msg

        if block_until_fix:
            if st.session_state.last_gps_msg is not None:
                break
            if time.time() >= deadline:
                break
            continue

        try:
            if not line or ser.in_waiting == 0:
                break
        except Exception:
            break

    return st.session_state.last_gps_msg

def init_placement_state(current_lat, current_lon):
    if "grid_df" not in st.session_state:
        st.session_state.grid_df = None
    if "grid_finalized" not in st.session_state:
        st.session_state.grid_finalized = False
    if "preview_origin_lat" not in st.session_state:
        st.session_state.preview_origin_lat = current_lat
    if "preview_origin_lon" not in st.session_state:
        st.session_state.preview_origin_lon = current_lon
    if "map_view_center" not in st.session_state:
        st.session_state.map_view_center = [current_lat, current_lon]
    if "map_zoom" not in st.session_state:
        st.session_state.map_zoom = 19
    if "placement_map_mounted" not in st.session_state:
        st.session_state.placement_map_mounted = False

def reset_grid_to_current_position(current_lat, current_lon, line_count_m, line_count_e, spacing):
    st.session_state.preview_origin_lat = current_lat
    st.session_state.preview_origin_lon = current_lon
    st.session_state.map_view_center = [current_lat, current_lon]
    if st.session_state.grid_finalized:
        st.session_state.grid_df = generate_grid(
            current_lat, current_lon, line_count_m, line_count_e, spacing
        )

def finalize_grid_location(line_count_m, line_count_e, spacing):
    st.session_state.grid_df = generate_grid(
        st.session_state.preview_origin_lat,
        st.session_state.preview_origin_lon,
        line_count_m,
        line_count_e,
        spacing,
    )
    st.session_state.grid_finalized = True

# --- STREAMLIT UI LAYOUT ---
st.set_page_config(page_title="RTK Live Map Guide", layout="wide")
st.title("RTK Live Map Guide")

if st.session_state.get("last_gps_msg") is None:
    msg = poll_gps(block_until_fix=True, timeout_s=10)
else:
    msg = poll_gps()

if msg is None:
    st.warning("Waiting for GPS fix from the receiver...")
    st.stop()

current_lat, current_lon = float(msg.latitude), float(msg.longitude)
init_placement_state(current_lat, current_lon)


with st.sidebar:
    st.header("Grid Settings")

    @st.fragment
    def render_grid_settings(current_lat, current_lon):
        grid_lines_m = st.number_input("Rows", min_value=1, value=4, key="grid_dim_m")
        grid_lines_e = st.number_input("Columns", min_value=1, value=4, key="grid_dim_e")
        spacing = st.number_input(
            "Spacing (Meters)", min_value=0.5, value=2.0, step=0.5, key="grid_spacing"
        )

        if st.button("Reset grid to current position", use_container_width=True):
            reset_grid_to_current_position(
                current_lat, current_lon, grid_lines_m, grid_lines_e, spacing
            )
            post_map_message({
                "action": "updateView",
                "center_lat": current_lat,
                "center_lon": current_lon,
                "zoom": st.session_state.map_zoom,
            })

        post_map_message({
            "action": "updateGrid",
            "dim_m": int(grid_lines_m),
            "dim_e": int(grid_lines_e),
            "spacing_m": float(spacing),
        })

    render_grid_settings(current_lat, current_lon)

if not st.session_state.grid_finalized:
    sync_placement_from_query_params()

    st.subheader("Position Your Grid")
    st.caption(
        "Pan and zoom the map to slide the terrain under the preview grid. "
        "The grid origin (R1C1, green) stays at the map center. "
        "Click **Apply grid origin** on the map, then **Finalize grid location**."
    )

    if not st.session_state.placement_map_mounted:
        render_placement_map(
            center_lat=st.session_state.preview_origin_lat,
            center_lon=st.session_state.preview_origin_lon,
            zoom=st.session_state.map_zoom,
            line_count_m=int(st.session_state.get("grid_dim_m", 4)),
            line_count_e=int(st.session_state.get("grid_dim_e", 4)),
            spacing=float(st.session_state.get("grid_spacing", 2.0)),
            user_lat=current_lat,
            user_lon=current_lon,
            height=520,
        )
        st.session_state.placement_map_mounted = True

    @st.fragment(run_every=1.0)
    def refresh_placement_user_marker():
        if not st.session_state.placement_map_mounted:
            return
        msg = poll_gps()
        if msg is None:
            return
        post_user_position_update(
            float(msg.latitude),
            float(msg.longitude),
            "placement_user_pos",
        )

    refresh_placement_user_marker()

    col_info, col_finalize = st.columns([2, 1])
    with col_info:
        st.markdown(
            f"**Preview origin (R1C1):** "
            f"{st.session_state.preview_origin_lat:.8f}, {st.session_state.preview_origin_lon:.8f}"
        )
        st.markdown("**Legend:** green = grid origin (R1C1) · blue = preview points · red = your position")
    with col_finalize:
        if st.button("Finalize grid location", type="primary", use_container_width=True):
            finalize_grid_location(
                int(st.session_state.grid_dim_m),
                int(st.session_state.grid_dim_e),
                float(st.session_state.grid_spacing),
            )
            st.session_state.placement_map_mounted = False
            st.session_state.live_map_mounted = False
            st.rerun()

else:
    df = st.session_state.grid_df
    target_point = st.selectbox("Select Target Grid Point:", df["Point"].tolist())
    target_row = df[df["Point"] == target_point].iloc[0]
    target_lat, target_lon = target_row["lat"], target_row["lon"]

    if st.button("Adjust grid placement"):
        st.session_state.grid_finalized = False
        st.session_state.placement_map_mounted = False
        st.session_state.live_map_mounted = False
        st.rerun()

    grid_points = [
        {"point": row["Point"], "lat": row["lat"], "lon": row["lon"]}
        for _, row in df.iterrows()
    ]

    st.subheader("Live Field View")
    if not st.session_state.get("live_map_mounted"):
        render_live_field_map(
            grid_points=grid_points,
            target_point=target_point,
            user_lat=current_lat,
            user_lon=current_lon,
            center_lat=target_lat,
            center_lon=target_lon,
            zoom=19,
            height=520,
        )
        st.session_state.live_map_mounted = True
        st.session_state.live_map_target = target_point
        st.session_state.live_user_pos = (current_lat, current_lon)
    elif st.session_state.get("live_map_target") != target_point:
        st.session_state.live_map_target = target_point
        post_map_message({
            "action": "updateTarget",
            "target_point": target_point,
        })

    @st.fragment(run_every=0.5)
    def render_live_dashboard(t_lat, t_lon, t_name):
        msg = poll_gps()
        if msg is None:
            st.info("Waiting for GPS fix...")
            return

        c_lat, c_lon = float(msg.latitude), float(msg.longitude)
        dist, bear = get_distance_and_bearing(c_lat, c_lon, t_lat, t_lon)

        if int(msg.gps_qual) in [0, 1]:
            st.warning("RTK fix Quality is low. Please check your base station connection.")

        st.markdown("---")
        col1, col2 = st.columns(2)
        col1.metric(label="Distance to Target", value=f"{dist:.2f} m")
        col2.metric(label="Required Heading", value=f"{bear:.1f}°")

        if dist < 0.015:
            st.success("TARGET REACHED (Within 1.5cm RTK tolerance)!")
        else:
            st.info(f"Walk towards {bear:.1f}° for {dist:.2f} meters")

        post_user_position_update(c_lat, c_lon, "live_user_pos")

    render_live_dashboard(target_lat, target_lon, target_point)