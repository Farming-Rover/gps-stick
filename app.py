import streamlit as st
import pandas as pd
import serial
import pynmea2
import math
import time
import folium
from streamlit_folium import st_folium

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

def build_placement_map(view_center, zoom, preview_df, user_lat, user_lon):
    m = folium.Map(location=view_center, zoom_start=zoom, tiles="OpenStreetMap")
    for _, row in preview_df.iterrows():
        is_origin = row["Point"] == "R1C1"
        folium.CircleMarker(
            location=[row["lat"], row["lon"]],
            radius=9 if is_origin else 6,
            color="#00C853" if is_origin else "#1E90FF",
            fill=True,
            fill_color="#00C853" if is_origin else "#1E90FF",
            fill_opacity=0.85 if is_origin else 0.45,
            weight=2,
            popup=row["Point"],
        ).add_to(m)
    folium.CircleMarker(
        location=[user_lat, user_lon],
        radius=7,
        color="#FF0000",
        fill=True,
        fill_color="#FF0000",
        fill_opacity=1.0,
        weight=2,
        popup="Your position",
    ).add_to(m)
    return m

def sync_map_center_from_output(map_output):
    """Keep the preview grid anchored to the map viewport center after pan/zoom."""
    if not map_output or not map_output.get("center"):
        return False

    new_lat = map_output["center"]["lat"]
    new_lng = map_output["center"]["lng"]
    new_zoom = map_output.get("zoom", st.session_state.map_zoom)

    origin_moved = (
        abs(new_lat - st.session_state.preview_origin_lat) > 1e-8
        or abs(new_lng - st.session_state.preview_origin_lon) > 1e-8
    )
    zoom_changed = new_zoom != st.session_state.map_zoom

    if origin_moved or zoom_changed:
        st.session_state.preview_origin_lat = new_lat
        st.session_state.preview_origin_lon = new_lng
        st.session_state.map_view_center = [new_lat, new_lng]
        st.session_state.map_zoom = new_zoom
        return True
    return False

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

    serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    serial_conn.reset_input_buffer()
    st.session_state.gps_serial = serial_conn

def poll_gps(block_until_fix=False, timeout_s=5.0):
    """Read buffered serial data and return the latest valid GGA fix."""
    _ensure_gps_serial()
    ser = st.session_state.gps_serial
    deadline = time.time() + (timeout_s if block_until_fix else 0)

    while True:
        line = ser.readline().decode("ascii", errors="replace").strip()
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

        if not line or ser.in_waiting == 0:
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

def reset_grid_to_current_position(current_lat, current_lon, rows, cols, spacing):
    st.session_state.preview_origin_lat = current_lat
    st.session_state.preview_origin_lon = current_lon
    st.session_state.map_view_center = [current_lat, current_lon]
    if st.session_state.grid_finalized:
        st.session_state.grid_df = generate_grid(
            current_lat, current_lon, rows, cols, spacing
        )

def finalize_grid_location(rows, cols, spacing):
    st.session_state.grid_df = generate_grid(
        st.session_state.preview_origin_lat,
        st.session_state.preview_origin_lon,
        rows,
        cols,
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
    grid_rows = st.number_input("Rows", min_value=1, value=4, key="grid_rows")
    grid_cols = st.number_input("Columns", min_value=1, value=4, key="grid_cols")
    spacing = st.number_input("Spacing (Meters)", min_value=0.5, value=2.0, step=0.5, key="grid_spacing")

    if st.button("Reset grid to current position", use_container_width=True):
        reset_grid_to_current_position(current_lat, current_lon, grid_rows, grid_cols, spacing)
        st.rerun()

if not st.session_state.grid_finalized:
    st.subheader("Position Your Grid")
    st.caption(
        "Pan and zoom the map to slide the terrain under the preview grid. "
        "The grid origin (R1C1, green) stays at the map center. "
        "Use **Finalize grid location** when the grid is where you want it."
    )

    preview_df = generate_grid(
        st.session_state.preview_origin_lat,
        st.session_state.preview_origin_lon,
        grid_rows,
        grid_cols,
        spacing,
    )

    placement_map = build_placement_map(
        st.session_state.map_view_center,
        st.session_state.map_zoom,
        preview_df,
        current_lat,
        current_lon,
    )

    map_output = st_folium(
        placement_map,
        width=None,
        height=520,
        returned_objects=["center", "zoom"],
        center=st.session_state.map_view_center,
        zoom=st.session_state.map_zoom,
        key="grid_placement_map",
    )

    if sync_map_center_from_output(map_output):
        st.rerun()

    col_info, col_finalize = st.columns([2, 1])
    with col_info:
        st.markdown(
            f"**Preview origin (R1C1):** "
            f"{st.session_state.preview_origin_lat:.8f}, {st.session_state.preview_origin_lon:.8f}"
        )
        st.markdown("**Legend:** green = grid origin (R1C1) · blue = preview points · red = your position")
    with col_finalize:
        if st.button("Finalize grid location", type="primary", use_container_width=True):
            finalize_grid_location(grid_rows, grid_cols, spacing)
            st.rerun()

else:
    df = st.session_state.grid_df
    target_point = st.selectbox("Select Target Grid Point:", df["Point"].tolist())
    target_row = df[df["Point"] == target_point].iloc[0]
    target_lat, target_lon = target_row["lat"], target_row["lon"]

    if st.button("Adjust grid placement"):
        st.session_state.grid_finalized = False
        st.rerun()

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

        map_df = st.session_state.grid_df.copy()
        map_df.loc[map_df["Point"] == t_name, "color"] = "#00FF00"
        map_df.loc[map_df["Point"] == t_name, "size"] = TARGET_MARKER_SIZE_M

        user_marker = pd.DataFrame([{
            "Point": "YOU",
            "lat": c_lat,
            "lon": c_lon,
            "color": "#FF0000",
            "size": USER_MARKER_SIZE_M,
        }])
        combined_map_df = pd.concat([map_df, user_marker], ignore_index=True)

        st.subheader("Live Field View")
        st.map(
            combined_map_df,
            latitude="lat",
            longitude="lon",
            color="color",
            size="size",
            zoom=19,
        )

    render_live_dashboard(target_lat, target_lon, target_point)
