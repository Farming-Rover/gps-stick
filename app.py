import base64
import json
import math
import re
import socket
import threading
import time
import uuid
from pathlib import Path

import pynmea2
import serial
import streamlit as st

from placement_map_html import (
    MAP_MESSAGE_TYPE,
    post_broadcast_message,
    post_map_message,
    render_html_embed,
    render_placement_map,
)
import gps_runtime as gps_rt
import ntrip_runtime as ntrip_rt
# Serial Port Configuration
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 115200

# RTK2GO NTRIP caster (same settings as ntrip-rover.py)
NTRIP_HOST = "rtk2go.com"
NTRIP_PORT = 2101
NTRIP_USER_EMAIL = "voukich@gmail.com"
NTRIP_DEFAULT_MOUNTPOINT = "CA_SanJose_ML_X5"

# st.map size values are in meters (not pixels)
GRID_MARKER_SIZE_M = 0.8
TARGET_MARKER_SIZE_M = 1.2
USER_MARKER_SIZE_M = 0.6

# Shared GNSS serial port (GPS reads + NTRIP RTCM writes). Module-level so the
# NTRIP background thread can use it across Streamlit script reruns.
#
# A serial fd is full-duplex: one thread can read NMEA while another writes
# RTCM at the same time (POSIX tty). So we deliberately use SEPARATE locks:
#   * _gps_serial_lock guards only opening/reopening the shared handle.
#   * _gps_read_lock serializes NMEA readers against each other.
# RTCM writes take NEITHER, so corrections are never starved behind a blocking
# readline() (that starvation is why the UI couldn't reach RTK float while the
# standalone ntrip-rover.py — which only writes — could lock).
_gps_serial = None
_gps_serial_port = SERIAL_PORT
_gps_serial_error = None
_gps_serial_lock = threading.Lock()
_gps_read_lock = threading.Lock()

# Latest raw GGA sentence read from the board. The NTRIP thread periodically
# sends this back up to the caster (like ntrip-rover.py) — SNIP/RTK2GO casters
# generally expect the client to report its position to keep RTCM flowing.
_latest_gga_sentence = ""
_latest_gga_lock = threading.Lock()

NAV_MESSAGE_TYPE = "gps-stick-nav"


def rtk_status_from_gps_qual(gps_qual):
    """Map GGA gps_qual to a short label / CSS class for the live nav bar."""
    try:
        qual = int(gps_qual)
    except (TypeError, ValueError):
        return {
            "gps_qual": None,
            "rtk_label": "Waiting for GPS…",
            "rtk_class": "unknown",
        }
    if qual == 4:
        return {
            "gps_qual": qual,
            "rtk_label": "RTK Fix",
            "rtk_class": "fix",
        }
    if qual == 5:
        return {
            "gps_qual": qual,
            "rtk_label": "RTK Float",
            "rtk_class": "float",
        }
    return {
        "gps_qual": qual,
        "rtk_label": "No RTK",
        "rtk_class": "none",
    }


def _set_latest_gps_quality(msg):
    """Publish the latest GGA quality for all browser sessions."""
    if msg is None:
        return
    try:
        qual = int(msg.gps_qual)
    except (TypeError, ValueError, AttributeError):
        return
    num_sats = None
    try:
        if getattr(msg, "num_sats", None) not in (None, ""):
            num_sats = int(msg.num_sats)
    except (TypeError, ValueError):
        num_sats = None
    with gps_rt.lock:
        gps_rt.gps_qual = qual
        gps_rt.num_sats = num_sats
        gps_rt.updated_at = time.time()


def get_latest_rtk_status():
    with gps_rt.lock:
        qual = gps_rt.gps_qual
        num_sats = gps_rt.num_sats
    status = rtk_status_from_gps_qual(qual)
    status["num_sats"] = num_sats
    return status


def post_rtk_status_update(force=False):
    """Push current RTK quality to the live nav panel (all sessions share it)."""
    status = get_latest_rtk_status()
    key = (status.get("gps_qual"), status.get("rtk_label"), status.get("num_sats"))
    if not force and st.session_state.get("last_posted_rtk_status") == key:
        return
    st.session_state.last_posted_rtk_status = key
    post_broadcast_message(
        NAV_MESSAGE_TYPE,
        {
            "gps_qual": status.get("gps_qual"),
            "rtk_label": status.get("rtk_label"),
            "rtk_class": status.get("rtk_class"),
            "num_sats": status.get("num_sats"),
        },
    )

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

def offset_latlon(lat, lon, bearing_deg, distance_m):
    """Move from (lat, lon) by distance_m along bearing_deg (0=N, 90=E)."""
    if distance_m == 0:
        return float(lat), float(lon)
    earth_radius = 6371000.0
    bearing = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    ang = distance_m / earth_radius
    lat2 = math.asin(
        math.sin(lat1) * math.cos(ang)
        + math.cos(lat1) * math.sin(ang) * math.cos(bearing)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(ang) * math.cos(lat1),
        math.cos(ang) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)

def _row_is_odd(row_index):
    """True for odd lattice rows (including negatives: -1, -3, ...)."""
    return ((int(row_index) % 2) + 2) % 2 == 1


def _local_row_col_meters(origin_lat, origin_lon, lat, lon, orientation_deg):
    """Project (lat, lon) into meters along the grid row/col axes from origin."""
    north_m = (float(lat) - float(origin_lat)) * 111320.0
    east_m = (
        (float(lon) - float(origin_lon))
        * 111320.0
        * math.cos(math.radians(float(origin_lat)))
    )
    theta = math.radians(float(orientation_deg) % 360.0)
    # Unit along row bearing (0=N): (north, east) = (cos θ, sin θ)
    row_m = north_m * math.cos(theta) + east_m * math.sin(theta)
    # Unit along col bearing (θ+90): (-sin θ, cos θ)
    col_m = -north_m * math.sin(theta) + east_m * math.cos(theta)
    return row_m, col_m


def _nearest_lattice_indices(row_m, col_m, spacing_meters, staggered):
    """Snap local meters to the nearest (row, col) lattice indices."""
    spacing = float(spacing_meters)
    r0 = int(round(row_m / spacing))
    best = None
    for r in (r0 - 1, r0, r0 + 1):
        stagger = (spacing / 2.0) if (staggered and _row_is_odd(r)) else 0.0
        c = int(round((col_m - stagger) / spacing))
        for candidate_c in (c - 1, c, c + 1):
            d_row = row_m - r * spacing
            d_col = col_m - (candidate_c * spacing + stagger)
            dist2 = d_row * d_row + d_col * d_col
            if best is None or dist2 < best[0]:
                best = (dist2, r, candidate_c)
    return best[1], best[2]


def _grid_point_latlon(
    origin_lat, origin_lon, row_index, col_index, spacing_meters, orientation_deg, staggered
):
    row_bearing = float(orientation_deg) % 360.0
    col_bearing = (row_bearing + 90.0) % 360.0
    stagger = (
        (float(spacing_meters) / 2.0)
        if (staggered and _row_is_odd(row_index))
        else 0.0
    )
    point_lat, point_lon = offset_latlon(
        origin_lat, origin_lon, row_bearing, row_index * spacing_meters
    )
    return offset_latlon(
        point_lat, point_lon, col_bearing, col_index * spacing_meters + stagger
    )


def generate_grid(
    origin_lat,
    origin_lon,
    rows,
    cols,
    spacing_meters,
    orientation_deg=0,
    staggered=True,
):
    """Build a finite grid whose row axis points along orientation_deg (0=N)."""
    grid = []
    for r in range(int(rows)):
        for c in range(int(cols)):
            point_lat, point_lon = _grid_point_latlon(
                origin_lat,
                origin_lon,
                r,
                c,
                spacing_meters,
                orientation_deg,
                staggered,
            )
            grid.append({
                "Point": f"R{r + 1}C{c + 1}",
                "lat": point_lat,
                "lon": point_lon,
                "color": "#1E90FF",
                "size": GRID_MARKER_SIZE_M,
            })
    return grid


def generate_nearest_endless_grid_points(
    origin_lat,
    origin_lon,
    user_lat,
    user_lon,
    spacing_meters,
    orientation_deg=0,
    staggered=True,
    count=4,
):
    """Return the closest `count` points in the infinite, oriented lattice."""
    count = max(1, int(count))
    row_m, col_m = _local_row_col_meters(
        origin_lat, origin_lon, user_lat, user_lon, orientation_deg
    )
    center_r, center_c = _nearest_lattice_indices(
        row_m, col_m, spacing_meters, staggered
    )
    spacing = float(spacing_meters)
    # Neighborhood large enough that the N nearest lattice points are included.
    search = max(2, int(math.ceil(math.sqrt(count))) + 1)
    candidates = []
    for r in range(center_r - search, center_r + search + 1):
        stagger = (spacing / 2.0) if (staggered and _row_is_odd(r)) else 0.0
        for c in range(center_c - search, center_c + search + 1):
            d_row = row_m - r * spacing
            d_col = col_m - (c * spacing + stagger)
            dist2 = d_row * d_row + d_col * d_col
            candidates.append((dist2, r, c))
    candidates.sort(key=lambda item: item[0])

    grid = []
    key_parts = []
    for _dist2, r, c in candidates[:count]:
        point_lat, point_lon = _grid_point_latlon(
            origin_lat,
            origin_lon,
            r,
            c,
            spacing_meters,
            orientation_deg,
            staggered,
        )
        name = f"R{r}C{c}"
        key_parts.append(name)
        grid.append({
            "Point": name,
            "lat": point_lat,
            "lon": point_lon,
            "color": "#1E90FF",
            "size": GRID_MARKER_SIZE_M,
        })
    return grid, tuple(sorted(key_parts))


def grid_df_to_points(df):
    """Convert grid rows (list[dict] or legacy DataFrame-like) to map points."""
    if df is None:
        return []
    if hasattr(df, "to_dict"):
        rows = df.to_dict("records")
    else:
        rows = df
    return [
        {"point": row["Point"], "lat": float(row["lat"]), "lon": float(row["lon"])}
        for row in rows
    ]


def build_active_grid_df(user_lat, user_lon, line_count_m, line_count_e, spacing):
    """Build the active grid rows from current session settings."""
    origin_lat = float(st.session_state.preview_origin_lat)
    origin_lon = float(st.session_state.preview_origin_lon)
    orientation = float(st.session_state.get("preview_orientation_deg", 0.0)) % 360.0
    if st.session_state.get("grid_finalized") and "grid_orientation_deg" in st.session_state:
        orientation = float(st.session_state.grid_orientation_deg) % 360.0
    staggered = bool(st.session_state.get("grid_staggered", True))
    if st.session_state.get("grid_endless", True):
        df, center = generate_nearest_endless_grid_points(
            origin_lat,
            origin_lon,
            user_lat,
            user_lon,
            spacing,
            orientation,
            staggered,
            count=4,
        )
        return df, center
    return (
        generate_grid(
            origin_lat,
            origin_lon,
            line_count_m,
            line_count_e,
            spacing,
            orientation,
            staggered,
        ),
        None,
    )

def _encode_map_payload(payload):
    return base64.b64encode(json.dumps(payload).encode("utf-8")).decode("ascii")


_STATIC_DIR = Path(__file__).resolve().parent / "static"


@st.cache_resource
def _load_static_html(name: str, mtime: float) -> str:
    """Load large HTML templates once per process (mtime busts stale cache)."""
    return (_STATIC_DIR / name).read_text(encoding="utf-8")


def _static_html(name: str) -> str:
    path = _STATIC_DIR / name
    return _load_static_html(name, path.stat().st_mtime)


def _app_shell_css() -> str:
    """Parent-page CSS only (Streamlit strips <script> from markdown HTML)."""
    return """
    <style>
    [data-testid="stElementContainer"][data-stale="true"]:has(iframe.stCustomComponentV1) {
        opacity: 1 !important;
    }
    /* Keep the sidebar collapse chevron visible (not hover-only). */
    [data-testid="stSidebarCollapseButton"],
    [data-testid="stSidebarCollapseButton"] button {
        opacity: 1 !important;
        visibility: visible !important;
        pointer-events: auto !important;
    }
    [data-testid="stSidebarHeader"] {
        opacity: 1 !important;
    }
    /* Clearer expand control when the sidebar is collapsed. */
    [data-testid="stSidebarCollapsedControl"] {
        opacity: 1 !important;
        z-index: 1000 !important;
    }
    /* Safari iOS zooms focused inputs under 16px — keep form text at 16px. */
    input, textarea, select,
    [data-baseweb="input"] input,
    [data-baseweb="textarea"] textarea,
    [data-baseweb="select"],
    .stTextInput input,
    .stNumberInput input,
    .stTextArea textarea,
    .stSelectbox div,
    .stMultiSelect div {
        font-size: 16px !important;
    }
    </style>
    """


def install_app_shell_bridge():
    """Install top-window listeners via a tiny iframe (markdown cannot run JS)."""
    render_html_embed(_static_html("shell_bridge.html"), height=1)


def render_live_field_map(
    grid_points,
    user_lat,
    user_lon,
    center_lat,
    center_lon,
    zoom=19,
    height=520,
):
    # Active target is chosen client-side among the visible (non-visited) points.
    grid_id = (
        f"{float(st.session_state.get('preview_origin_lat', center_lat)):.7f},"
        f"{float(st.session_state.get('preview_origin_lon', center_lon)):.7f},"
        f"{current_grid_orientation_deg():.3f},"
        f"{float(st.session_state.get('grid_spacing', 2.0)):.3f},"
        f"{1 if st.session_state.get('grid_staggered', True) else 0},"
        f"{1 if st.session_state.get('grid_endless', True) else 0}"
    )
    config = {
        "grid_points": grid_points,
        "grid_id": grid_id,
        "target_point": None,
        "target_lat": None,
        "target_lon": None,
        "user_lat": user_lat,
        "user_lon": user_lon,
        "user_heading": None,
        "user_accuracy": None,
        "reach_tolerance_m": 0.015,
        "center_lat": center_lat,
        "center_lon": center_lon,
        "zoom": zoom,
    }
    html = (
        _static_html("live_field_map.html")
        .replace("__MAP_CONFIG_B64__", _encode_map_payload(config))
        .replace("__MAP_MESSAGE_TYPE__", json.dumps(MAP_MESSAGE_TYPE))
    )
    render_html_embed(html, height=height)


def render_live_nav_panel(height=460):
    status = get_latest_rtk_status()
    label = status.get("rtk_label") or "Waiting for GPS…"
    css = status.get("rtk_class") or "unknown"
    sats = status.get("num_sats")
    try:
        sats_i = int(sats) if sats is not None else 0
    except (TypeError, ValueError):
        sats_i = 0
    sats_note = f" · {sats_i} sats" if sats_i > 0 else ""
    html = (
        _static_html("live_nav_panel.html")
        .replace("__RTK_BAR_CLASS__", css)
        .replace("__RTK_BAR_LABEL__", f"{label}{sats_note}")
    )
    render_html_embed(html, height=height)
    # Match the seeded bar so the next poll only posts when quality changes.
    st.session_state.last_posted_rtk_status = (
        status.get("gps_qual"),
        status.get("rtk_label"),
        status.get("num_sats"),
    )


def invalidate_live_nav_panel():
    """Force the live nav iframe to remount and re-sync RTK status."""
    st.session_state.live_nav_panel_mounted = False
    st.session_state.pop("last_posted_rtk_status", None)


def post_user_position_update(user_lat, user_lon, session_key):
    """Push a GPS position to mounted maps without remounting them."""
    # Round to ~1 cm so RTK jitter doesn't trigger a new message every poll.
    position = (round(user_lat, 7), round(user_lon, 7))
    if st.session_state.get(session_key) == position:
        return
    st.session_state[session_key] = position
    post_map_message({
        "action": "updateUser",
        "user_lat": position[0],
        "user_lon": position[1],
    })


def post_endless_grid_window_if_needed(user_lat, user_lon):
    """Refresh the live endless lattice when the nearest-4 set changes."""
    if not st.session_state.get("grid_finalized"):
        return
    if not st.session_state.get("grid_endless", True):
        return
    df, center = build_active_grid_df(
        user_lat,
        user_lon,
        int(st.session_state.get("grid_dim_m", 4)),
        int(st.session_state.get("grid_dim_e", 4)),
        float(st.session_state.get("grid_spacing", 2.0)),
    )
    if center is not None and st.session_state.get("last_endless_window_key") == center:
        return
    st.session_state.last_endless_window_key = center
    st.session_state.grid_df = df
    post_map_message({
        "action": "updateGridPoints",
        "grid_points": grid_df_to_points(df),
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


_COMPLETE_NMEA_RE = re.compile(
    r"\$[A-Z0-9]{5},[^$\r\n]*\*[0-9A-Fa-f]{2}"
)


def _extract_complete_nmea_sentences(buffered):
    """Extract checksum-complete NMEA sentences without requiring CR/LF.

    Some high-rate receiver configurations expose a complete `$GNGGA...*HH`
    packet through USB before its line terminator arrives. Completion is
    unambiguous once the two checksum digits are present, so parse immediately
    instead of waiting for a newline or the following `$`.
    """
    matches = list(_COMPLETE_NMEA_RE.finditer(buffered))
    if matches:
        sentences = [match.group(0) for match in matches]
        remainder = buffered[matches[-1].end():].lstrip("\r\n")
        return sentences, remainder[-8192:]

    # Discard noise before the newest possible sentence start, but retain a
    # partial sentence for completion by the next serial read.
    last_start = buffered.rfind("$")
    if last_start >= 0:
        buffered = buffered[last_start:]
    return [], buffered[-8192:]


def _parse_gga_sentence(sentence):
    if not (sentence.startswith("$GNGGA") or sentence.startswith("$GPGGA")):
        return None
    try:
        return pynmea2.parse(sentence)
    except (pynmea2.ParseError, pynmea2.ChecksumError):
        return None


def _gga_lat_lon(msg):
    """Return (lat, lon) floats from a GGA message, or None if the fix is unusable.

    Truncated NMEA fields like '3726.' parse as messages but explode when
    pynmea2 converts DDDMM.MMM → decimal degrees.
    """
    if msg is None:
        return None
    try:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
    except (TypeError, ValueError, AttributeError):
        return None
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return None
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return None
    return lat, lon


def latest_gps_fix():
    """Latest validated (lat, lon), preferring a cached pair over re-reading the msg."""
    cached = st.session_state.get("last_gps_lat_lon")
    if cached is not None:
        return cached
    coords = _gga_lat_lon(st.session_state.get("last_gps_msg"))
    if coords is not None:
        st.session_state.last_gps_lat_lon = coords
    return coords

def _resolve_gps_serial_port():
    """Return the GPIO UART port, with common Pi serial aliases as fallback."""
    configured = Path(SERIAL_PORT)
    if configured.exists():
        return str(configured)

    # /dev/serial0 is the stable Pi UART symlink; fall back to common aliases
    # if the symlink is missing (e.g. serial console still claiming the port).
    for candidate in ("/dev/ttyAMA0", "/dev/ttyS0", "/dev/serial1"):
        if Path(candidate).exists():
            return candidate
    return SERIAL_PORT


def _open_gps_serial_if_needed():
    """Return the shared GNSS serial handle, opening it if needed (thread-safe)."""
    global _gps_serial, _gps_serial_port, _gps_serial_error
    with _gps_serial_lock:
        if _gps_serial is not None:
            try:
                if _gps_serial.is_open:
                    return _gps_serial
            except Exception:
                pass
        port = _resolve_gps_serial_port()
        try:
            _gps_serial = serial.Serial(port, BAUD_RATE, timeout=0.1)
            # Match ntrip-rover.py's flushInput()/flushOutput() on open.
            _gps_serial.reset_input_buffer()
            _gps_serial.reset_output_buffer()
            _gps_serial_port = port
            _gps_serial_error = None
        except Exception as exc:
            _gps_serial = None
            _gps_serial_port = port
            _gps_serial_error = f"{port}: {exc}"
        return _gps_serial


def _set_latest_gga(sentence):
    """Remember the most recent raw GGA sentence for upload back to the caster."""
    if not sentence or not sentence.startswith(("$GNGGA", "$GPGGA")):
        return
    global _latest_gga_sentence
    with _latest_gga_lock:
        _latest_gga_sentence = sentence


def _get_latest_gga_for_caster():
    """Latest GGA as CRLF-terminated bytes, or None if we have not seen one yet."""
    with _latest_gga_lock:
        sentence = _latest_gga_sentence
    if not sentence:
        return None
    if not sentence.endswith("\r\n"):
        sentence = sentence + "\r\n"
    return sentence.encode("ascii", errors="ignore")


def _ensure_gps_serial():
    """Open (or re-open) the GNSS serial port used for NMEA reads and RTCM writes."""
    if "last_gps_msg" not in st.session_state:
        st.session_state.last_gps_msg = None
    if "last_gps_lat_lon" not in st.session_state:
        st.session_state.last_gps_lat_lon = None

    st.session_state.gps_serial = _open_gps_serial_if_needed()


def _write_rtcm_to_gps(data):
    """Push RTCM correction bytes to the receiver (called from the NTRIP thread).

    Intentionally does NOT take the NMEA read lock: writing corrections is
    independent of reading NMEA on a full-duplex serial port, and making writes
    wait on blocking reads starves the base's RTCM stream (no RTK float).
    """
    if not data:
        return True
    ser = _open_gps_serial_if_needed()
    if ser is None:
        return False
    try:
        ser.write(data)
        ser.flush()
        return True
    except Exception:
        return False


def get_ntrip_status():
    """Return a UI-facing snapshot of NTRIP state (includes live stream health)."""
    # Revive the worker if a mountpoint is still wanted after a script reload.
    with ntrip_rt.lock:
        desired = ntrip_rt.desired
    if desired:
        _ensure_ntrip_thread()

    with ntrip_rt.lock:
        status = dict(ntrip_rt.status)
        desired = dict(ntrip_rt.desired) if ntrip_rt.desired else None
        thread = ntrip_rt.thread
        owner_session_id = ntrip_rt.owner_session_id

    last_rtcm_at = float(status.get("last_rtcm_at") or 0.0)
    age_s = (time.time() - last_rtcm_at) if last_rtcm_at > 0 else None
    active_key = ntrip_rt.target_key(desired)
    mountpoint = (desired or {}).get("mountpoint") or status.get("mountpoint")
    bytes_total = int(status.get("bytes_total") or 0)
    thread_alive = thread is not None and thread.is_alive()
    label = ntrip_rt.target_label(desired) if desired else None

    if not desired:
        status["state"] = "idle"
        status["message"] = "Not connected"
        status["mountpoint"] = None
        status["host"] = None
        status["port"] = None
        status["source"] = None
        status["desired_mountpoint"] = None
        status["desired_key"] = None
        status["desired"] = None
        status["owner_session_id"] = None
        status["streaming"] = False
        status["rtcm_age_s"] = age_s
        return status

    if (
        age_s is not None
        and age_s <= ntrip_rt.STREAM_STALE_S
        and status.get("state") in ("connected", "connecting")
    ):
        kb = bytes_total / 1024.0
        status["state"] = "connected"
        status["message"] = f"Streaming RTCM from {label} · {kb:.1f} KB received"
        status["streaming"] = True
    elif status.get("state") == "connected":
        status["message"] = (
            f"Connected to {label}, waiting for RTCM…"
            if age_s is None or age_s > ntrip_rt.STREAM_STALE_S
            else status.get("message") or f"Connected to {label}"
        )
        status["streaming"] = False
    elif status.get("state") == "idle" and desired:
        # Desired target survived a reload but status looked idle — recover.
        status["state"] = "connecting"
        status["message"] = (
            f"Reconnecting to {label}…"
            if thread_alive
            else f"Starting NTRIP for {label}…"
        )
        status["streaming"] = False
    else:
        status["streaming"] = False

    status["rtcm_age_s"] = age_s
    status["mountpoint"] = mountpoint
    status["host"] = desired.get("host")
    status["port"] = desired.get("port")
    status["source"] = desired.get("source")
    status["desired_mountpoint"] = mountpoint
    status["desired_key"] = active_key
    status["desired"] = desired
    status["owner_session_id"] = owner_session_id
    return status


def _set_ntrip_status(state, message, target=None):
    with ntrip_rt.lock:
        ntrip_rt.status["state"] = state
        ntrip_rt.status["message"] = message
        if target is not None:
            ntrip_rt.status["mountpoint"] = target.get("mountpoint")
            ntrip_rt.status["host"] = target.get("host")
            ntrip_rt.status["port"] = target.get("port")
            ntrip_rt.status["source"] = target.get("source")


def _note_rtcm_received(nbytes):
    """Record that RTCM bytes were successfully forwarded to the receiver."""
    if nbytes <= 0:
        return
    with ntrip_rt.lock:
        ntrip_rt.status["last_rtcm_at"] = time.time()
        ntrip_rt.status["bytes_total"] = int(
            ntrip_rt.status.get("bytes_total") or 0
        ) + int(nbytes)
        label = ntrip_rt.target_label(ntrip_rt.desired) or "caster"
        kb = ntrip_rt.status["bytes_total"] / 1024.0
        ntrip_rt.status["state"] = "connected"
        ntrip_rt.status["message"] = (
            f"Streaming RTCM from {label} · {kb:.1f} KB received"
        )


def _ntrip_request_headers(target):
    """Build an NTRIP request for RTK2GO or a local caster."""
    mountpoint = (target.get("mountpoint") or "").strip().lstrip("/")
    username = target.get("username")
    password = target.get("password")
    req = f"GET /{mountpoint} HTTP/1.0\r\n"
    req += "User-Agent: NTRIP PythonClient/1.0\r\n"
    # RTK2GO always authenticates with email:none. Local casters often need
    # no auth; only send Basic credentials when a username was provided.
    if username not in (None, ""):
        credentials = f"{username}:{password if password is not None else ''}"
        encoded_creds = base64.b64encode(credentials.encode("utf-8")).decode("utf-8")
        req += f"Authorization: Basic {encoded_creds}\r\n"
    req += "Ntrip-Version: Ntrip/2.0\r\n"
    req += "Connection: close\r\n\r\n"
    return req.encode("utf-8")


def _read_ntrip_headers(sock):
    """Read until end of HTTP headers; return (header_text, leftover_body_bytes).

    Same logic as ntrip-rover.py: the first TCP chunk often contains both the
    ICY/HTTP header and the start of the binary RTCM stream. Decoding the whole
    recv() as UTF-8 and discarding it drops those RTCM bytes and can keep the
    receiver from ever reaching float/fixed.
    """
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(1024)
        if not chunk:
            break
        buf += chunk
        if len(buf) > 16384:
            break
    header, sep, body = buf.partition(b"\r\n\r\n")
    if not sep:
        return buf.decode("utf-8", errors="ignore"), b""
    return header.decode("utf-8", errors="ignore"), body


def _normalize_ntrip_target(
    *,
    mountpoint,
    host=None,
    port=None,
    username=None,
    password=None,
    source="rtk2go",
):
    cleaned = (mountpoint or "").strip().lstrip("/")
    if not cleaned:
        raise ValueError("Mountpoint cannot be empty")
    source = (source or "rtk2go").strip().lower()
    if source not in ("rtk2go", "local"):
        raise ValueError("Caster source must be 'rtk2go' or 'local'")

    if source == "rtk2go":
        host = NTRIP_HOST
        port = NTRIP_PORT
        username = NTRIP_USER_EMAIL
        password = "none"
    else:
        host = (host or "").strip()
        if not host:
            raise ValueError("Local caster IP / hostname cannot be empty")
        try:
            port = int(port if port is not None else NTRIP_PORT)
        except (TypeError, ValueError) as exc:
            raise ValueError("Caster port must be an integer") from exc
        if not (1 <= port <= 65535):
            raise ValueError("Caster port must be between 1 and 65535")
        username = (username or "").strip()
        password = "" if password is None else str(password)

    return {
        "source": source,
        "host": host,
        "port": int(port),
        "mountpoint": cleaned,
        "username": username,
        "password": password,
    }


def _ntrip_stream_once(target, stop_event):
    """Connect to one caster target and relay RTCM until stop or error."""
    label = ntrip_rt.target_label(target)
    _set_ntrip_status("connecting", f"Connecting to {label}…", target)
    if _open_gps_serial_if_needed() is None:
        raise RuntimeError(
            f"Cannot open GNSS serial port {_gps_serial_port}: "
            f"{_gps_serial_error or 'unknown error'}"
        )

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(15.0)
    try:
        sock.connect((target["host"], int(target["port"])))
        sock.sendall(_ntrip_request_headers(target))
        response, leftover = _read_ntrip_headers(sock)
        if not (
            "ICY 200 OK" in response
            or "HTTP/1.0 200 OK" in response
            or "HTTP/1.1 200 OK" in response
        ):
            raise RuntimeError(f"Caster rejected mountpoint: {response.strip()[:200]}")

        _set_ntrip_status(
            "connected",
            f"Connected to {label}, waiting for RTCM…",
            target,
        )
        if leftover:
            if not _write_rtcm_to_gps(leftover):
                raise RuntimeError("GNSS serial port write failed")
            _note_rtcm_received(len(leftover))

        sock.settimeout(5.0)
        last_gga_upload = 0.0

        def upload_gga_if_due(force=False):
            """Report the rover's position to the caster every ~5s (keeps RTCM flowing)."""
            nonlocal last_gga_upload
            now = time.time()
            if not force and now - last_gga_upload < 5.0:
                return
            gga = _get_latest_gga_for_caster()
            if not gga:
                return
            try:
                sock.sendall(gga)
                last_gga_upload = now
            except OSError:
                # Main loop will detect the disconnect on the next recv().
                pass

        # Send an initial position (if we already have one) so casters that
        # gate RTCM on a client GGA start streaming immediately.
        upload_gga_if_due(force=True)

        while not stop_event.is_set():
            with ntrip_rt.lock:
                desired = ntrip_rt.desired
            if desired != target:
                break
            upload_gga_if_due()
            try:
                data = sock.recv(2048)
            except socket.timeout:
                continue
            if not data:
                raise RuntimeError("Connection closed by caster")
            if not _write_rtcm_to_gps(data):
                raise RuntimeError("GNSS serial port write failed")
            _note_rtcm_received(len(data))
    finally:
        try:
            sock.close()
        except Exception:
            pass


def _ntrip_worker():
    """Background loop: keep streaming the selected target, reconnecting on errors."""
    delay = 5
    while not ntrip_rt.stop.is_set():
        with ntrip_rt.lock:
            target = dict(ntrip_rt.desired) if ntrip_rt.desired else None
        if not target:
            _set_ntrip_status("idle", "Not connected", None)
            time.sleep(0.4)
            continue
        try:
            _ntrip_stream_once(target, ntrip_rt.stop)
            delay = 5
        except Exception as exc:
            with ntrip_rt.lock:
                still_wanted = ntrip_rt.desired == target
            if still_wanted and not ntrip_rt.stop.is_set():
                delay = min(delay * 2, 300)
                label = ntrip_rt.target_label(target)
                _set_ntrip_status(
                    "error",
                    f"{exc}. Reconnecting in {delay}s…",
                    target,
                )
                # Interruptible wait so Disconnect is responsive.
                for _ in range(int(delay * 10)):
                    if ntrip_rt.stop.is_set():
                        break
                    with ntrip_rt.lock:
                        if ntrip_rt.desired != target:
                            break
                    time.sleep(0.1)


def _ensure_ntrip_thread():
    with ntrip_rt.lock:
        if ntrip_rt.thread is not None and ntrip_rt.thread.is_alive():
            return
        ntrip_rt.stop.clear()
        ntrip_rt.thread = threading.Thread(
            target=_ntrip_worker,
            name="ntrip-rtcm",
            daemon=True,
        )
        ntrip_rt.thread.start()


def _browser_session_id():
    """Stable id for this browser tab's Streamlit session."""
    if "browser_session_id" not in st.session_state:
        st.session_state.browser_session_id = uuid.uuid4().hex
    return st.session_state.browser_session_id


def start_ntrip(
    mountpoint,
    *,
    host=None,
    port=None,
    username=None,
    password=None,
    source="rtk2go",
):
    """Start (or switch) NTRIP streaming for the given caster target."""
    target = _normalize_ntrip_target(
        mountpoint=mountpoint,
        host=host,
        port=port,
        username=username,
        password=password,
        source=source,
    )
    _ensure_ntrip_thread()
    with ntrip_rt.lock:
        ntrip_rt.desired = target
        ntrip_rt.owner_session_id = _browser_session_id()
        ntrip_rt.status["bytes_total"] = 0
        ntrip_rt.status["last_rtcm_at"] = 0.0
    _set_ntrip_status(
        "connecting",
        f"Connecting to {ntrip_rt.target_label(target)}…",
        target,
    )


def stop_ntrip():
    """Stop streaming corrections (worker thread stays idle for a later connect)."""
    with ntrip_rt.lock:
        ntrip_rt.desired = None
        ntrip_rt.owner_session_id = None
        ntrip_rt.status["last_rtcm_at"] = 0.0
        ntrip_rt.status["bytes_total"] = 0
        ntrip_rt.status["host"] = None
        ntrip_rt.status["port"] = None
        ntrip_rt.status["source"] = None
        ntrip_rt.status["mountpoint"] = None
    _set_ntrip_status("idle", "Not connected", None)


def poll_gps(timeout_s=0.15):
    """Read buffered serial data and return the latest valid GGA fix.

    Drain every byte currently waiting so high-rate NMEA output cannot build a
    backlog. Reading one line per pass can leave the displayed GGA several
    sentences behind when the receiver also emits GSA/GSV/RMC messages.
    """
    _ensure_gps_serial()
    ser = st.session_state.get("gps_serial")
    if ser is None:
        return st.session_state.get("last_gps_msg")

    deadline = time.time() + max(0.0, float(timeout_s))
    if "gps_rx_buffer" not in st.session_state:
        st.session_state.gps_rx_buffer = ""

    while True:
        lines = []
        waiting = 0
        # Only serialize NMEA readers against each other; RTCM writes run
        # concurrently on the same full-duplex fd and must never wait here.
        with _gps_read_lock:
            try:
                waiting = ser.in_waiting
            except Exception:
                break

            if waiting > 0:
                try:
                    # read(waiting) is non-blocking here and drains the backlog
                    # in one syscall instead of consuming one NMEA line at a time.
                    chunk = ser.read(waiting).decode("ascii", errors="replace")
                except Exception:
                    break
                buffered = st.session_state.gps_rx_buffer + chunk
                lines, st.session_state.gps_rx_buffer = (
                    _extract_complete_nmea_sentences(buffered)
                )

        if waiting <= 0:
            if st.session_state.get("last_gps_msg") is not None:
                break
            if time.time() >= deadline:
                break
            time.sleep(0.02)
            continue

        for line in lines:
            for sentence in _iter_nmea_sentences(line.strip()):
                msg = _parse_gga_sentence(sentence)
                coords = _gga_lat_lon(msg)
                if coords is not None:
                    st.session_state.last_gps_msg = msg
                    st.session_state.last_gps_lat_lon = coords
                    # Feed the NTRIP thread the rover position to report upstream.
                    _set_latest_gga(sentence)
                    _set_latest_gps_quality(msg)

        if time.time() >= deadline:
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
    # Freeze the red "you are here" marker during placement at the first fix.
    if "placement_user_lat" not in st.session_state:
        st.session_state.placement_user_lat = float(current_lat)
        st.session_state.placement_user_lon = float(current_lon)
    if "map_zoom" not in st.session_state:
        st.session_state.map_zoom = 19
    if "preview_orientation_deg" not in st.session_state:
        st.session_state.preview_orientation_deg = 0.0
    # Bumped whenever Python wants to force the placement map's view to the
    # preview origin (instead of leaving the user's pan/zoom alone).
    if "placement_view_seq" not in st.session_state:
        st.session_state.placement_view_seq = 0


ORIGIN_STEP_M = 5.0
_METERS_PER_DEG_LAT = 111320.0


def _lat_step_deg(step_m=ORIGIN_STEP_M):
    """Degrees of latitude equal to step_m meters."""
    return float(step_m) / _METERS_PER_DEG_LAT


def _lon_step_deg(lat, step_m=ORIGIN_STEP_M):
    """Degrees of longitude equal to step_m meters at the given latitude."""
    cos_lat = math.cos(math.radians(float(lat)))
    meters_per_deg_lon = _METERS_PER_DEG_LAT * max(abs(cos_lat), 1e-6)
    return float(step_m) / meters_per_deg_lon


def freeze_placement_user_position(lat, lon):
    """Pin the placement-screen user marker (no live tracking until finalize)."""
    st.session_state.placement_user_lat = float(lat)
    st.session_state.placement_user_lon = float(lon)


def placement_user_latlon():
    """Frozen user position shown on the placement map."""
    return (
        float(st.session_state.placement_user_lat),
        float(st.session_state.placement_user_lon),
    )


def post_placement_view(*, lat=None, lon=None, bearing=None, zoom=None):
    """Move the already-mounted placement map without a Streamlit remount."""
    payload = {"action": "setView"}
    if lat is not None:
        payload["lat"] = float(lat)
    if lon is not None:
        payload["lon"] = float(lon)
    if bearing is not None:
        payload["bearing"] = float(bearing) % 360.0
    if zoom is not None:
        payload["zoom"] = int(zoom)
    post_map_message(payload)


def queue_origin_latlon_inputs(lat, lon):
    """Stage absolute origin coords for the sidebar widgets (safe after they mount)."""
    st.session_state.pending_origin_lat = float(lat)
    st.session_state.pending_origin_lon = float(lon)


def apply_pending_origin_latlon_inputs():
    """Apply staged lat/lon to widget keys. Call only before number_input mounts."""
    if "pending_origin_lat" in st.session_state:
        st.session_state.origin_lat_input = float(
            st.session_state.pop("pending_origin_lat")
        )
    if "pending_origin_lon" in st.session_state:
        st.session_state.origin_lon_input = float(
            st.session_state.pop("pending_origin_lon")
        )


def apply_origin_latlon(lat, lon, *, sync_inputs=True, snap_map=True, remount_map=False):
    """Set the grid origin to an absolute lat/lon."""
    st.session_state.preview_origin_lat = float(lat)
    st.session_state.preview_origin_lon = float(lon)
    if sync_inputs:
        queue_origin_latlon_inputs(lat, lon)
    if remount_map:
        st.session_state.placement_view_seq += 1
    elif snap_map and not st.session_state.get("grid_finalized"):
        post_placement_view(
            lat=lat,
            lon=lon,
            bearing=float(st.session_state.get("preview_orientation_deg", 0.0)),
            zoom=st.session_state.get("map_zoom"),
        )


def _heading_delta_deg(a, b):
    """Smallest signed difference from a to b on a 0–360 circle."""
    return ((float(b) - float(a) + 180.0) % 360.0) - 180.0


def queue_grid_heading_input(orientation_deg):
    """Stage a heading for the sidebar widget (safe after the widget has mounted)."""
    st.session_state.pending_heading_from_map = float(orientation_deg) % 360.0


def apply_pending_grid_heading_input():
    """Apply a staged heading to the widget key. Call only before number_input mounts."""
    if "pending_heading_from_map" not in st.session_state:
        return
    st.session_state.grid_heading_input = (
        float(st.session_state.pop("pending_heading_from_map")) % 360.0
    )


def apply_grid_heading_deg(orientation_deg, *, sync_input=True, remount_map=False):
    """Set grid heading and rotate the placement map screen-up to match."""
    orientation = float(orientation_deg) % 360.0
    st.session_state.preview_orientation_deg = orientation
    if sync_input:
        # Never write the widget key here — it may already be mounted (sidebar
        # apply path) or live in another fragment (map rotate). Stage instead.
        queue_grid_heading_input(orientation)
    if remount_map:
        st.session_state.placement_view_seq += 1
    elif not st.session_state.get("grid_finalized"):
        post_placement_view(
            lat=float(st.session_state.preview_origin_lat),
            lon=float(st.session_state.preview_origin_lon),
            bearing=orientation,
            zoom=st.session_state.get("map_zoom"),
        )


def reset_grid_to_current_position(current_lat, current_lon, line_count_m, line_count_e, spacing):
    freeze_placement_user_position(current_lat, current_lon)
    apply_origin_latlon(
        current_lat, current_lon, sync_inputs=True, snap_map=False, remount_map=True
    )
    apply_grid_heading_deg(0.0, remount_map=False)
    # Remount picks up heading 0 with the new view_seq origin snap.
    st.session_state.preview_orientation_deg = 0.0
    queue_grid_heading_input(0.0)
    st.session_state.pop("last_endless_window_key", None)
    if st.session_state.grid_finalized:
        st.session_state.grid_orientation_deg = 0.0
        df, center = build_active_grid_df(
            current_lat, current_lon, line_count_m, line_count_e, spacing
        )
        st.session_state.grid_df = df
        if center is not None:
            st.session_state.last_endless_window_key = center

def finalize_grid_location(line_count_m, line_count_e, spacing):
    orientation = float(st.session_state.get("preview_orientation_deg", 0.0)) % 360.0
    st.session_state.preview_orientation_deg = orientation
    # Frozen copy used by save so orientation can't drift after finalize.
    st.session_state.grid_orientation_deg = orientation
    user_coords = latest_gps_fix()
    if user_coords is None:
        user_lat = float(st.session_state.preview_origin_lat)
        user_lon = float(st.session_state.preview_origin_lon)
    else:
        user_lat, user_lon = user_coords
    df, center = build_active_grid_df(
        user_lat, user_lon, line_count_m, line_count_e, spacing
    )
    st.session_state.grid_df = df
    if center is not None:
        st.session_state.last_endless_window_key = center
    else:
        st.session_state.pop("last_endless_window_key", None)
    st.session_state.grid_finalized = True

def current_grid_orientation_deg():
    """Orientation baked into the finalized grid, else the live placement preview."""
    if st.session_state.get("grid_finalized") and "grid_orientation_deg" in st.session_state:
        return float(st.session_state.grid_orientation_deg) % 360.0
    return float(st.session_state.get("preview_orientation_deg", 0.0)) % 360.0

# --- GRID SAVE / LOAD ---
GRID_SAVE_DIR = Path(__file__).parent / "saved_grids"

def grid_save_safe_name(name):
    """Sanitize a user-facing grid name into a filesystem-safe stem."""
    safe_name = re.sub(r"[^\w-]+", "_", (name or "").strip()).strip("_")
    if not safe_name:
        safe_name = time.strftime("grid_%Y%m%d_%H%M%S")
    return safe_name

@st.cache_data(ttl=5.0, show_spinner=False)
def _list_saved_grid_names():
    """Cached basenames only — Path objects are not reliably cacheable."""
    if not GRID_SAVE_DIR.is_dir():
        return ()
    return tuple(sorted(p.name for p in GRID_SAVE_DIR.glob("*.json")))


def list_saved_grids():
    """Return saved grid paths (cached briefly — cleared on save)."""
    return [GRID_SAVE_DIR / name for name in _list_saved_grid_names()]


def save_grid_to_file(name):
    """Save origin, orientation, and grid parameters as JSON. Returns filename."""
    GRID_SAVE_DIR.mkdir(exist_ok=True)
    safe_name = grid_save_safe_name(name)
    orientation = current_grid_orientation_deg()
    payload = {
        "origin_lat": float(st.session_state.preview_origin_lat),
        "origin_lon": float(st.session_state.preview_origin_lon),
        # Degrees clockwise from north for the row axis (toward map "up"
        # when the grid was placed). 0 = north-up.
        "orientation_deg": orientation,
        "rows": int(st.session_state.grid_dim_m),
        "cols": int(st.session_state.grid_dim_e),
        "spacing_m": float(st.session_state.grid_spacing),
        "staggered": bool(st.session_state.get("grid_staggered", True)),
        "endless": bool(st.session_state.get("grid_endless", True)),
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    file_path = GRID_SAVE_DIR / f"{safe_name}.json"
    file_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    _list_saved_grid_names.clear()
    return file_path.name, orientation

def saved_grid_exists(name):
    return (GRID_SAVE_DIR / f"{grid_save_safe_name(name)}.json").is_file()


def delete_saved_grid(name):
    """Delete a saved grid JSON file. Returns True if a file was removed."""
    path = GRID_SAVE_DIR / f"{grid_save_safe_name(name)}.json"
    if not path.is_file():
        return False
    path.unlink()
    _list_saved_grid_names.clear()
    return True

def apply_pending_grid_load():
    """Apply a grid load staged by the sidebar's Load button.

    Widget values (rows/cols/spacing) can only be written to session state
    before their widgets are instantiated, so the Load button stages the data
    and reruns; this applies it at the top of the next full run.
    """
    data = st.session_state.pop("pending_grid_load", None)
    if data is None:
        return
    try:
        origin_lat = float(data["origin_lat"])
        origin_lon = float(data["origin_lon"])
        rows = max(1, int(data["rows"]))
        cols = max(1, int(data["cols"]))
        spacing = max(0.5, float(data["spacing_m"]))
        # Older saves omit orientation; treat them as north-up.
        orientation = float(data.get("orientation_deg", 0.0)) % 360.0
        staggered = bool(data.get("staggered", True))
        endless = bool(data.get("endless", False))
    except (KeyError, TypeError, ValueError):
        st.error("That saved grid file is invalid and can't be loaded.")
        return
    st.session_state.grid_dim_m = rows
    st.session_state.grid_dim_e = cols
    st.session_state.grid_spacing = spacing
    st.session_state.grid_staggered = staggered
    st.session_state.grid_endless = endless
    st.session_state.preview_origin_lat = origin_lat
    st.session_state.preview_origin_lon = origin_lon
    queue_origin_latlon_inputs(origin_lat, origin_lon)
    st.session_state.preview_orientation_deg = orientation
    st.session_state.grid_orientation_deg = orientation
    queue_grid_heading_input(orientation)
    st.session_state.pop("last_endless_window_key", None)
    # Skip placement — go straight to live field navigation.
    finalize_grid_location(rows, cols, spacing)
    st.session_state.live_map_mounted = False
    invalidate_live_nav_panel()

def apply_pending_saved_grid_choice():
    """Select a newly saved grid in the sidebar before its selectbox mounts."""
    choice = st.session_state.pop("pending_saved_grid_choice", None)
    if choice is not None:
        st.session_state.saved_grid_choice = choice


def apply_pending_grid_save_name():
    """Apply a staged Grid name textbox value before the widget mounts."""
    if "pending_grid_save_name" not in st.session_state:
        return
    st.session_state.grid_save_name = st.session_state.pop("pending_grid_save_name")

# --- STREAMLIT UI LAYOUT ---
st.set_page_config(page_title="RTK Live Map Guide", layout="wide")
st.title("RTK Live Map Guide")

st.markdown(_app_shell_css(), unsafe_allow_html=True)
install_app_shell_bridge()

# Prefer a cached fix; only briefly drain the serial port. Avoid long blocking
# waits so the first page paint stays responsive on the Pi.
poll_gps(timeout_s=0.05 if st.session_state.get("grid_finalized") else 0.0)
coords = latest_gps_fix()
if coords is None:
    poll_gps(timeout_s=0.25)
    coords = latest_gps_fix()
if coords is None:
    if _gps_serial_error:
        st.error(f"Cannot read GPS serial port: {_gps_serial_error}")
    else:
        st.warning(
            f"Waiting for a valid GPS fix from the receiver on {_gps_serial_port}..."
        )

    @st.fragment(run_every=0.5)
    def _wait_for_first_fix():
        poll_gps(timeout_s=0.2)
        if latest_gps_fix() is not None:
            st.rerun(scope="app")
        st.caption("Listening for GGA…")

    _wait_for_first_fix()
    st.stop()

current_lat, current_lon = coords
init_placement_state(current_lat, current_lon)
apply_pending_grid_load()
apply_pending_saved_grid_choice()
apply_pending_grid_save_name()


with st.sidebar:
    def _sync_grid_config_from_session():
        """Push layout/view changes to the placement map (and live lattice if finalized)."""
        if "grid_endless" not in st.session_state:
            st.session_state.grid_endless = True
        if "grid_staggered" not in st.session_state:
            st.session_state.grid_staggered = True
        if "grid_dim_m" not in st.session_state:
            st.session_state.grid_dim_m = 4
        if "grid_dim_e" not in st.session_state:
            st.session_state.grid_dim_e = 4
        if "grid_spacing" not in st.session_state:
            st.session_state.grid_spacing = 2.0

        spacing = float(st.session_state.grid_spacing)
        staggered = bool(st.session_state.grid_staggered)
        endless = bool(st.session_state.grid_endless)
        # Endless placement preview is always a fixed 4×4 neighborhood.
        if endless:
            grid_lines_m, grid_lines_e = 4, 4
        else:
            grid_lines_m = int(st.session_state.grid_dim_m)
            grid_lines_e = int(st.session_state.grid_dim_e)
        grid_config = (
            grid_lines_m,
            grid_lines_e,
            spacing,
            staggered,
            endless,
        )
        previous_config = st.session_state.get("last_grid_config_sent")
        if previous_config == grid_config:
            return
        st.session_state.last_grid_config_sent = grid_config
        post_map_message({
            "action": "updateGrid",
            "dim_m": grid_config[0],
            "dim_e": grid_config[1],
            "spacing_m": grid_config[2],
            "staggered": grid_config[3],
            "endless": grid_config[4],
        })
        # After finalize, regenerate the live lattice when layout toggles change.
        if st.session_state.get("grid_finalized") and previous_config is not None:
            latest_coords = latest_gps_fix()
            if latest_coords is None:
                user_lat = float(st.session_state.preview_origin_lat)
                user_lon = float(st.session_state.preview_origin_lon)
            else:
                user_lat, user_lon = latest_coords
            st.session_state.pop("last_endless_window_key", None)
            df, center = build_active_grid_df(
                user_lat, user_lon, grid_lines_m, grid_lines_e, spacing
            )
            st.session_state.grid_df = df
            if center is not None:
                st.session_state.last_endless_window_key = center
            st.session_state.live_map_mounted = False
            invalidate_live_nav_panel()
            st.rerun(scope="app")

    st.header("Grid Settings")

    @st.fragment
    def render_grid_settings(current_lat, current_lon):
        if "grid_endless" not in st.session_state:
            st.session_state.grid_endless = True
        if "grid_staggered" not in st.session_state:
            st.session_state.grid_staggered = True

        endless = st.checkbox(
            "Endless grid",
            key="grid_endless",
            help=(
                "Infinite lattice locked at finalize. The live field view displays "
                "the four nearest lattice points. Visited points stay yellow. "
                "Placement preview uses a fixed 4×4 neighborhood."
            ),
        )
        st.checkbox(
            "Staggered grid",
            key="grid_staggered",
            help="Offset every other row by half a spacing (like brickwork).",
        )

        if not endless:
            st.number_input("Rows", min_value=1, value=4, key="grid_dim_m")
            st.number_input("Columns", min_value=1, value=4, key="grid_dim_e")
        else:
            if "grid_dim_m" not in st.session_state:
                st.session_state.grid_dim_m = 4
            if "grid_dim_e" not in st.session_state:
                st.session_state.grid_dim_e = 4
            st.caption("Endless mode uses a fixed 4×4 placement preview.")

        spacing = st.number_input(
            "Spacing (Meters)",
            min_value=0.5,
            value=4.0,
            step=0.5,
            key="grid_spacing",
            help="Distance between neighboring grid points.",
        )

        if not st.session_state.get("grid_finalized"):
            apply_pending_origin_latlon_inputs()
            stored_lat = float(st.session_state.preview_origin_lat)
            stored_lon = float(st.session_state.preview_origin_lon)
            if "origin_lat_input" not in st.session_state:
                st.session_state.origin_lat_input = stored_lat
            if "origin_lon_input" not in st.session_state:
                st.session_state.origin_lon_input = stored_lon
            st.caption(
                "Grid origin as absolute latitude / longitude. "
                f"The +/- buttons step by {ORIGIN_STEP_M:.0f} meters."
            )
            lat_in = st.number_input(
                "Origin latitude",
                min_value=-90.0,
                max_value=90.0,
                step=_lat_step_deg(),
                format="%.7f",
                key="origin_lat_input",
                help=(
                    "Absolute latitude of the grid origin. "
                    f"Each step moves about {ORIGIN_STEP_M:.0f} meters north/south."
                ),
            )
            lon_in = st.number_input(
                "Origin longitude",
                min_value=-180.0,
                max_value=180.0,
                step=_lon_step_deg(lat_in),
                format="%.7f",
                key="origin_lon_input",
                help=(
                    "Absolute longitude of the grid origin. "
                    f"Each step moves about {ORIGIN_STEP_M:.0f} meters east/west "
                    "at the current latitude."
                ),
            )
            if (
                abs(float(lat_in) - stored_lat) > 1e-9
                or abs(float(lon_in) - stored_lon) > 1e-9
            ):
                apply_origin_latlon(
                    float(lat_in),
                    float(lon_in),
                    sync_inputs=False,
                    snap_map=True,
                    remount_map=False,
                )

            external = float(st.session_state.get("preview_orientation_deg", 0.0)) % 360.0
            apply_pending_grid_heading_input()
            if "grid_heading_input" not in st.session_state:
                st.session_state.grid_heading_input = external

            heading = st.number_input(
                "Grid heading (°)",
                min_value=0.0,
                max_value=359.9,
                step=0.5,
                key="grid_heading_input",
                help=(
                    "Degrees clockwise from north for the grid row axis "
                    "(points toward the top of the phone screen). "
                    "Changing this rotates the map so the grid stays vertically aligned."
                ),
            )
            typed = float(heading) % 360.0
            if abs(_heading_delta_deg(external, typed)) > 0.05:
                # Live-rotate the mounted map; do not full-app remount.
                apply_grid_heading_deg(typed, sync_input=False, remount_map=False)
        else:
            st.caption(
                f"Locked origin: "
                f"{float(st.session_state.preview_origin_lat):.7f}°, "
                f"{float(st.session_state.preview_origin_lon):.7f}° "
                f"· heading {current_grid_orientation_deg():.1f}° · "
                f"spacing {float(st.session_state.get('grid_spacing', 2.0)):.1f} m "
                "(Return to grid creation to change)."
            )

        if st.button("Reset grid to current position", use_container_width=True):
            latest_coords = latest_gps_fix()
            if latest_coords is not None:
                current_lat, current_lon = latest_coords
            reset_grid_to_current_position(
                current_lat,
                current_lon,
                int(st.session_state.get("grid_dim_m", 4)),
                int(st.session_state.get("grid_dim_e", 4)),
                float(spacing),
            )
            st.session_state.live_map_mounted = False
            invalidate_live_nav_panel()
            st.rerun(scope="app")

        _sync_grid_config_from_session()

    render_grid_settings(current_lat, current_lon)

    st.header("NTRIP")

    # During placement, poll NTRIP status less often so sidebar buttons stay snappy.
    _ntrip_refresh_s = 1.0 if st.session_state.get("grid_finalized") else 3.0

    @st.fragment(run_every=_ntrip_refresh_s)
    def render_ntrip_controls():
        # Keep the serial RX drained (and NTRIP GGA fed) without a busy placement loop.
        poll_gps(timeout_s=0.0)
        status = get_ntrip_status()
        active = status.get("desired")
        active_key = status.get("desired_key")

        if "ntrip_source" not in st.session_state:
            st.session_state.ntrip_source = (
                "Local base"
                if (active or {}).get("source") == "local"
                else "RTK2GO"
            )
        if st.session_state.get("ntrip_mountpoint_input") is None:
            st.session_state.ntrip_mountpoint_input = (
                (active or {}).get("mountpoint") or NTRIP_DEFAULT_MOUNTPOINT
            )
        if "ntrip_local_host" not in st.session_state:
            st.session_state.ntrip_local_host = (
                ((active or {}).get("host") or "")
                if (active or {}).get("source") == "local"
                else ""
            )
        if "ntrip_local_port" not in st.session_state:
            st.session_state.ntrip_local_port = int(
                (active or {}).get("port") or NTRIP_PORT
            )
        if "ntrip_local_user" not in st.session_state:
            st.session_state.ntrip_local_user = (
                ((active or {}).get("username") or "")
                if (active or {}).get("source") == "local"
                else ""
            )
        if "ntrip_local_pass" not in st.session_state:
            st.session_state.ntrip_local_pass = (
                ((active or {}).get("password") or "")
                if (active or {}).get("source") == "local"
                else ""
            )

        source_label = st.radio(
            "Caster source",
            ["RTK2GO", "Local base"],
            key="ntrip_source",
            horizontal=True,
            help=(
                "RTK2GO uses the public caster. Local base connects directly to "
                "an NTRIP caster on your LAN by IP address."
            ),
        )
        source = "local" if source_label == "Local base" else "rtk2go"

        if source == "local":
            host = st.text_input(
                "Caster IP / hostname",
                key="ntrip_local_host",
                placeholder="192.168.1.50",
                help="Local NTRIP caster address (your base station or LAN caster).",
            )
            port = st.number_input(
                "Port",
                min_value=1,
                max_value=65535,
                step=1,
                key="ntrip_local_port",
            )
            username = st.text_input(
                "Username (optional)",
                key="ntrip_local_user",
                help="Leave blank if your local caster does not require auth.",
            )
            password = st.text_input(
                "Password (optional)",
                type="password",
                key="ntrip_local_pass",
            )
            mount_help = "Mountpoint advertised by your local caster."
        else:
            host = NTRIP_HOST
            port = NTRIP_PORT
            username = NTRIP_USER_EMAIL
            password = "none"
            mount_help = f"RTK2GO caster mountpoint on {NTRIP_HOST}:{NTRIP_PORT}"

        mountpoint = st.text_input(
            "Mountpoint",
            key="ntrip_mountpoint_input",
            help=mount_help,
        )
   

        try:
            requested_target = _normalize_ntrip_target(
                mountpoint=mountpoint,
                host=host,
                port=port,
                username=username,
                password=password,
                source=source,
            )
            requested_key = ntrip_rt.target_key(requested_target)
        except ValueError:
            requested_target = None
            requested_key = None

        pending_switch = st.session_state.get("ntrip_pending_switch")

        if active_key and status.get("owner_session_id") != _browser_session_id():
            st.info(
                f"This Pi is already streaming from **{active_key}**. "
                "All connected devices share that single NTRIP session."
            )

        col_connect, col_disconnect = st.columns(2)
        with col_connect:
            connect_label = "Connect"
            if active_key and requested_key and requested_key != active_key:
                connect_label = "Switch station…"
            elif active_key and requested_key == active_key:
                connect_label = "Reconnect"
            if st.button(connect_label, use_container_width=True, type="primary"):
                try:
                    if requested_target is None:
                        requested_target = _normalize_ntrip_target(
                            mountpoint=mountpoint,
                            host=host,
                            port=port,
                            username=username,
                            password=password,
                            source=source,
                        )
                        requested_key = ntrip_rt.target_key(requested_target)
                    if active_key and requested_key != active_key:
                        st.session_state.ntrip_pending_switch = requested_target
                        st.rerun(scope="fragment")
                    st.session_state.pop("ntrip_pending_switch", None)
                    start_ntrip(
                        requested_target["mountpoint"],
                        host=requested_target["host"],
                        port=requested_target["port"],
                        username=requested_target["username"],
                        password=requested_target["password"],
                        source=requested_target["source"],
                    )
                except ValueError as exc:
                    st.error(str(exc))
        with col_disconnect:
            if st.button("Disconnect", use_container_width=True):
                st.session_state.pop("ntrip_pending_switch", None)
                stop_ntrip()

        pending_key = ntrip_rt.target_key(pending_switch) if pending_switch else None
        if pending_key and active_key and pending_key != active_key:
            st.warning(
                f"Switch the shared stream from **{active_key}** to "
                f"**{pending_key}**? This changes the caster for every "
                "device using this Pi."
            )
            col_yes, col_no = st.columns(2)
            with col_yes:
                if st.button("Yes, switch", use_container_width=True, type="primary"):
                    try:
                        start_ntrip(
                            pending_switch["mountpoint"],
                            host=pending_switch["host"],
                            port=pending_switch["port"],
                            username=pending_switch["username"],
                            password=pending_switch["password"],
                            source=pending_switch["source"],
                        )
                        st.session_state.pop("ntrip_pending_switch", None)
                        st.rerun(scope="fragment")
                    except ValueError as exc:
                        st.error(str(exc))
            with col_no:
                if st.button("Cancel", use_container_width=True):
                    st.session_state.pop("ntrip_pending_switch", None)
                    st.rerun(scope="fragment")

        state = status.get("state", "idle")
        message = status.get("message", "")
        streaming = bool(status.get("streaming"))
        if streaming or state == "connected":
            st.success(message)
        elif state == "connecting":
            st.info(message)
        elif state == "error":
            st.warning(message)
        else:
            st.caption(message)

        if source == "local":
            st.caption("Local caster · auth only sent when a username is set")
        else:
            st.caption(
                f"Caster: {NTRIP_HOST}:{NTRIP_PORT} · user: {NTRIP_USER_EMAIL}"
            )

    render_ntrip_controls()

    st.header("Saved grids")

    @st.fragment
    def render_saved_grids():
        saved_grids = list_saved_grids()
        if saved_grids:
            chosen_grid = st.selectbox(
                "Saved grids",
                [f.stem for f in saved_grids],
                key="saved_grid_choice",
                label_visibility="collapsed",
            )
            if st.button("Load grid", use_container_width=True):
                try:
                    grid_data = json.loads(
                        (GRID_SAVE_DIR / f"{chosen_grid}.json").read_text(
                            encoding="utf-8"
                        )
                    )
                except (OSError, json.JSONDecodeError):
                    st.error("Could not read that grid file.")
                else:
                    st.session_state.pending_grid_load = grid_data
                    st.rerun(scope="app")

            pending_delete = st.session_state.get("grid_delete_confirm")
            if pending_delete and pending_delete != chosen_grid:
                st.session_state.pop("grid_delete_confirm", None)
                pending_delete = None
            if st.button("Delete grid", use_container_width=True):
                if pending_delete != chosen_grid:
                    st.session_state.grid_delete_confirm = chosen_grid
                    st.rerun(scope="fragment")
                st.session_state.pop("grid_delete_confirm", None)
                if delete_saved_grid(chosen_grid):
                    remaining = [p.stem for p in list_saved_grids()]
                    next_name = remaining[0] if remaining else ""
                    # Widget keys must be written before the widgets remount.
                    st.session_state.pending_grid_save_name = next_name
                    if remaining:
                        st.session_state.pending_saved_grid_choice = next_name
                    else:
                        st.session_state.pop("saved_grid_choice", None)
                        st.session_state.pop("pending_saved_grid_choice", None)
                    st.rerun(scope="app")
                else:
                    st.error("That grid file was already missing.")
            # Keep permanent room under Delete so the confirm warning can
            # appear without shoving controls off the bottom of the sidebar.
            if pending_delete == chosen_grid:
                st.warning(
                    f'Delete "{chosen_grid}"? Click Delete again to confirm.'
                )
            st.markdown(
                "<div aria-hidden='true' style='min-height:5.5rem'></div>",
                unsafe_allow_html=True,
            )
        else:
            st.caption("No saved grids yet.")

    render_saved_grids()

if not st.session_state.grid_finalized:
    st.subheader("Position Your Grid")
    st.caption(
        "Pan and zoom to place the origin, or set **latitude / longitude** "
        f"(+/- steps ≈ {ORIGIN_STEP_M:.0f} m) and **Grid heading** in the sidebar. "
        "Your position marker stays fixed at the first GPS reading until you finalize. "
        "The grid origin (green) stays at the map center. "
        "Click **Finalize grid location** to confirm."
    )

    # The map lives in a fragment so each pan-end (which reports the new
    # origin as a component value) reruns only this block, not the whole
    # script. That keeps reruns fast on the Pi and the UI steady.
    @st.fragment
    def placement_map_fragment():
        placement_endless = bool(st.session_state.get("grid_endless", True))
        if placement_endless:
            placement_rows, placement_cols = 4, 4
        else:
            placement_rows = int(st.session_state.get("grid_dim_m", 4))
            placement_cols = int(st.session_state.get("grid_dim_e", 4))
        user_lat, user_lon = placement_user_latlon()
        map_state = render_placement_map(
            center_lat=st.session_state.preview_origin_lat,
            center_lon=st.session_state.preview_origin_lon,
            zoom=st.session_state.map_zoom,
            line_count_m=placement_rows,
            line_count_e=placement_cols,
            spacing=float(st.session_state.get("grid_spacing", 2.0)),
            user_lat=user_lat,
            user_lon=user_lon,
            height=520,
            view_seq=st.session_state.placement_view_seq,
            bearing=float(st.session_state.get("preview_orientation_deg", 0.0)),
            staggered=bool(st.session_state.get("grid_staggered", True)),
            endless=placement_endless,
        )
        # The component reports the map center after every pan/zoom/rotate.
        # Ignore values tagged with an old view_seq: they predate a "Reset
        # grid to current position" snap and would override it.
        if (
            map_state is not None
            and map_state.get("seq") == st.session_state.placement_view_seq
        ):
            st.session_state.preview_origin_lat = float(map_state["lat"])
            st.session_state.preview_origin_lon = float(map_state["lon"])
            st.session_state.map_zoom = int(map_state["zoom"])
            # Keep lat/lon inputs aligned with a manual map pan on the next
            # sidebar interaction (avoid a full-app remount here).
            queue_origin_latlon_inputs(
                st.session_state.preview_origin_lat,
                st.session_state.preview_origin_lon,
            )

            if map_state.get("bearing") is not None:
                bearing = float(map_state["bearing"]) % 360.0
                st.session_state.preview_orientation_deg = bearing
                widget_heading = st.session_state.get("grid_heading_input")
                if widget_heading is None or (
                    abs(_heading_delta_deg(float(widget_heading), bearing)) > 0.05
                ):
                    queue_grid_heading_input(bearing)

    placement_map_fragment()

    col_info, col_finalize = st.columns([2, 1])
    with col_info:
        st.markdown(
            "**Legend:** green = grid origin · blue = preview points · red = your position  \n"
        )
    with col_finalize:
        if st.button("Finalize grid location", type="primary", use_container_width=True):
            with st.spinner("Finalizing grid..."):
                finalize_grid_location(
                    int(st.session_state.grid_dim_m),
                    int(st.session_state.grid_dim_e),
                    float(st.session_state.grid_spacing),
                )
                st.session_state.live_map_mounted = False
                invalidate_live_nav_panel()
            st.rerun()

else:
    df = st.session_state.grid_df
    grid_points = grid_df_to_points(df)

    st.subheader("Live Navigation")

    if not st.session_state.get("live_nav_panel_mounted"):
        render_live_nav_panel(height=520)
        st.session_state.live_nav_panel_mounted = True
        # Remount can race the BroadcastChannel listener; force a follow-up
        # push so the bar does not stick on the HTML default.
        post_rtk_status_update(force=True)

    @st.fragment(run_every=0.25)
    def refresh_live_user_marker():
        poll_gps(timeout_s=0.1)
        coords = latest_gps_fix()
        post_rtk_status_update()
        if coords is None:
            return
        post_user_position_update(
            coords[0],
            coords[1],
            "live_user_pos",
        )
        post_endless_grid_window_if_needed(coords[0], coords[1])

    refresh_live_user_marker()

    st.markdown("##### Field map")
    st.caption(
        "Secondary map for skip-toggles and context. "
        "Compass-follow rotates the view while grid points stay locked to the ground."
    )
    if not st.session_state.get("live_map_mounted"):
        render_live_field_map(
            grid_points=grid_points,
            user_lat=current_lat,
            user_lon=current_lon,
            center_lat=current_lat,
            center_lon=current_lon,
            zoom=19,
            height=260,
        )
        st.session_state.live_map_mounted = True

    col_adjust, col_save = st.columns(2)
    with col_adjust:
        if st.button("Return to grid creation", use_container_width=True):
            st.session_state.grid_finalized = False
            # Keep the finalized orientation when returning to placement.
            if "grid_orientation_deg" in st.session_state:
                st.session_state.preview_orientation_deg = float(
                    st.session_state.grid_orientation_deg
                ) % 360.0
            # Re-freeze the red marker at the current fix for the placement screen.
            latest_coords = latest_gps_fix()
            if latest_coords is not None:
                freeze_placement_user_position(latest_coords[0], latest_coords[1])
            # Snap the placement map back to the finalized origin when it remounts.
            st.session_state.placement_view_seq += 1
            st.session_state.live_map_mounted = False
            invalidate_live_nav_panel()
            st.rerun()
    with col_save:
        # Keep name entry / overwrite warnings in a fragment so typing a name
        # doesn't full-rerun the page and strip the already-mounted live map.
        @st.fragment
        def render_grid_save_controls():
            save_name = st.text_input(
                "Grid name",
                value=time.strftime("grid_%Y%m%d_%H%M%S"),
                key="grid_save_name",
                label_visibility="collapsed",
                placeholder="Grid name",
            )
            safe_name = grid_save_safe_name(save_name)
            pending_overwrite = st.session_state.get("grid_overwrite_confirm")
            # Changing the name cancels a pending overwrite confirmation.
            if pending_overwrite and pending_overwrite != safe_name:
                st.session_state.pop("grid_overwrite_confirm", None)
                pending_overwrite = None

            if pending_overwrite == safe_name:
                st.warning(
                    f'"{safe_name}" already exists. Click Save again to overwrite it.'
                )

            if st.button("Save Grid Location", use_container_width=True):
                if saved_grid_exists(save_name) and pending_overwrite != safe_name:
                    st.session_state.grid_overwrite_confirm = safe_name
                    # Fragment-only: a full app remount would clear the live
                    # map iframe while live_map_mounted stayed True, so the
                    # map would never be re-rendered and would "disappear".
                    st.rerun(scope="fragment")
                st.session_state.pop("grid_overwrite_confirm", None)
                saved_file, orientation = save_grid_to_file(save_name)
                # Persist across the app-scoped rerun so the success toast and
                # the sidebar "Saved grids" list both refresh; remount the live
                # map because a full rerun would otherwise skip it.
                st.session_state.grid_save_flash = (
                    f"Grid saved to saved_grids/{saved_file}."
                )
                st.session_state.pending_saved_grid_choice = Path(saved_file).stem
                st.session_state.live_map_mounted = False
                invalidate_live_nav_panel()
                st.rerun(scope="app")

            flash = st.session_state.pop("grid_save_flash", None)
            if flash:
                st.success(flash)

        render_grid_save_controls()
