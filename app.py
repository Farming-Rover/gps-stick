import base64
import json
import math
import re
import socket
import threading
import time
import uuid
from pathlib import Path

import pandas as pd
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
    return pd.DataFrame(grid)


def generate_nearest_endless_grid_point(
    origin_lat,
    origin_lon,
    user_lat,
    user_lon,
    spacing_meters,
    orientation_deg=0,
    staggered=True,
):
    """Return only the closest point in the infinite, oriented lattice."""
    row_m, col_m = _local_row_col_meters(
        origin_lat, origin_lon, user_lat, user_lon, orientation_deg
    )
    center_r, center_c = _nearest_lattice_indices(
        row_m, col_m, spacing_meters, staggered
    )
    point_lat, point_lon = _grid_point_latlon(
        origin_lat,
        origin_lon,
        center_r,
        center_c,
        spacing_meters,
        orientation_deg,
        staggered,
    )
    grid = [{
        "Point": f"R{center_r}C{center_c}",
        "lat": point_lat,
        "lon": point_lon,
        "color": "#1E90FF",
        "size": GRID_MARKER_SIZE_M,
    }]
    return pd.DataFrame(grid), (center_r, center_c)


def grid_df_to_points(df):
    return [
        {"point": row["Point"], "lat": float(row["lat"]), "lon": float(row["lon"])}
        for _, row in df.iterrows()
    ]


def build_active_grid_df(user_lat, user_lon, line_count_m, line_count_e, spacing):
    """Build the active grid DataFrame from current session settings."""
    origin_lat = float(st.session_state.preview_origin_lat)
    origin_lon = float(st.session_state.preview_origin_lon)
    orientation = float(st.session_state.get("preview_orientation_deg", 0.0)) % 360.0
    if st.session_state.get("grid_finalized") and "grid_orientation_deg" in st.session_state:
        orientation = float(st.session_state.grid_orientation_deg) % 360.0
    staggered = bool(st.session_state.get("grid_staggered", True))
    if st.session_state.get("grid_endless", True):
        df, center = generate_nearest_endless_grid_point(
            origin_lat,
            origin_lon,
            user_lat,
            user_lon,
            spacing,
            orientation,
            staggered,
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

_LIVE_FIELD_MAP_HTML = """<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <!-- Leaflet is served locally by Streamlit (from the placement map
         component's folder) instead of CDNs: faster load, works offline. -->
    <link
      rel="stylesheet"
      href="/component/placement_map_html.placement_map/vendor/leaflet.css"
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

      .map-hud {
        position: absolute;
        left: 10px;
        right: 10px;
        bottom: 10px;
        z-index: 1000;
        display: flex;
        flex-direction: column;
        gap: 8px;
        pointer-events: none;
      }

      .nav-readout {
        background: rgba(255, 255, 255, 0.94);
        border: 1px solid rgba(0, 0, 0, 0.15);
        border-radius: 8px;
        padding: 10px 12px;
        font: 14px/1.35 sans-serif;
        color: #222;
        box-shadow: 0 1px 4px rgba(0, 0, 0, 0.12);
      }

      .nav-readout.reached {
        background: rgba(232, 245, 233, 0.96);
        border-color: rgba(46, 125, 50, 0.35);
        color: #1b5e20;
        font-weight: 600;
      }

      .nav-readout.warn {
        background: rgba(255, 243, 224, 0.96);
        border-color: rgba(239, 108, 0, 0.35);
      }

      .point-toast {
        position: absolute;
        top: 56px;
        left: 50%;
        transform: translateX(-50%);
        z-index: 1002;
        max-width: calc(100% - 24px);
        padding: 8px 12px;
        border-radius: 8px;
        background: rgba(255, 255, 255, 0.96);
        border: 1px solid rgba(0, 0, 0, 0.15);
        box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
        font: 13px/1.35 sans-serif;
        color: #222;
        text-align: center;
        pointer-events: none;
        opacity: 0;
        transition: opacity 0.2s ease;
      }

      .point-toast.visible {
        opacity: 1;
      }

      /* DivIcon dots live on leaflet-rotate's norotate marker pane so they
         stay locked to lat/lng during zoom/rotate (SVG circleMarkers drift). */
      .gps-stick-dot-icon,
      .gps-stick-cone-icon {
        background: transparent !important;
        border: none !important;
      }

      .gps-stick-dot {
        box-sizing: border-box;
        border-radius: 50%;
        border: 2px solid;
      }

      .gps-stick-cone-wrap {
        width: 40px;
        height: 48px;
        /* Pivot on the cone apex (bottom-center), which sits on the user dot. */
        transform-origin: 20px 46px;
        /* Short ease so turns feel immediate without snapping to sensor noise. */
        transition: transform 0.12s ease-out;
      }

      .follow-toggle {
        position: absolute;
        top: 10px;
        right: 10px;
        z-index: 1000;
        border: 1px solid rgba(0, 0, 0, 0.15);
        border-radius: 6px;
        padding: 8px 12px;
        font: 13px/1.2 sans-serif;
        font-weight: 600;
        color: #333;
        background: rgba(255, 255, 255, 0.95);
        box-shadow: 0 1px 4px rgba(0, 0, 0, 0.2);
        cursor: pointer;
      }

      .follow-toggle.active {
        color: #fff;
        background: #1565c0;
        border-color: #0d47a1;
      }
    </style>
  </head>
  <body>
    <div class="map-shell">
      <div id="map-config" data-config-b64="__MAP_CONFIG_B64__" hidden></div>
      <div id="map"></div>
      <button id="follow-toggle" class="follow-toggle" type="button" title="Toggle whether the map rotates with your heading">
        Map: free rotate
      </button>
      <div id="point-toast" class="point-toast" aria-live="polite"></div>
      <div class="map-hud">
        <div id="nav-readout" class="nav-readout">Waiting for RTK position from the receiver...</div>
      </div>
    </div>

    <script src="/component/placement_map_html.placement_map/vendor/leaflet.js"></script>
    <script src="/component/placement_map_html.placement_map/vendor/leaflet-rotate.js"></script>
    <script>
      const MAP_MESSAGE_TYPE = __MAP_MESSAGE_TYPE__;
      const MAP_CONFIG = JSON.parse(
        atob(document.getElementById("map-config").dataset.configB64)
      );

      const NAV_MESSAGE_TYPE = "gps-stick-nav";
      const NAV_CONFIG_MESSAGE_TYPE = "gps-stick-nav-config";
      const HEADING_MESSAGE_TYPE = "gps-stick-heading";
      if (MAP_CONFIG.reach_tolerance_m == null) {
        MAP_CONFIG.reach_tolerance_m = 0.015;
      }
      let sensorIssue = null; // "insecure" | "desktop" | "no-compass"
      let orientationEventSeen = false;
      // When true, the map bearing tracks our smoothed compass heading.
      let followHeading = false;
      let map = null;

      function isLikelyDesktop() {
        const ua = navigator.userAgent || "";
        const mobile = /Android|iPhone|iPad|iPod|Mobile|Tablet/i.test(ua);
        const hasTouch = (navigator.maxTouchPoints || 0) > 0;
        return !mobile && !hasTouch;
      }

      function showSensorWarning(issue, text) {
        sensorIssue = issue;
        publishNavUpdate({
          distance: null,
          relative_bearing: null,
          direction: null,
          reached: false,
          sensorIssue: issue,
        });
      }

      function applyExternalHeading(heading) {
        const next = Number(heading);
        if (!Number.isFinite(next)) {
          return;
        }
        orientationEventSeen = true;
        if (sensorIssue === "no-compass") {
          sensorIssue = null;
        }
        MAP_CONFIG.user_heading = ((next % 360) + 360) % 360;
        drawUserMarker();
      }

      function getDistanceAndBearing(lat1, lon1, lat2, lon2) {
        const earthRadius = 6371000;
        const phi1 = (lat1 * Math.PI) / 180;
        const phi2 = (lat2 * Math.PI) / 180;
        const deltaPhi = ((lat2 - lat1) * Math.PI) / 180;
        const deltaLambda = ((lon2 - lon1) * Math.PI) / 180;
        const a =
          Math.sin(deltaPhi / 2) ** 2 +
          Math.cos(phi1) * Math.cos(phi2) * Math.sin(deltaLambda / 2) ** 2;
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        const distance = earthRadius * c;
        const y = Math.sin(deltaLambda) * Math.cos(phi2);
        const x =
          Math.cos(phi1) * Math.sin(phi2) -
          Math.sin(phi1) * Math.cos(phi2) * Math.cos(deltaLambda);
        const bearing = (Math.atan2(y, x) * 180) / Math.PI;
        return { distance, bearing: (bearing + 360) % 360 };
      }

      function getRelativeDirection(userHeading, bearingToTarget) {
        const relative = (bearingToTarget - userHeading + 360) % 360;
        if (relative >= 315 || relative < 45) {
          return "forward";
        }
        if (relative < 135) {
          return "right";
        }
        if (relative < 225) {
          return "back";
        }
        return "left";
      }

      function directionLabel(direction) {
        return {
          forward: "Head forward",
          back: "Head back",
          left: "Head left",
          right: "Head right",
        }[direction];
      }

      function publishNavUpdate(payload) {
        try {
          if (typeof BroadcastChannel !== "undefined") {
            const navChannel = new BroadcastChannel(NAV_MESSAGE_TYPE);
            navChannel.postMessage(payload);
            navChannel.close();
          }
        } catch (error) {
          // Ignore channel errors on older browsers.
        }
      }

      function refreshNearestTarget(userLat, userLon) {
        let nearest = null;
        let nearestDistance = Infinity;
        MAP_CONFIG.grid_points.forEach((point) => {
          if (skippedPoints.has(point.point)) {
            return;
          }
          const { distance } = getDistanceAndBearing(
            userLat,
            userLon,
            point.lat,
            point.lon
          );
          if (distance < nearestDistance) {
            nearestDistance = distance;
            nearest = point;
          }
        });

        const nextTarget = nearest ? nearest.point : null;
        const nextLat = nearest ? nearest.lat : null;
        const nextLon = nearest ? nearest.lon : null;
        if (
          nextTarget !== MAP_CONFIG.target_point ||
          nextLat !== MAP_CONFIG.target_lat ||
          nextLon !== MAP_CONFIG.target_lon
        ) {
          MAP_CONFIG.target_point = nextTarget;
          MAP_CONFIG.target_lat = nextLat;
          MAP_CONFIG.target_lon = nextLon;
          drawGridMarkers();
        }
      }

      function updateNavigationHud() {
        const readout = document.getElementById("nav-readout");
        if (!readout) {
          return;
        }

        const userLat = Number(MAP_CONFIG.user_lat);
        const userLon = Number(MAP_CONFIG.user_lon);
        const toleranceM = Number(MAP_CONFIG.reach_tolerance_m);
        const reachTol =
          Number.isFinite(toleranceM) && toleranceM > 0 ? toleranceM : 0.015;

        if (!Number.isFinite(userLat) || !Number.isFinite(userLon)) {
          readout.textContent = "Waiting for RTK position from the receiver...";
          readout.className = "nav-readout";
          publishNavUpdate({
            distance: null,
            relative_bearing: null,
            direction: null,
            reached: false,
            tolerance_m: reachTol,
          });
          return;
        }

        refreshNearestTarget(userLat, userLon);

        const targetLat = Number(MAP_CONFIG.target_lat);
        const targetLon = Number(MAP_CONFIG.target_lon);
        const targetName = MAP_CONFIG.target_point;

        if (!Number.isFinite(targetLat) || !Number.isFinite(targetLon)) {
          const anyActive = MAP_CONFIG.grid_points.some(
            (point) => !skippedPoints.has(point.point)
          );
          readout.textContent = anyActive
            ? "No grid points available."
            : "All grid points skipped — tap a yellow point to include it again.";
          readout.className = "nav-readout";
          publishNavUpdate({
            distance: null,
            relative_bearing: null,
            direction: null,
            reached: false,
            target: null,
            accuracy: MAP_CONFIG.user_accuracy,
            tolerance_m: reachTol,
          });
          return;
        }

        const { distance, bearing } = getDistanceAndBearing(
          userLat,
          userLon,
          targetLat,
          targetLon
        );

        if (distance < reachTol) {
          const tolCm = (reachTol * 100).toFixed(reachTol * 100 < 10 ? 1 : 0);
          readout.textContent = `${targetName} REACHED (within ${tolCm} cm)!`;
          readout.className = "nav-readout reached";
          publishNavUpdate({
            distance: 0,
            relative_bearing: 0,
            direction: "forward",
            reached: true,
            target: targetName,
            accuracy: MAP_CONFIG.user_accuracy,
            tolerance_m: reachTol,
          });
          return;
        }

        const accuracy = Number(MAP_CONFIG.user_accuracy);
        let accuracyNote = "";
        if (Number.isFinite(accuracy) && accuracy > 5) {
          accuracyNote = " · GPS accuracy is low";
          readout.className = "nav-readout warn";
        } else {
          readout.className = "nav-readout";
        }

        const heading = Number(MAP_CONFIG.user_heading);
        if (!Number.isFinite(heading)) {
          const headingHint = {
            insecure: "open the app over HTTPS for compass directions",
            desktop: "compass not available on desktop",
            "no-compass": "compass not available on this device",
          }[sensorIssue] || "tap Enable compass for directions";
          readout.textContent =
            `${distance.toFixed(2)} m to ${targetName} · ${headingHint}` +
            accuracyNote;
          publishNavUpdate({
            distance,
            relative_bearing: null,
            direction: null,
            reached: false,
            target: targetName,
            accuracy,
            sensorIssue,
            tolerance_m: reachTol,
          });
          return;
        }

        const direction = getRelativeDirection(heading, bearing);
        const relativeBearing = (bearing - heading + 360) % 360;
        readout.textContent =
          `${directionLabel(direction)} · ${distance.toFixed(2)} m to ${targetName}` +
          accuracyNote;
        publishNavUpdate({
          distance,
          relative_bearing: relativeBearing,
          direction,
          reached: false,
          target: targetName,
          accuracy,
          tolerance_m: reachTol,
        });
      }

      function makeDotIcon(color, radius, fillOpacity) {
        const diameter = radius * 2;
        const size = diameter + 4;
        return L.divIcon({
          className: "gps-stick-dot-icon",
          iconSize: [size, size],
          iconAnchor: [size / 2, size / 2],
          html:
            `<div class="gps-stick-dot" style="` +
            `width:${diameter}px;height:${diameter}px;` +
            `margin:2px;background:${color};border-color:${color};` +
            `opacity:${fillOpacity};"></div>`,
        });
      }

      function screenUpBearing() {
        if (!map || typeof map.getBearing !== "function") {
          return 0;
        }
        const bearing = Number(map.getBearing());
        if (!Number.isFinite(bearing)) {
          return 0;
        }
        return (360 - (((bearing % 360) + 360) % 360)) % 360;
      }

      function makeConeIcon(screenRotDeg) {
        // Apex at bottom-center sits on the user dot; the cone opens upward
        // (screen +Y at rotation 0) in the facing direction.
        return L.divIcon({
          className: "gps-stick-cone-icon",
          iconSize: [40, 48],
          iconAnchor: [20, 46],
          html:
            `<div class="gps-stick-cone-wrap" style="transform:rotate(${screenRotDeg}deg)">` +
            `<svg width="40" height="48" viewBox="0 0 40 48" aria-hidden="true">` +
            `<polygon points="20,46 36,2 4,2" fill="rgba(255,68,68,0.35)" ` +
            `stroke="#FF4444" stroke-width="1"></polygon></svg></div>`,
        });
      }

      const gridMarkersByPoint = new Map();
      // Yellow / skipped points are ignored when choosing the nearest target.
      const skippedPoints = new Set();
      let userDotMarker = null;
      let facingConeMarker = null;
      let zoomGestureActive = false;
      let userDrawDeferred = false;
      let pointToastTimer = null;

      function mapBusyWithZoom() {
        return zoomGestureActive || !!(map && map._animatingZoom);
      }

      function showPointToast(text) {
        const el = document.getElementById("point-toast");
        if (!el) {
          return;
        }
        el.textContent = text;
        el.classList.add("visible");
        clearTimeout(pointToastTimer);
        pointToastTimer = setTimeout(() => {
          el.classList.remove("visible");
        }, 1000);
      }

      function toggleSkippedPoint(pointName) {
        if (skippedPoints.has(pointName)) {
          skippedPoints.delete(pointName);
          showPointToast(`${pointName} included again`);
        } else {
          skippedPoints.add(pointName);
          showPointToast(`${pointName} excluded`);
          if (MAP_CONFIG.target_point === pointName) {
            MAP_CONFIG.target_point = null;
            MAP_CONFIG.target_lat = null;
            MAP_CONFIG.target_lon = null;
          }
        }
        drawGridMarkers();
        updateNavigationHud();
      }

      function drawGridMarkers() {
        const keep = new Set(
          MAP_CONFIG.grid_points.map((point) => point.point)
        );
        gridMarkersByPoint.forEach((marker, pointName) => {
          if (!keep.has(pointName)) {
            map.removeLayer(marker);
            gridMarkersByPoint.delete(pointName);
          }
        });
        if (
          MAP_CONFIG.target_point != null &&
          !keep.has(MAP_CONFIG.target_point)
        ) {
          MAP_CONFIG.target_point = null;
          MAP_CONFIG.target_lat = null;
          MAP_CONFIG.target_lon = null;
        }
        MAP_CONFIG.grid_points.forEach((point) => {
          const skipped = skippedPoints.has(point.point);
          const isTarget =
            !skipped && point.point === MAP_CONFIG.target_point;
          let color;
          let radius;
          let fillOpacity;
          let zIndex;
          if (skipped) {
            color = "#FFD600";
            radius = 7;
            fillOpacity = 0.9;
            zIndex = 300;
          } else if (isTarget) {
            color = "#00FF00";
            radius = 9;
            fillOpacity = 0.9;
            zIndex = 400;
          } else {
            color = "#1E90FF";
            radius = 6;
            fillOpacity = 0.45;
            zIndex = 200;
          }
          const icon = makeDotIcon(color, radius, fillOpacity);
          let marker = gridMarkersByPoint.get(point.point);
          if (!marker) {
            marker = L.marker([point.lat, point.lon], {
              icon,
              keyboard: false,
              zIndexOffset: zIndex,
            })
              .on("click", (event) => {
                L.DomEvent.stopPropagation(event);
                toggleSkippedPoint(point.point);
              })
              .addTo(map);
            gridMarkersByPoint.set(point.point, marker);
          } else {
            marker.setLatLng([point.lat, point.lon]);
            marker.setIcon(icon);
            marker.setZIndexOffset(zIndex);
          }
        });
      }

      // Cumulative CSS angle for the cone. Tracking the shortest-path delta
      // (instead of a raw 0-360 value) keeps the CSS transition from spinning
      // the long way around when the heading crosses north.
      let coneScreenRotCum = null;

      function updateFacingCone() {
        const heading = Number(MAP_CONFIG.user_heading);
        const lat = Number(MAP_CONFIG.user_lat);
        const lon = Number(MAP_CONFIG.user_lon);
        if (
          !Number.isFinite(heading) ||
          !Number.isFinite(lat) ||
          !Number.isFinite(lon)
        ) {
          if (facingConeMarker) {
            map.removeLayer(facingConeMarker);
            facingConeMarker = null;
            coneScreenRotCum = null;
          }
          return;
        }
        // When the map follows heading, screen-up == facing, so the cone
        // points straight up on screen. Otherwise CSS rotate (clockwise)
        // is heading relative to geographic screen-up.
        const target = followHeading
          ? 0
          : (heading - screenUpBearing() + 360) % 360;
        if (!facingConeMarker) {
          coneScreenRotCum = target;
          facingConeMarker = L.marker([lat, lon], {
            icon: makeConeIcon(target),
            interactive: false,
            keyboard: false,
            zIndexOffset: 550,
          }).addTo(map);
          return;
        }
        facingConeMarker.setLatLng([lat, lon]);
        const currentMod = ((coneScreenRotCum % 360) + 360) % 360;
        coneScreenRotCum += ((target - currentMod + 540) % 360) - 180;
        const el = facingConeMarker.getElement();
        const wrap = el && el.querySelector(".gps-stick-cone-wrap");
        if (wrap) {
          // Mutate the existing element so the CSS transition animates the
          // turn; setIcon would replace the DOM node and jump instead.
          wrap.style.transform = `rotate(${coneScreenRotCum}deg)`;
        } else {
          facingConeMarker.setIcon(makeConeIcon(target));
        }
      }

      function drawUserMarker() {
        if (mapBusyWithZoom()) {
          // setLatLng mid-zoom animation drifts markers; apply after zoomend.
          userDrawDeferred = true;
          return;
        }
        const lat = Number(MAP_CONFIG.user_lat);
        const lon = Number(MAP_CONFIG.user_lon);
        if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
          return;
        }
        const icon = makeDotIcon("#FF0000", 7, 1.0);
        if (!userDotMarker) {
          userDotMarker = L.marker([lat, lon], {
            icon,
            keyboard: false,
            zIndexOffset: 600,
          })
            .bindPopup("Your position")
            .addTo(map);
        } else {
          userDotMarker.setLatLng([lat, lon]);
          userDotMarker.setIcon(icon);
        }
        updateFacingCone();
        updateNavigationHud();
      }

      // Heading filter state. Android fires BOTH deviceorientationabsolute
      // (true north) and plain deviceorientation (relative, arbitrary zero);
      // mixing them makes the heading jump. Once an absolute source is seen,
      // relative readings are discarded.
      let hasAbsoluteHeading = false;
      let headingSin = null;
      let headingCos = null;
      let headingFilterReady = false;
      let headingRefreshTimer = null;
      let mapBearingTimer = null;
      let lastAppliedMapBearing = null;
      // Higher = snappier (closer to raw). Still an EMA on sin/cos so wrap and
      // magnetometer jitter stay filtered instead of driving the UI raw.
      const HEADING_SMOOTHING = 0.35;
      // Cone + nav arrow refresh (~10 Hz). Map follow stays on its own ~30 fps
      // loop. Latency came mostly from the old 0.5 Hz UI sample + 1.8s CSS ease.
      const HEADING_REFRESH_MS = 100;
      const MAP_BEARING_MS = 1000 / 30;
      // Ignore stillness wiggle, but start tracking real turns sooner than 3°.
      const HEADING_DEADBAND_DEG = 1.5;
      // Tiny map deadband so 30 fps still feels continuous but skips noise.
      const MAP_BEARING_DEADBAND_DEG = 0.35;

      function getSmoothedHeading() {
        if (!headingFilterReady) {
          return null;
        }
        return (
          ((Math.atan2(headingSin, headingCos) * 180) / Math.PI + 360) % 360
        );
      }

      function applyConeHeading() {
        const smoothed = getSmoothedHeading();
        if (smoothed == null) {
          return;
        }
        const previous = Number(MAP_CONFIG.user_heading);
        if (Number.isFinite(previous)) {
          const delta = Math.abs(((smoothed - previous + 540) % 360) - 180);
          if (delta < HEADING_DEADBAND_DEG) {
            return;
          }
        }
        MAP_CONFIG.user_heading = smoothed;
        // Map bearing is driven by the 30 fps loop when followHeading is on.
        drawUserMarker();
      }

      function applyMapBearing() {
        if (
          !followHeading ||
          !map ||
          typeof map.setBearing !== "function" ||
          mapBusyWithZoom()
        ) {
          return;
        }
        const smoothed = getSmoothedHeading();
        if (smoothed == null) {
          return;
        }
        if (lastAppliedMapBearing != null) {
          const delta = Math.abs(
            ((smoothed - lastAppliedMapBearing + 540) % 360) - 180
          );
          if (delta < MAP_BEARING_DEADBAND_DEG) {
            return;
          }
        }
        lastAppliedMapBearing = smoothed;
        // leaflet-rotate pane angle is opposite screen-up; put heading at top.
        map.setBearing((360 - smoothed) % 360);
      }

      function startHeadingRefreshTimer() {
        if (headingRefreshTimer != null) {
          return;
        }
        applyConeHeading();
        headingRefreshTimer = setInterval(applyConeHeading, HEADING_REFRESH_MS);
      }

      function stopHeadingRefreshTimer() {
        if (headingRefreshTimer != null) {
          clearInterval(headingRefreshTimer);
          headingRefreshTimer = null;
        }
      }

      function startMapBearingTimer() {
        if (mapBearingTimer != null) {
          return;
        }
        lastAppliedMapBearing = null;
        applyMapBearing();
        mapBearingTimer = setInterval(applyMapBearing, MAP_BEARING_MS);
      }

      function stopMapBearingTimer() {
        if (mapBearingTimer != null) {
          clearInterval(mapBearingTimer);
          mapBearingTimer = null;
        }
        lastAppliedMapBearing = null;
      }

      function updateFollowToggleUi() {
        const btn = document.getElementById("follow-toggle");
        if (!btn) {
          return;
        }
        btn.classList.toggle("active", followHeading);
        if (followHeading) {
          btn.textContent = "Map: follows heading";
          btn.title = "Map rotates so the direction you face is always up";
        } else {
          btn.textContent = "Map: free rotate";
          btn.title =
            "Map stays where you leave it; rotate with two fingers. Cone shows heading.";
        }
      }

      function setFollowHeading(enabled) {
        followHeading = !!enabled;
        if (map) {
          if (followHeading) {
            if (map.touchRotate && map.touchRotate.disable) {
              map.touchRotate.disable();
            }
            if (map.shiftKeyRotate && map.shiftKeyRotate.disable) {
              map.shiftKeyRotate.disable();
            }
            startMapBearingTimer();
          } else {
            stopMapBearingTimer();
            if (map.touchRotate && map.touchRotate.enable) {
              map.touchRotate.enable();
            }
            if (map.shiftKeyRotate && map.shiftKeyRotate.enable) {
              map.shiftKeyRotate.enable();
            }
          }
          if (map.compassBearing) {
            map.compassBearing._enabled = followHeading;
            map.fire("rotate");
          }
        }
        updateFollowToggleUi();
        updateFacingCone();
      }

      function extractHeading(event) {
        // iOS Safari: true compass heading, already in degrees from north.
        if (
          event.webkitCompassHeading != null &&
          Number.isFinite(event.webkitCompassHeading)
        ) {
          return { heading: event.webkitCompassHeading, absolute: true };
        }
        if (event.alpha != null && Number.isFinite(event.alpha)) {
          const absolute =
            event.absolute === true || event.type === "deviceorientationabsolute";
          return { heading: (360 - event.alpha) % 360, absolute };
        }
        return null;
      }

      function onDeviceOrientation(event) {
        orientationEventSeen = true;
        if (sensorIssue === "no-compass") {
          sensorIssue = null;
        }

        const reading = extractHeading(event);
        if (reading == null) {
          return;
        }

        if (reading.absolute) {
          hasAbsoluteHeading = true;
        } else if (hasAbsoluteHeading) {
          // A north-referenced source exists; ignore relative readings.
          return;
        }

        // Low-pass filter on the heading's unit vector. Filtering sin/cos
        // instead of degrees handles the 359°->0° wraparound correctly and
        // absorbs magnetometer jitter.
        const rad = (reading.heading * Math.PI) / 180;
        if (headingSin == null) {
          headingSin = Math.sin(rad);
          headingCos = Math.cos(rad);
        } else {
          headingSin += HEADING_SMOOTHING * (Math.sin(rad) - headingSin);
          headingCos += HEADING_SMOOTHING * (Math.cos(rad) - headingCos);
        }
        headingFilterReady = true;
      }

      async function requestOrientationPermission() {
        if (
          typeof DeviceOrientationEvent !== "undefined" &&
          typeof DeviceOrientationEvent.requestPermission === "function"
        ) {
          const response = await DeviceOrientationEvent.requestPermission();
          return response === "granted";
        }
        return true;
      }

      function startCompass() {
        // Android Chrome delivers true-north headings via the "absolute"
        // event; iOS Safari only fires plain deviceorientation (with
        // webkitCompassHeading). Listen to both and let whichever fires win.
        if ("ondeviceorientationabsolute" in window) {
          window.addEventListener(
            "deviceorientationabsolute",
            onDeviceOrientation,
            true
          );
        }
        window.addEventListener("deviceorientation", onDeviceOrientation, true);
        startHeadingRefreshTimer();

        setTimeout(() => {
          if (!orientationEventSeen && sensorIssue == null) {
            showSensorWarning(
              "no-compass",
              "No compass data is coming from this device. Distance still works, but facing direction will be unavailable."
            );
          }
        }, 4000);
      }

      async function enableClientSensors() {
        try {
          const orientationGranted = await requestOrientationPermission();
          if (!orientationGranted) {
            const readout = document.getElementById("nav-readout");
            if (readout) {
              readout.textContent =
                "Compass permission denied. You can still see distance, but not facing direction.";
              readout.className = "nav-readout warn";
            }
          }
        } catch (error) {
          // Still attach listeners; some browsers fire events without the prompt.
        }
        startCompass();
      }

      function handleMapMessage(data) {
        if (!data || data.type !== MAP_MESSAGE_TYPE || !map) {
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
          return;
        }

        if (data.action === "updateGridPoints") {
          const points = Array.isArray(data.grid_points) ? data.grid_points : [];
          MAP_CONFIG.grid_points = points
            .map((point) => ({
              point: String(point.point),
              lat: Number(point.lat),
              lon: Number(point.lon),
            }))
            .filter(
              (point) =>
                point.point &&
                Number.isFinite(point.lat) &&
                Number.isFinite(point.lon)
            );
          drawGridMarkers();
          updateNavigationHud();
        }
      }

      map = L.map("map", {
        center: [MAP_CONFIG.center_lat, MAP_CONFIG.center_lon],
        zoom: MAP_CONFIG.zoom,
        zoomControl: true,
        maxZoom: 25,
        rotate: true,
        bearing: 0,
        touchRotate: true,
        shiftKeyRotate: true,
        rotateControl: {
          closeOnZeroBearing: false,
          position: "topleft",
        },
      });

      // Drive "compass follow" from our own heading filter (handles iOS/Android
      // quirks). Hijack the plugin handler so the rotate control's orange
      // compass mode toggles followHeading instead of attaching a second listener.
      if (map.compassBearing) {
        map.compassBearing.enable = function () {
          setFollowHeading(true);
          return this;
        };
        map.compassBearing.disable = function () {
          setFollowHeading(false);
          return this;
        };
        map.compassBearing.enabled = function () {
          return !!followHeading;
        };
      }

      L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        attribution: "&copy; OpenStreetMap contributors",
        maxZoom: 25,
        maxNativeZoom: 19,
      }).addTo(map);

      L.control
        .scale({ position: "bottomright", metric: true, imperial: false, maxWidth: 120 })
        .addTo(map);

      // Markers (DivIcon) sit on leaflet-rotate's norotate pane and stay locked
      // to lat/lng during zoom/rotate. SVG circleMarkers on overlayPane drift.
      drawGridMarkers();
      drawUserMarker();

      map.on("zoomstart", () => {
        zoomGestureActive = true;
      });
      map.on("zoomend", () => {
        zoomGestureActive = false;
        gridMarkersByPoint.forEach((marker) => marker.update());
        if (userDotMarker) {
          userDotMarker.update();
        }
        if (facingConeMarker) {
          facingConeMarker.update();
        }
        if (userDrawDeferred) {
          userDrawDeferred = false;
          drawUserMarker();
        } else {
          updateFacingCone();
        }
      });
      map.on("rotate", () => {
        // In follow mode the cone stays screen-up (target 0) and the map
        // turns underneath at 30 fps — skip cone work on every rotate tick.
        if (!mapBusyWithZoom() && !followHeading) {
          updateFacingCone();
        }
      });

      // Browsers remove geolocation and orientation APIs entirely on plain
      // HTTP (except localhost), so check the secure context before blaming
      // the device type.
      if (!window.isSecureContext) {
        showSensorWarning(
          "insecure",
          "This page is served over HTTP, so the browser blocks compass access. Position from the RTK receiver still works, but open the app via HTTPS for facing direction."
        );
      } else if (isLikelyDesktop()) {
        showSensorWarning(
          "desktop",
          "Desktop browsers usually do not provide compass orientation. Use a phone or tablet to see which way you are facing and get forward/back/left/right directions."
        );
      }

      document
        .getElementById("follow-toggle")
        .addEventListener("click", () => {
          setFollowHeading(!followHeading);
        });
      updateFollowToggleUi();

      try {
        if (typeof BroadcastChannel !== "undefined") {
          const gridChannel = new BroadcastChannel(MAP_MESSAGE_TYPE);
          gridChannel.onmessage = (event) => {
            handleMapMessage(event.data);
          };
          const navConfigChannel = new BroadcastChannel(NAV_CONFIG_MESSAGE_TYPE);
          navConfigChannel.onmessage = (event) => {
            const nextTol = Number(event.data && event.data.tolerance_m);
            if (Number.isFinite(nextTol) && nextTol > 0) {
              MAP_CONFIG.reach_tolerance_m = nextTol;
              updateNavigationHud();
            }
          };
          // Compass is enabled from the Live Navigation panel (not a map
          // overlay). Heading can arrive from that panel, or this iframe
          // can start its own sensors when told to enable.
          const headingChannel = new BroadcastChannel(HEADING_MESSAGE_TYPE);
          headingChannel.onmessage = (event) => {
            const data = event.data || {};
            if (data.action === "enableCompass") {
              enableClientSensors();
              return;
            }
            if (data.heading != null) {
              applyExternalHeading(data.heading);
            }
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


_LIVE_NAV_PANEL_HTML = """<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <style>
      html,
      body {
        margin: 0;
        padding: 0;
        height: 100%;
        font-family: sans-serif;
        background: #f4f6f8;
        color: #111;
        color-scheme: light;
      }

      .panel {
        box-sizing: border-box;
        min-height: 100%;
        padding: 16px 16px 12px;
        display: flex;
        flex-direction: column;
        align-items: center;
        gap: 10px;
        background: #f4f6f8;
        color: #111;
      }

      .arrow-wrap {
        flex: 1 1 auto;
        width: 100%;
        min-height: 220px;
        display: flex;
        align-items: center;
        justify-content: center;
        position: relative;
      }

      .arrow {
        position: relative;
        width: min(58vw, 220px);
        height: min(72vw, 270px);
        display: flex;
        align-items: center;
        justify-content: center;
        transform-origin: 50% 55%;
        transition: transform 0.12s linear;
      }

      .arrow-svg {
        position: relative;
        width: 78%;
        height: 88%;
        overflow: visible;
        filter: drop-shadow(0 10px 18px rgba(13, 71, 161, 0.35));
        transition: filter 0.2s ease, opacity 0.2s ease;
      }

      .arrow-svg .arrow-body {
        fill: url(#arrowFill);
        stroke: #0d47a1;
        stroke-width: 5;
        stroke-linejoin: round;
        stroke-linecap: round;
      }

      .arrow-svg .arrow-sheen {
        fill: url(#arrowSheen);
        opacity: 0.55;
        pointer-events: none;
      }

      .arrow.disabled .arrow-svg {
        filter: none;
        opacity: 0.55;
      }

      .arrow.disabled .arrow-svg .arrow-body {
        fill: #9e9e9e;
        stroke: #757575;
      }

      .arrow.disabled .arrow-svg .arrow-sheen {
        display: none;
      }

      .arrow.reached {
        display: none;
      }

      .reached-mark {
        display: none;
        width: min(44vw, 140px);
        height: min(44vw, 140px);
        border-radius: 50%;
        background: radial-gradient(circle at 40% 35%, #66bb6a, #1b5e20);
        color: #fff;
        font-size: min(22vw, 72px);
        font-weight: 700;
        line-height: 1;
        align-items: center;
        justify-content: center;
        box-shadow: 0 12px 28px rgba(27, 94, 32, 0.35);
      }

      .reached-mark.visible {
        display: flex;
      }

      .distance-block {
        text-align: center;
        width: 100%;
      }

      .distance-value {
        font-size: 42px;
        font-weight: 700;
        letter-spacing: -0.02em;
        color: #111;
        line-height: 1.1;
      }

      .status {
        margin-top: 4px;
        font-size: 15px;
        color: #555;
        text-align: center;
        min-height: 1.2em;
      }

      .status.success {
        color: #1b5e20;
        font-weight: 600;
      }

      .status.warn {
        color: #e65100;
      }

      .tolerance-row {
        display: flex;
        align-items: center;
        justify-content: center;
        gap: 8px;
        font-size: 12px;
        color: #777;
        margin-top: 2px;
      }

      .tolerance-row input {
        width: 64px;
        padding: 4px 6px;
        border: 1px solid rgba(0, 0, 0, 0.12);
        border-radius: 6px;
        font-size: 13px;
        color: #444;
        background: #fff;
      }

      .compass-row {
        width: 100%;
        display: flex;
        justify-content: center;
        align-items: center;
        gap: 10px;
      }

      .compass-btn {
        border: none;
        border-radius: 10px;
        padding: 12px 18px;
        font: 15px/1.2 sans-serif;
        font-weight: 700;
        color: #fff;
        background: #1565c0;
        cursor: pointer;
        box-shadow: 0 2px 8px rgba(21, 101, 192, 0.28);
      }

      .compass-btn:hover {
        background: #0d47a1;
      }

      .compass-btn.enabled {
        background: #2e7d32;
        box-shadow: none;
        cursor: default;
      }

      .compass-btn:disabled {
        opacity: 0.7;
        cursor: default;
      }

      .desktop-warn {
        display: none;
        width: 100%;
        box-sizing: border-box;
        text-align: center;
        border-radius: 10px;
        padding: 10px 12px;
        font: 600 14px/1.35 sans-serif;
        color: #e65100;
        background: #fff3e0;
        border: 1px solid rgba(230, 81, 0, 0.28);
      }

      .desktop-warn.visible {
        display: block;
      }

      .fullscreen-btn {
        border: 1px solid rgba(0, 0, 0, 0.15);
        border-radius: 10px;
        padding: 12px 14px;
        font: 14px/1.2 sans-serif;
        font-weight: 600;
        color: #1565c0;
        background: #fff;
        cursor: pointer;
      }

      .fullscreen-btn:hover {
        background: #e3f2fd;
      }

      .rtk-bar {
        width: 100%;
        box-sizing: border-box;
        text-align: center;
        border-radius: 10px;
        padding: 10px 12px;
        font: 600 15px/1.3 sans-serif;
        letter-spacing: 0.01em;
        border: 1px solid transparent;
      }

      .rtk-bar.unknown {
        color: #555;
        background: #eceff1;
        border-color: rgba(0, 0, 0, 0.08);
      }

      .rtk-bar.none {
        color: #b71c1c;
        background: #ffebee;
        border-color: rgba(183, 28, 28, 0.25);
      }

      .rtk-bar.float {
        color: #e65100;
        background: #fff3e0;
        border-color: rgba(230, 81, 0, 0.28);
      }

      .rtk-bar.fix {
        color: #1b5e20;
        background: #e8f5e9;
        border-color: rgba(27, 94, 32, 0.28);
      }

      .panel.is-fullscreen,
      .panel:fullscreen,
      .panel:-webkit-full-screen {
        position: fixed;
        inset: 0;
        width: 100%;
        height: 100%;
        min-height: 100%;
        border-radius: 0;
        z-index: 9999;
        justify-content: center;
        /* Force light surface: some phones paint a black fullscreen
           backdrop while our distance text stays dark (#111). */
        background: #f4f6f8 !important;
        color: #111 !important;
        color-scheme: light;
      }

      .panel.is-fullscreen .rtk-bar,
      .panel:fullscreen .rtk-bar,
      .panel:-webkit-full-screen .rtk-bar {
        font-size: 18px;
        padding: 12px 14px;
      }

      .panel.is-fullscreen .arrow-wrap,
      .panel:fullscreen .arrow-wrap,
      .panel:-webkit-full-screen .arrow-wrap {
        flex: 1 1 auto;
        min-height: 0;
      }

      .panel.is-fullscreen .arrow,
      .panel:fullscreen .arrow,
      .panel:-webkit-full-screen .arrow {
        width: min(90vw, 560px);
        height: min(110vw, 680px);
      }

      .panel.is-fullscreen .reached-mark,
      .panel:fullscreen .reached-mark,
      .panel:-webkit-full-screen .reached-mark {
        width: min(56vw, 220px);
        height: min(56vw, 220px);
        font-size: min(28vw, 96px);
      }

      .panel.is-fullscreen .distance-value,
      .panel:fullscreen .distance-value,
      .panel:-webkit-full-screen .distance-value {
        color: #111 !important;
        -webkit-text-fill-color: #111;
        font-size: 64px;
      }

      .panel.is-fullscreen .distance-block,
      .panel:fullscreen .distance-block,
      .panel:-webkit-full-screen .distance-block {
        color: #111;
      }

      .panel.is-fullscreen .status,
      .panel:fullscreen .status,
      .panel:-webkit-full-screen .status {
        color: #444 !important;
      }

      .panel.is-fullscreen .tolerance-row,
      .panel:fullscreen .tolerance-row,
      .panel:-webkit-full-screen .tolerance-row {
        color: #555 !important;
      }

      .panel.is-fullscreen .tolerance-row input,
      .panel:fullscreen .tolerance-row input,
      .panel:-webkit-full-screen .tolerance-row input {
        color: #222 !important;
        background: #fff !important;
      }
    </style>
  </head>
  <body>
    <div class="panel" id="nav-panel">
      <div class="compass-row">
        <button id="enable-compass" class="compass-btn" type="button">
          Enable compass
        </button>
        <button
          id="fullscreen-btn"
          class="fullscreen-btn"
          type="button"
          title="Expand the navigation arrow"
        >
          Fullscreen
        </button>
      </div>
      <div id="desktop-warn" class="desktop-warn" role="status" aria-live="polite">
        Open this page on a phone or tablet to get compass heading. Distance still works on desktop.
      </div>
      <div id="rtk-bar" class="rtk-bar unknown" role="status" aria-live="polite">
        Waiting for GPS…
      </div>
      <div class="arrow-wrap" title="Direction to target relative to the way you are facing">
        <div id="nav-arrow" class="arrow disabled">
          <svg
            class="arrow-svg"
            viewBox="0 0 200 280"
            xmlns="http://www.w3.org/2000/svg"
            aria-hidden="true"
          >
            <defs>
              <linearGradient id="arrowFill" x1="100" y1="16" x2="100" y2="268" gradientUnits="userSpaceOnUse">
                <stop offset="0%" stop-color="#42a5f5" />
                <stop offset="45%" stop-color="#1e88e5" />
                <stop offset="100%" stop-color="#0d47a1" />
              </linearGradient>
              <linearGradient id="arrowSheen" x1="70" y1="40" x2="130" y2="220" gradientUnits="userSpaceOnUse">
                <stop offset="0%" stop-color="#ffffff" stop-opacity="0.55" />
                <stop offset="55%" stop-color="#ffffff" stop-opacity="0.08" />
                <stop offset="100%" stop-color="#ffffff" stop-opacity="0" />
              </linearGradient>
            </defs>
            <!-- Classic navigation arrow: tip, wings, tapered shaft -->
            <path
              class="arrow-body"
              d="M100 18
                 L178 118
                 L142 118
                 L142 250
                 Q142 266 126 266
                 L74 266
                 Q58 266 58 250
                 L58 118
                 L22 118
                 Z"
            />
            <path
              class="arrow-sheen"
              d="M100 34
                 L150 108
                 L128 108
                 L128 240
                 L100 240
                 Z"
            />
          </svg>
        </div>
        <div id="reached-mark" class="reached-mark">✓</div>
      </div>
      <div class="distance-block">
        <div id="distance-value" class="distance-value">--</div>
        <div id="status-line" class="status">Tap Enable compass above, then follow the arrow. Position comes from the RTK receiver.</div>
      </div>
      <div class="tolerance-row">
        <label for="tolerance-cm">Reach tolerance</label>
        <input
          id="tolerance-cm"
          type="number"
          min="0.5"
          step="0.5"
          value="1.5"
        />
        <span>cm</span>
      </div>
    </div>
    <script>
      const NAV_MESSAGE_TYPE = "gps-stick-nav";
      const NAV_CONFIG_MESSAGE_TYPE = "gps-stick-nav-config";
      const HEADING_MESSAGE_TYPE = "gps-stick-heading";

      // Cumulative CSS angle so 359°→0° takes the short path instead of
      // spinning ~359° the long way around (0 and 360 are the same heading).
      let arrowRotCum = null;
      let compassEnabled = false;
      let hasAbsoluteHeading = false;
      let headingSin = null;
      let headingCos = null;
      let headingFilterReady = false;
      let headingPublishTimer = null;
      let orientationEventSeen = false;
      // Sticky across BroadcastChannel updates so desktop/insecure warnings
      // are not wiped by distance-only payloads from the field map.
      let panelSensorIssue = null;
      const HEADING_SMOOTHING = 0.35;
      const HEADING_PUBLISH_MS = 100;

      function setArrowRotation(degrees) {
        const arrow = document.getElementById("nav-arrow");
        if (!arrow) {
          return;
        }
        if (!Number.isFinite(degrees)) {
          arrowRotCum = null;
          arrow.style.transform = "rotate(0deg)";
          return;
        }
        const target = ((Number(degrees) % 360) + 360) % 360;
        if (arrowRotCum == null) {
          arrowRotCum = target;
        } else {
          const currentMod = ((arrowRotCum % 360) + 360) % 360;
          arrowRotCum += ((target - currentMod + 540) % 360) - 180;
        }
        arrow.style.transform = `rotate(${arrowRotCum}deg)`;
      }

      function formatDistance(meters) {
        if (meters == null || !Number.isFinite(Number(meters))) {
          return "--";
        }
        return `${Number(meters).toFixed(2)} m`;
      }

      function publishTolerance() {
        const input = document.getElementById("tolerance-cm");
        const cm = Number(input && input.value);
        if (!Number.isFinite(cm) || cm <= 0) {
          return;
        }
        try {
          if (typeof BroadcastChannel !== "undefined") {
            const channel = new BroadcastChannel(NAV_CONFIG_MESSAGE_TYPE);
            channel.postMessage({ tolerance_m: cm / 100 });
            channel.close();
          }
        } catch (error) {
          // Ignore channel errors on older browsers.
        }
      }

      function publishHeading(heading) {
        try {
          if (typeof BroadcastChannel !== "undefined") {
            const channel = new BroadcastChannel(HEADING_MESSAGE_TYPE);
            channel.postMessage({ heading: Number(heading) });
            channel.close();
          }
        } catch (error) {
          // Ignore channel errors on older browsers.
        }
      }

      function requestMapEnableCompass() {
        try {
          if (typeof BroadcastChannel !== "undefined") {
            const channel = new BroadcastChannel(HEADING_MESSAGE_TYPE);
            channel.postMessage({ action: "enableCompass" });
            channel.close();
          }
        } catch (error) {
          // Ignore channel errors on older browsers.
        }
      }

      function extractHeading(event) {
        if (
          event.webkitCompassHeading != null &&
          Number.isFinite(event.webkitCompassHeading)
        ) {
          return { heading: event.webkitCompassHeading, absolute: true };
        }
        if (event.alpha != null && Number.isFinite(event.alpha)) {
          const absolute =
            event.absolute === true || event.type === "deviceorientationabsolute";
          return { heading: (360 - event.alpha) % 360, absolute };
        }
        return null;
      }

      function getSmoothedHeading() {
        if (!headingFilterReady) {
          return null;
        }
        return (
          ((Math.atan2(headingSin, headingCos) * 180) / Math.PI + 360) % 360
        );
      }

      function onDeviceOrientation(event) {
        orientationEventSeen = true;
        const reading = extractHeading(event);
        if (reading == null) {
          return;
        }
        if (reading.absolute) {
          hasAbsoluteHeading = true;
        } else if (hasAbsoluteHeading) {
          return;
        }
        const rad = (reading.heading * Math.PI) / 180;
        if (headingSin == null) {
          headingSin = Math.sin(rad);
          headingCos = Math.cos(rad);
        } else {
          headingSin += HEADING_SMOOTHING * (Math.sin(rad) - headingSin);
          headingCos += HEADING_SMOOTHING * (Math.cos(rad) - headingCos);
        }
        headingFilterReady = true;
      }

      function startHeadingPublish() {
        if (headingPublishTimer != null) {
          return;
        }
        headingPublishTimer = setInterval(() => {
          const heading = getSmoothedHeading();
          if (heading == null) {
            return;
          }
          publishHeading(heading);
        }, HEADING_PUBLISH_MS);
      }

      async function requestOrientationPermission() {
        if (
          typeof DeviceOrientationEvent !== "undefined" &&
          typeof DeviceOrientationEvent.requestPermission === "function"
        ) {
          const response = await DeviceOrientationEvent.requestPermission();
          return response === "granted";
        }
        return true;
      }

      async function enableCompass() {
        const button = document.getElementById("enable-compass");
        const statusLine = document.getElementById("status-line");
        try {
          const granted = await requestOrientationPermission();
          if (!granted) {
            if (statusLine) {
              statusLine.textContent =
                "Compass permission denied. Distance still works without facing direction.";
              statusLine.className = "status warn";
            }
            return;
          }
        } catch (error) {
          // Continue; some browsers fire orientation without an explicit grant.
        }

        if ("ondeviceorientationabsolute" in window) {
          window.addEventListener(
            "deviceorientationabsolute",
            onDeviceOrientation,
            true
          );
        }
        window.addEventListener("deviceorientation", onDeviceOrientation, true);
        startHeadingPublish();
        // Also ask the map iframe to enable its own sensors (cone / follow).
        requestMapEnableCompass();

        compassEnabled = true;
        if (button) {
          button.textContent = "Compass enabled";
          button.classList.add("enabled");
          button.disabled = true;
        }
        if (statusLine && statusLine.className.indexOf("success") < 0) {
          statusLine.textContent =
            "Compass enabled. Follow the arrow toward the nearest grid point.";
          statusLine.className = "status";
        }

        setTimeout(() => {
          if (!orientationEventSeen && statusLine) {
            statusLine.textContent =
              "No compass data is coming from this device. Distance still works.";
            statusLine.className = "status warn";
          }
        }, 4000);
      }

      function updateRtkBar(payload) {
        const bar = document.getElementById("rtk-bar");
        if (!bar) {
          return;
        }
        if (payload.rtk_label == null && payload.gps_qual == null) {
          return;
        }
        let label = payload.rtk_label;
        let cssClass = payload.rtk_class || "unknown";
        if (!label) {
          const qual = Number(payload.gps_qual);
          if (qual === 4) {
            label = "RTK Fix";
            cssClass = "fix";
          } else if (qual === 5) {
            label = "RTK Float";
            cssClass = "float";
          } else if (Number.isFinite(qual)) {
            label = "No RTK";
            cssClass = "none";
          } else {
            label = "Waiting for GPS…";
            cssClass = "unknown";
          }
        }
        const sats = Number(payload.num_sats);
        const satsNote =
          Number.isFinite(sats) && sats > 0 ? ` · ${sats} sats` : "";
        bar.textContent = `${label}${satsNote}`;
        bar.className = `rtk-bar ${cssClass}`;
      }

      function updatePanel(payload) {
        updateRtkBar(payload);

        if (payload.sensorIssue) {
          panelSensorIssue = payload.sensorIssue;
        }
        const activeSensorIssue = payload.sensorIssue || panelSensorIssue;

        // Quality-only updates from Python skip the distance/arrow path.
        if (
          payload.distance === undefined &&
          payload.relative_bearing === undefined &&
          payload.reached === undefined &&
          payload.sensorIssue === undefined &&
          (payload.rtk_label != null || payload.gps_qual != null)
        ) {
          return;
        }

        const distanceValue = document.getElementById("distance-value");
        const statusLine = document.getElementById("status-line");
        const arrow = document.getElementById("nav-arrow");
        const reachedMark = document.getElementById("reached-mark");
        const targetSuffix = payload.target ? ` to ${payload.target}` : "";
        const toleranceM = Number(payload.tolerance_m);
        const tolCm = Number.isFinite(toleranceM)
          ? (toleranceM * 100).toFixed(toleranceM * 100 < 10 ? 1 : 0)
          : "1.5";
        const hasDistance =
          payload.distance != null && Number.isFinite(Number(payload.distance));

        distanceValue.textContent = formatDistance(payload.distance);

        if (payload.reached) {
          arrow.classList.add("reached", "disabled");
          reachedMark.classList.add("visible");
          statusLine.textContent = `${payload.target || "TARGET"} REACHED (within ${tolCm} cm)!`;
          statusLine.className = "status success";
          return;
        }

        reachedMark.classList.remove("visible");
        arrow.classList.remove("reached");

        const bearing = Number(payload.relative_bearing);
        if (Number.isFinite(bearing)) {
          arrow.classList.remove("disabled");
          setArrowRotation(bearing);
          statusLine.textContent = `Target${targetSuffix}`;
          statusLine.className = "status";
        } else if (activeSensorIssue === "insecure") {
          arrow.classList.add("disabled");
          setArrowRotation(null);
          statusLine.textContent = hasDistance
            ? `${formatDistance(payload.distance)} to target · open over HTTPS for heading`
            : "The browser blocks the compass over plain HTTP. Distance still works; open the app via HTTPS for the heading arrow.";
          statusLine.className = "status warn";
        } else if (activeSensorIssue === "desktop") {
          arrow.classList.add("disabled");
          setArrowRotation(null);
          statusLine.textContent = hasDistance
            ? `${formatDistance(payload.distance)}${targetSuffix} · use a phone or tablet for heading`
            : "Desktop detected: use a phone or tablet for compass heading. Distance still works here.";
          statusLine.className = "status warn";
        } else if (activeSensorIssue === "no-compass") {
          arrow.classList.add("disabled");
          setArrowRotation(null);
          statusLine.textContent =
            "This device is not reporting compass data. Distance still works, but the heading arrow is unavailable.";
          statusLine.className = "status warn";
        } else if (payload.distance == null) {
          arrow.classList.add("disabled");
          setArrowRotation(null);
          statusLine.textContent =
            "Waiting for RTK position and an active grid target.";
          statusLine.className = "status warn";
        } else if (!compassEnabled) {
          arrow.classList.add("disabled");
          setArrowRotation(null);
          statusLine.textContent =
            "Tap Enable compass above for facing direction.";
          statusLine.className = "status warn";
        } else {
          arrow.classList.add("disabled");
          setArrowRotation(null);
          statusLine.textContent =
            "Waiting for compass heading from this device.";
          statusLine.className = "status warn";
        }

        if (payload.accuracy != null && Number(payload.accuracy) > 5) {
          statusLine.textContent += " GPS accuracy is low.";
          statusLine.className = "status warn";
        }
      }

      const toleranceInput = document.getElementById("tolerance-cm");
      toleranceInput.addEventListener("change", publishTolerance);
      toleranceInput.addEventListener("input", publishTolerance);
      publishTolerance();

      function isNativeFullscreen() {
        return !!(
          document.fullscreenElement ||
          document.webkitFullscreenElement ||
          document.msFullscreenElement
        );
      }

      function updateFullscreenButton() {
        const button = document.getElementById("fullscreen-btn");
        const panel = document.getElementById("nav-panel");
        if (!button || !panel) {
          return;
        }
        const active = isNativeFullscreen() || panel.classList.contains("is-fullscreen");
        button.textContent = active ? "Exit fullscreen" : "Fullscreen";
      }

      async function toggleFullscreen() {
        const panel = document.getElementById("nav-panel");
        if (!panel) {
          return;
        }

        const active =
          isNativeFullscreen() || panel.classList.contains("is-fullscreen");

        // Prefer the browser Fullscreen API so the arrow can fill the phone.
        // Always keep .is-fullscreen in sync so light background + larger arrow
        // CSS apply even when the browser paints a dark fullscreen chrome.
        if (active) {
          try {
            if (isNativeFullscreen()) {
              const exit =
                document.exitFullscreen ||
                document.webkitExitFullscreen ||
                document.msExitFullscreen;
              if (exit) {
                await exit.call(document);
              }
            }
          } catch (error) {
            // Ignore API failures; CSS class below still exits the fallback.
          }
          panel.classList.remove("is-fullscreen");
        } else {
          try {
            const request =
              panel.requestFullscreen ||
              panel.webkitRequestFullscreen ||
              panel.msRequestFullscreen;
            if (request) {
              await request.call(panel);
            }
          } catch (error) {
            // Fall through to CSS-only fullscreen inside the iframe.
          }
          panel.classList.add("is-fullscreen");
        }
        updateFullscreenButton();
      }

      function onFullscreenChange() {
        const panel = document.getElementById("nav-panel");
        if (!panel) {
          return;
        }
        if (isNativeFullscreen()) {
          panel.classList.add("is-fullscreen");
        } else {
          panel.classList.remove("is-fullscreen");
        }
        updateFullscreenButton();
      }

      document
        .getElementById("fullscreen-btn")
        .addEventListener("click", toggleFullscreen);
      document.addEventListener("fullscreenchange", onFullscreenChange);
      document.addEventListener("webkitfullscreenchange", onFullscreenChange);

      try {
        if (typeof BroadcastChannel !== "undefined") {
          const navChannel = new BroadcastChannel(NAV_MESSAGE_TYPE);
          navChannel.onmessage = (event) => {
            updatePanel(event.data || {});
          };
        }
      } catch (error) {
        // Ignore channel errors on older browsers.
      }

      (function initCompassUi() {
        const button = document.getElementById("enable-compass");
        const desktopWarn = document.getElementById("desktop-warn");
        const ua = navigator.userAgent || "";
        const mobile = /Android|iPhone|iPad|iPod|Mobile|Tablet/i.test(ua);
        const hasTouch = (navigator.maxTouchPoints || 0) > 0;
        const desktop = !mobile && !hasTouch;

        if (!window.isSecureContext) {
          if (button) {
            button.style.display = "none";
          }
          updatePanel({ sensorIssue: "insecure" });
          return;
        }
        if (desktop) {
          // No device compass on desktop — hide the control, keep a clear warning.
          if (button) {
            button.style.display = "none";
          }
          if (desktopWarn) {
            desktopWarn.classList.add("visible");
          }
          updatePanel({ sensorIssue: "desktop" });
          return;
        }
        if (desktopWarn) {
          desktopWarn.classList.remove("visible");
        }
        if (button) {
          button.addEventListener("click", enableCompass);
        }
      })();
    </script>
  </body>
</html>
"""


def _encode_map_payload(payload):
    return base64.b64encode(json.dumps(payload).encode("utf-8")).decode("ascii")


def render_live_field_map(
    grid_points,
    user_lat,
    user_lon,
    center_lat,
    center_lon,
    zoom=19,
    height=520,
):
    # The nearest grid point is picked client-side and updated as you move.
    config = {
        "grid_points": grid_points,
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
        _LIVE_FIELD_MAP_HTML.replace("__MAP_CONFIG_B64__", _encode_map_payload(config))
        .replace("__MAP_MESSAGE_TYPE__", json.dumps(MAP_MESSAGE_TYPE))
    )
    render_html_embed(html, height=height)


def render_live_nav_panel(height=460):
    render_html_embed(_LIVE_NAV_PANEL_HTML, height=height)


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
    """Show only the closest infinite-lattice point in the live field view."""
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
    # First GPS reading used as the zero point for meter offsets.
    if "grid_ref_lat" not in st.session_state:
        st.session_state.grid_ref_lat = current_lat
    if "grid_ref_lon" not in st.session_state:
        st.session_state.grid_ref_lon = current_lon
    if "origin_offset_north_m" not in st.session_state:
        st.session_state.origin_offset_north_m = 0.0
    if "origin_offset_east_m" not in st.session_state:
        st.session_state.origin_offset_east_m = 0.0
    if "map_zoom" not in st.session_state:
        st.session_state.map_zoom = 19
    if "preview_orientation_deg" not in st.session_state:
        st.session_state.preview_orientation_deg = 0.0
    # Bumped whenever Python wants to force the placement map's view to the
    # preview origin (instead of leaving the user's pan/zoom alone).
    if "placement_view_seq" not in st.session_state:
        st.session_state.placement_view_seq = 0


def _meters_north_east(from_lat, from_lon, to_lat, to_lon):
    """Return (north_m, east_m) from one lat/lon to another."""
    north_m = (float(to_lat) - float(from_lat)) * 111320.0
    east_m = (
        (float(to_lon) - float(from_lon))
        * 111320.0
        * math.cos(math.radians(float(from_lat)))
    )
    return north_m, east_m


def _origin_latlon_from_offsets(ref_lat, ref_lon, north_m, east_m):
    """Move from the reference fix by north/east meters to a lat/lon origin."""
    lat, lon = offset_latlon(ref_lat, ref_lon, 0.0, float(north_m))
    return offset_latlon(lat, lon, 90.0, float(east_m))


def queue_origin_offset_inputs(north_m, east_m):
    """Stage meter offsets for the sidebar widgets (safe after they mount)."""
    st.session_state.pending_origin_offset_north_m = float(north_m)
    st.session_state.pending_origin_offset_east_m = float(east_m)


def apply_pending_origin_offset_inputs():
    """Apply staged offsets to widget keys. Call only before number_input mounts."""
    if "pending_origin_offset_north_m" in st.session_state:
        st.session_state.origin_offset_north_input = float(
            st.session_state.pop("pending_origin_offset_north_m")
        )
    if "pending_origin_offset_east_m" in st.session_state:
        st.session_state.origin_offset_east_input = float(
            st.session_state.pop("pending_origin_offset_east_m")
        )


def apply_origin_offsets(north_m, east_m, *, sync_inputs=True, snap_map=True):
    """Set the grid origin from meter offsets relative to the first GPS fix."""
    ref_lat = float(st.session_state.grid_ref_lat)
    ref_lon = float(st.session_state.grid_ref_lon)
    lat, lon = _origin_latlon_from_offsets(ref_lat, ref_lon, north_m, east_m)
    st.session_state.preview_origin_lat = lat
    st.session_state.preview_origin_lon = lon
    st.session_state.origin_offset_north_m = float(north_m)
    st.session_state.origin_offset_east_m = float(east_m)
    if sync_inputs:
        queue_origin_offset_inputs(north_m, east_m)
    if snap_map:
        st.session_state.placement_view_seq += 1


def sync_origin_offsets_from_latlon(origin_lat, origin_lon, *, sync_inputs=True):
    """Update stored meter offsets to match an absolute origin lat/lon."""
    ref_lat = float(st.session_state.grid_ref_lat)
    ref_lon = float(st.session_state.grid_ref_lon)
    north_m, east_m = _meters_north_east(ref_lat, ref_lon, origin_lat, origin_lon)
    st.session_state.origin_offset_north_m = float(north_m)
    st.session_state.origin_offset_east_m = float(east_m)
    if sync_inputs:
        queue_origin_offset_inputs(north_m, east_m)


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


def apply_grid_heading_deg(orientation_deg, *, sync_input=True):
    """Set grid heading and force the placement map to rotate screen-up to match."""
    orientation = float(orientation_deg) % 360.0
    st.session_state.preview_orientation_deg = orientation
    if sync_input:
        # Never write the widget key here — it may already be mounted (sidebar
        # apply path) or live in another fragment (map rotate). Stage instead.
        queue_grid_heading_input(orientation)
    st.session_state.placement_view_seq += 1


def reset_grid_to_current_position(current_lat, current_lon, line_count_m, line_count_e, spacing):
    st.session_state.grid_ref_lat = current_lat
    st.session_state.grid_ref_lon = current_lon
    apply_origin_offsets(0.0, 0.0, sync_inputs=True, snap_map=False)
    st.session_state.preview_origin_lat = current_lat
    st.session_state.preview_origin_lon = current_lon
    apply_grid_heading_deg(0.0)
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
    return file_path.name, orientation

def list_saved_grids():
    if not GRID_SAVE_DIR.is_dir():
        return []
    return sorted(GRID_SAVE_DIR.glob("*.json"))

def saved_grid_exists(name):
    return (GRID_SAVE_DIR / f"{grid_save_safe_name(name)}.json").is_file()

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
    # Treat the loaded origin as the new zero point for meter offsets.
    st.session_state.grid_ref_lat = origin_lat
    st.session_state.grid_ref_lon = origin_lon
    st.session_state.origin_offset_north_m = 0.0
    st.session_state.origin_offset_east_m = 0.0
    queue_origin_offset_inputs(0.0, 0.0)
    st.session_state.preview_orientation_deg = orientation
    st.session_state.grid_orientation_deg = orientation
    queue_grid_heading_input(orientation)
    st.session_state.pop("last_endless_window_key", None)
    # Open the placement view snapped to the loaded origin/orientation so the
    # user can confirm (or tweak) before pressing "Finalize grid location".
    st.session_state.grid_finalized = False
    st.session_state.placement_view_seq += 1
    st.session_state.live_map_mounted = False
    st.session_state.live_nav_panel_mounted = False

def apply_pending_saved_grid_choice():
    """Select a newly saved grid in the sidebar before its selectbox mounts."""
    choice = st.session_state.pop("pending_saved_grid_choice", None)
    if choice is not None:
        st.session_state.saved_grid_choice = choice

# --- STREAMLIT UI LAYOUT ---
st.set_page_config(page_title="RTK Live Map Guide", layout="wide")
st.title("RTK Live Map Guide")

# Streamlit fades elements to 33% opacity while a rerun is in flight
# ("stale" elements). Every map pan triggers a rerun, so without this the
# placement map grays out on each drag and feels broken.
st.markdown(
    """
    <style>
    [data-testid="stElementContainer"][data-stale="true"]:has(iframe.stCustomComponentV1) {
        opacity: 1 !important;
    }
    </style>
    """,
    unsafe_allow_html=True,
)

msg = poll_gps(timeout_s=0.2)
coords = latest_gps_fix()
if coords is None:
    # Brief blocking wait for the first fix, then yield back to Streamlit
    # instead of spinning forever in this process (which froze the UI).
    poll_gps(timeout_s=1.0)
    coords = latest_gps_fix()
if coords is None:
    if _gps_serial_error:
        st.error(f"Cannot read GPS serial port: {_gps_serial_error}")
    else:
        st.warning(
            f"Waiting for a valid GPS fix from the receiver on {_gps_serial_port}..."
        )
    time.sleep(0.5)
    st.rerun()

current_lat, current_lon = coords
init_placement_state(current_lat, current_lon)
apply_pending_grid_load()
apply_pending_saved_grid_choice()


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
            st.session_state.live_nav_panel_mounted = False
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
                "only the lattice point closest to your current position. "
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
            value=2.0,
            step=0.5,
            key="grid_spacing",
            help="Distance between neighboring grid points.",
        )

        if not st.session_state.get("grid_finalized"):
            apply_pending_origin_offset_inputs()
            if "origin_offset_north_input" not in st.session_state:
                st.session_state.origin_offset_north_input = float(
                    st.session_state.get("origin_offset_north_m", 0.0)
                )
            if "origin_offset_east_input" not in st.session_state:
                st.session_state.origin_offset_east_input = float(
                    st.session_state.get("origin_offset_east_m", 0.0)
                )
            st.caption(
                "Origin offset from the first GPS fix (or last Reset), in meters."
            )
            north_m = st.number_input(
                "North (+) / South (−) m",
                step=5.0,
                format="%.1f",
                key="origin_offset_north_input",
                help=(
                    "Move the grid origin north or south from your first GPS "
                    "reading. Step is 5 meters."
                ),
            )
            east_m = st.number_input(
                "East (+) / West (−) m",
                step=5.0,
                format="%.1f",
                key="origin_offset_east_input",
                help=(
                    "Move the grid origin east or west from your first GPS "
                    "reading. Step is 5 meters."
                ),
            )
            stored_north = float(st.session_state.get("origin_offset_north_m", 0.0))
            stored_east = float(st.session_state.get("origin_offset_east_m", 0.0))
            if (
                abs(float(north_m) - stored_north) > 0.05
                or abs(float(east_m) - stored_east) > 0.05
            ):
                apply_origin_offsets(
                    float(north_m),
                    float(east_m),
                    sync_inputs=False,
                    snap_map=True,
                )
                st.rerun(scope="app")

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
                apply_grid_heading_deg(typed, sync_input=False)
                st.rerun(scope="app")
        else:
            st.caption(
                f"Locked origin offset: "
                f"N {float(st.session_state.get('origin_offset_north_m', 0.0)):.1f} m, "
                f"E {float(st.session_state.get('origin_offset_east_m', 0.0)):.1f} m "
                f"· heading {current_grid_orientation_deg():.1f}° · "
                f"spacing {float(st.session_state.get('grid_spacing', 2.0)):.1f} m "
                "(Adjust grid placement to change)."
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
            st.session_state.live_nav_panel_mounted = False
            st.rerun(scope="app")

        _sync_grid_config_from_session()

    render_grid_settings(current_lat, current_lon)

    st.header("NTRIP")

    @st.fragment(run_every=1.0)
    def render_ntrip_controls():
        status = get_ntrip_status()
        active = status.get("desired")
        active_key = status.get("desired_key")

        if "ntrip_source" not in st.session_state:
            st.session_state.ntrip_source = (
                "Local base"
                if (active or {}).get("source") == "local"
                else "RTK2GO"
            )
        if "ntrip_mountpoint_input" not in st.session_state:
            st.session_state.ntrip_mountpoint_input = (
                (active or {}).get("mountpoint")
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
            value="CA_SanJose_ML_X5 ",
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
        else:
            st.caption("No saved grids yet.")

    render_saved_grids()

if not st.session_state.grid_finalized:
    st.subheader("Position Your Grid")
    st.caption(
        "Pan and zoom to place the origin, or set **North/East offsets** (5 m steps) "
        "and a **Grid heading** in the sidebar to rotate the map so the grid stays "
        "upright on screen. "
        "The grid origin (green) stays at the map center. "
        "Click **Finalize grid location** to confirm."
    )

    # The map lives in a fragment so each pan-end (which reports the new
    # origin as a component value) reruns only this block, not the whole
    # script. That keeps reruns fast on the Pi and the UI steady.
    @st.fragment
    def placement_map_fragment(user_lat, user_lon):
        placement_endless = bool(st.session_state.get("grid_endless", True))
        if placement_endless:
            placement_rows, placement_cols = 4, 4
        else:
            placement_rows = int(st.session_state.get("grid_dim_m", 4))
            placement_cols = int(st.session_state.get("grid_dim_e", 4))
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
            # Keep the meter-offset inputs aligned with a manual map pan.
            sync_origin_offsets_from_latlon(
                st.session_state.preview_origin_lat,
                st.session_state.preview_origin_lon,
                sync_inputs=True,
            )
            north_m = float(st.session_state.origin_offset_north_m)
            east_m = float(st.session_state.origin_offset_east_m)
            widget_north = st.session_state.get("origin_offset_north_input")
            widget_east = st.session_state.get("origin_offset_east_input")
            needs_offset_sync = (
                widget_north is None
                or widget_east is None
                or abs(float(widget_north) - north_m) > 0.05
                or abs(float(widget_east) - east_m) > 0.05
            )

            needs_heading_sync = False
            if map_state.get("bearing") is not None:
                bearing = float(map_state["bearing"]) % 360.0
                st.session_state.preview_orientation_deg = bearing
                widget_heading = st.session_state.get("grid_heading_input")
                needs_heading_sync = widget_heading is None or (
                    abs(_heading_delta_deg(float(widget_heading), bearing)) > 0.05
                )
                if needs_heading_sync:
                    queue_grid_heading_input(bearing)

            # Sidebar widgets live in another fragment — full app rerun is
            # required so pending North/East (and heading) values are applied
            # before those number_inputs remount.
            if needs_offset_sync or needs_heading_sync:
                st.rerun(scope="app")

    placement_map_fragment(current_lat, current_lon)

    @st.fragment(run_every=0.25)
    def refresh_placement_user_marker():
        poll_gps(timeout_s=0.1)
        coords = latest_gps_fix()
        if coords is None:
            return
        post_user_position_update(
            coords[0],
            coords[1],
            "placement_user_pos",
        )

    refresh_placement_user_marker()

    col_info, col_finalize = st.columns([2, 1])
    with col_info:
        orientation = current_grid_orientation_deg()
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
                st.session_state.live_nav_panel_mounted = False
            st.rerun()

else:
    df = st.session_state.grid_df

    flash = st.session_state.pop("grid_save_flash", None)
    if flash:
        st.success(flash)

    grid_points = grid_df_to_points(df)

    st.subheader("Live Navigation")

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

    if not st.session_state.get("live_nav_panel_mounted"):
        render_live_nav_panel(height=520)
        st.session_state.live_nav_panel_mounted = True

    col_adjust, col_save = st.columns(2)
    with col_adjust:
        if st.button("Adjust grid placement", use_container_width=True):
            st.session_state.grid_finalized = False
            # Keep the finalized orientation when returning to placement.
            if "grid_orientation_deg" in st.session_state:
                st.session_state.preview_orientation_deg = float(
                    st.session_state.grid_orientation_deg
                ) % 360.0
            # Snap the placement map back to the finalized origin when it remounts.
            st.session_state.placement_view_seq += 1
            st.session_state.live_map_mounted = False
            st.session_state.live_nav_panel_mounted = False
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
                    f"Grid saved to saved_grids/{saved_file} "
                    f"({orientation:.1f}° from north)"
                )
                st.session_state.pending_saved_grid_choice = Path(saved_file).stem
                st.session_state.live_map_mounted = False
                st.session_state.live_nav_panel_mounted = False
                st.rerun(scope="app")

        render_grid_save_controls()

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