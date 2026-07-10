import base64
import json
import math
import time

import pandas as pd
import pynmea2
import serial
import streamlit as st

from placement_map_html import (
    MAP_MESSAGE_TYPE,
    post_map_message,
    render_html_embed,
    render_placement_map,
)
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
            # Convert the meter offset into degrees
            offset = (spacing_meters / 2) / lon_degree_meters
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

      .perm-banner {
        position: absolute;
        top: 10px;
        left: 10px;
        right: 10px;
        z-index: 1001;
        background: rgba(255, 255, 255, 0.96);
        border: 1px solid rgba(0, 0, 0, 0.15);
        border-radius: 8px;
        padding: 10px 12px;
        box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
        pointer-events: auto;
      }

      .perm-banner p {
        margin: 0 0 8px;
        font: 13px/1.4 sans-serif;
        color: #333;
      }

      .perm-banner button {
        border: none;
        border-radius: 6px;
        padding: 8px 12px;
        font: 13px/1.2 sans-serif;
        font-weight: 600;
        color: #fff;
        background: #1565c0;
        cursor: pointer;
      }

      .perm-banner button:hover {
        background: #0d47a1;
      }

      .perm-banner.hidden {
        display: none;
      }

      .desktop-warn {
        background: rgba(255, 243, 224, 0.96);
        border-color: rgba(239, 108, 0, 0.35);
      }

      .desktop-warn p {
        color: #bf360c;
      }
    </style>
  </head>
  <body>
    <div class="map-shell">
      <div id="map-config" data-config-b64="__MAP_CONFIG_B64__" hidden></div>
      <div id="map"></div>
      <div id="perm-banner" class="perm-banner">
        <p>Allow this device to use its compass so the map can show where you are facing. Your position comes from the RTK receiver on the stick.</p>
        <button id="enable-sensors" type="button">Enable compass</button>
      </div>
      <div id="sensor-warn" class="perm-banner desktop-warn hidden">
        <p id="sensor-warn-text"></p>
      </div>
      <div class="map-hud">
        <div id="nav-readout" class="nav-readout">Waiting for RTK position from the receiver...</div>
      </div>
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

      const NAV_MESSAGE_TYPE = "gps-stick-nav";
      let sensorIssue = null; // "insecure" | "desktop" | "no-compass"
      let orientationEventSeen = false;

      function isLikelyDesktop() {
        const ua = navigator.userAgent || "";
        const mobile = /Android|iPhone|iPad|iPod|Mobile|Tablet/i.test(ua);
        const hasTouch = (navigator.maxTouchPoints || 0) > 0;
        return !mobile && !hasTouch;
      }

      function showSensorWarning(issue, text) {
        sensorIssue = issue;
        const banner = document.getElementById("sensor-warn");
        const bannerText = document.getElementById("sensor-warn-text");
        if (banner && bannerText) {
          bannerText.textContent = text;
          banner.classList.remove("hidden");
        }
        publishNavUpdate({
          distance: null,
          direction: null,
          reached: false,
          sensorIssue: issue,
        });
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

      function destinationLatLng(lat, lon, bearingDeg, distanceM) {
        const earthRadius = 6371000;
        const bearing = (bearingDeg * Math.PI) / 180;
        const lat1 = (lat * Math.PI) / 180;
        const lon1 = (lon * Math.PI) / 180;
        const lat2 = Math.asin(
          Math.sin(lat1) * Math.cos(distanceM / earthRadius) +
            Math.cos(lat1) *
              Math.sin(distanceM / earthRadius) *
              Math.cos(bearing)
        );
        const lon2 =
          lon1 +
          Math.atan2(
            Math.sin(bearing) *
              Math.sin(distanceM / earthRadius) *
              Math.cos(lat1),
            Math.cos(distanceM / earthRadius) -
              Math.sin(lat1) * Math.sin(lat2)
          );
        return [(lat2 * 180) / Math.PI, (lon2 * 180) / Math.PI];
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

        if (nearest && nearest.point !== MAP_CONFIG.target_point) {
          MAP_CONFIG.target_point = nearest.point;
          MAP_CONFIG.target_lat = nearest.lat;
          MAP_CONFIG.target_lon = nearest.lon;
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

        if (!Number.isFinite(userLat) || !Number.isFinite(userLon)) {
          readout.textContent = "Waiting for RTK position from the receiver...";
          readout.className = "nav-readout";
          return;
        }

        refreshNearestTarget(userLat, userLon);

        const targetLat = Number(MAP_CONFIG.target_lat);
        const targetLon = Number(MAP_CONFIG.target_lon);
        const targetName = MAP_CONFIG.target_point;

        if (!Number.isFinite(targetLat) || !Number.isFinite(targetLon)) {
          readout.textContent = "No grid points available.";
          readout.className = "nav-readout";
          return;
        }

        const { distance, bearing } = getDistanceAndBearing(
          userLat,
          userLon,
          targetLat,
          targetLon
        );

        if (distance < 0.015) {
          readout.textContent = `${targetName} REACHED (within 1.5 cm)!`;
          readout.className = "nav-readout reached";
          publishNavUpdate({
            distance,
            direction: "forward",
            reached: true,
            target: targetName,
            accuracy: MAP_CONFIG.user_accuracy,
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
            direction: null,
            reached: false,
            target: targetName,
            accuracy,
            sensorIssue,
          });
          return;
        }

        const direction = getRelativeDirection(heading, bearing);
        readout.textContent =
          `${directionLabel(direction)} · ${distance.toFixed(2)} m to ${targetName}` +
          accuracyNote;
        publishNavUpdate({
          distance,
          direction,
          reached: false,
          target: targetName,
          accuracy,
        });
      }

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

      function drawFacingCone() {
        facingLayer.clearLayers();
        const heading = Number(MAP_CONFIG.user_heading);
        const lat = Number(MAP_CONFIG.user_lat);
        const lon = Number(MAP_CONFIG.user_lon);
        if (!Number.isFinite(heading) || !Number.isFinite(lat) || !Number.isFinite(lon)) {
          return;
        }

        const coneLength = 14;
        const coneSpread = 32;
        const center = [lat, lon];
        const left = destinationLatLng(lat, lon, heading - coneSpread / 2, coneLength);
        const tip = destinationLatLng(lat, lon, heading, coneLength);
        const right = destinationLatLng(lat, lon, heading + coneSpread / 2, coneLength);

        L.polygon([center, left, tip, right], {
          color: "#FF4444",
          fillColor: "#FF4444",
          fillOpacity: 0.28,
          weight: 1,
          interactive: false,
        }).addTo(facingLayer);
      }

      function drawUserMarker() {
        userMarker.clearLayers();
        drawFacingCone();
        L.circleMarker([MAP_CONFIG.user_lat, MAP_CONFIG.user_lon], {
          radius: 7,
          color: "#FF0000",
          fillColor: "#FF0000",
          fillOpacity: 1.0,
          weight: 2,
        })
          .bindPopup("Your position")
          .addTo(userMarker);
        updateNavigationHud();
      }

      // Heading filter state. Android fires BOTH deviceorientationabsolute
      // (true north) and plain deviceorientation (relative, arbitrary zero);
      // mixing them makes the heading jump. Once an absolute source is seen,
      // relative readings are discarded.
      let hasAbsoluteHeading = false;
      let headingSin = null;
      let headingCos = null;
      let lastHeadingDrawMs = 0;
      const HEADING_SMOOTHING = 0.25; // 0..1, lower = smoother but laggier
      const HEADING_REDRAW_MS = 100;
      const HEADING_MIN_DELTA_DEG = 1.5;

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
          const banner = document.getElementById("sensor-warn");
          if (banner) {
            banner.classList.add("hidden");
          }
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
        const smoothed =
          ((Math.atan2(headingSin, headingCos) * 180) / Math.PI + 360) % 360;

        // Sensors fire up to 60 Hz; cap redraws and skip imperceptible changes.
        const now = Date.now();
        if (now - lastHeadingDrawMs < HEADING_REDRAW_MS) {
          return;
        }
        const previous = Number(MAP_CONFIG.user_heading);
        if (Number.isFinite(previous)) {
          const delta = Math.abs(((smoothed - previous + 540) % 360) - 180);
          if (delta < HEADING_MIN_DELTA_DEG) {
            return;
          }
        }
        lastHeadingDrawMs = now;
        MAP_CONFIG.user_heading = smoothed;
        drawUserMarker();
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

        setTimeout(() => {
          if (!orientationEventSeen && sensorIssue == null) {
            showSensorWarning(
              "no-compass",
              "No compass data is coming from this device. Distance still works, but facing direction will be unavailable."
            );
          }
        }, 4000);

        document.getElementById("perm-banner").classList.add("hidden");
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
        }
      }

      const map = L.map("map", {
        center: [MAP_CONFIG.center_lat, MAP_CONFIG.center_lon],
        zoom: MAP_CONFIG.zoom,
        zoomControl: true,
        maxZoom: 25,
      });

      L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        attribution: "&copy; OpenStreetMap contributors",
        maxZoom: 25,
        maxNativeZoom: 19,
      }).addTo(map);

      L.control
        .scale({ position: "bottomright", metric: true, imperial: false, maxWidth: 120 })
        .addTo(map);

      const gridLayer = L.layerGroup().addTo(map);
      const facingLayer = L.layerGroup().addTo(map);
      const userMarker = L.layerGroup().addTo(map);

      drawGridMarkers();
      drawUserMarker();

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
        .getElementById("enable-sensors")
        .addEventListener("click", enableClientSensors);

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
        font-family: sans-serif;
      }

      .panel {
        border: 1px solid rgba(0, 0, 0, 0.08);
        border-radius: 8px;
        padding: 12px 14px;
        background: #fafafa;
      }

      .metric-row {
        display: flex;
        gap: 12px;
      }

      .metric {
        flex: 1;
        background: #fff;
        border: 1px solid rgba(0, 0, 0, 0.08);
        border-radius: 8px;
        padding: 10px 12px;
      }

      .metric-label {
        font-size: 12px;
        color: #666;
        margin-bottom: 4px;
      }

      .metric-value {
        font-size: 20px;
        font-weight: 600;
        color: #222;
      }

      .status {
        margin-top: 10px;
        font-size: 14px;
        color: #444;
      }

      .status.success {
        color: #1b5e20;
        font-weight: 600;
      }

      .status.warn {
        color: #e65100;
      }
    </style>
  </head>
  <body>
    <div class="panel">
      <div class="metric-row">
        <div class="metric">
          <div class="metric-label">Distance to Target</div>
          <div id="distance-value" class="metric-value">--</div>
        </div>
        <div class="metric">
          <div class="metric-label">Direction</div>
          <div id="direction-value" class="metric-value">--</div>
        </div>
      </div>
      <div id="status-line" class="status">Enable the compass on the map above. Position comes from the RTK receiver.</div>
    </div>
    <script>
      const NAV_MESSAGE_TYPE = "gps-stick-nav";
      const directionLabels = {
        forward: "Head forward",
        back: "Head back",
        left: "Head left",
        right: "Head right",
      };

      function updatePanel(payload) {
        const distanceValue = document.getElementById("distance-value");
        const directionValue = document.getElementById("direction-value");
        const statusLine = document.getElementById("status-line");
        const targetSuffix = payload.target ? ` to ${payload.target}` : "";

        if (payload.reached) {
          distanceValue.textContent = "0.00 m";
          directionValue.textContent = "Arrived";
          statusLine.textContent = `${payload.target || "TARGET"} REACHED (within 1.5 cm)!`;
          statusLine.className = "status success";
          return;
        }

        if (payload.distance != null) {
          distanceValue.textContent = `${Number(payload.distance).toFixed(2)} m`;
        }

        if (payload.direction) {
          directionValue.textContent = directionLabels[payload.direction];
          statusLine.textContent = `${directionLabels[payload.direction]} for ${Number(payload.distance).toFixed(2)} meters${targetSuffix}`;
          statusLine.className = "status";
        } else if (payload.sensorIssue === "insecure") {
          directionValue.textContent = "Unavailable";
          statusLine.textContent =
            "The browser blocks the compass over plain HTTP. Distance from the RTK receiver still works; open the app via HTTPS for directions.";
          statusLine.className = "status warn";
        } else if (payload.sensorIssue === "desktop") {
          directionValue.textContent = "Unavailable";
          statusLine.textContent =
            "Desktop detected: compass orientation is not available. Use a phone or tablet for direction guidance.";
          statusLine.className = "status warn";
        } else if (payload.sensorIssue === "no-compass") {
          directionValue.textContent = "Unavailable";
          statusLine.textContent =
            "This device is not reporting compass data. Distance still works, but facing direction is unavailable.";
          statusLine.className = "status warn";
        } else {
          directionValue.textContent = "--";
          statusLine.textContent = "Waiting for compass heading from this device.";
          statusLine.className = "status warn";
        }

        if (payload.accuracy != null && Number(payload.accuracy) > 5) {
          statusLine.textContent += " GPS accuracy is low.";
          statusLine.className = "status warn";
        }
      }

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

      (function showSensorWarningOnLoad() {
        if (!window.isSecureContext) {
          updatePanel({ sensorIssue: "insecure" });
          return;
        }
        const ua = navigator.userAgent || "";
        const mobile = /Android|iPhone|iPad|iPod|Mobile|Tablet/i.test(ua);
        const hasTouch = (navigator.maxTouchPoints || 0) > 0;
        if (!mobile && !hasTouch) {
          updatePanel({ sensorIssue: "desktop" });
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
        "center_lat": center_lat,
        "center_lon": center_lon,
        "zoom": zoom,
    }
    html = (
        _LIVE_FIELD_MAP_HTML.replace("__MAP_CONFIG_B64__", _encode_map_payload(config))
        .replace("__MAP_MESSAGE_TYPE__", json.dumps(MAP_MESSAGE_TYPE))
    )
    render_html_embed(html, height=height)


def render_live_nav_panel(height=120):
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

    # Non-blocking polls get a hard time budget so serial reads can't stall
    # the Streamlit session (fragments in a session run one at a time).
    deadline = time.time() + (timeout_s if block_until_fix else 0.15)

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

        if time.time() >= deadline:
            break

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

        grid_config = (int(grid_lines_m), int(grid_lines_e), float(spacing))
        if st.session_state.get("last_grid_config_sent") != grid_config:
            st.session_state.last_grid_config_sent = grid_config
            post_map_message({
                "action": "updateGrid",
                "dim_m": grid_config[0],
                "dim_e": grid_config[1],
                "spacing_m": grid_config[2],
            })

    render_grid_settings(current_lat, current_lon)

if not st.session_state.grid_finalized:
    sync_placement_from_query_params()

    st.subheader("Position Your Grid")
    st.caption(
        "Pan and zoom the map to slide the terrain under the preview grid. "
        "The grid origin (in green) stays at the map center. "
        "Click **Finalize grid location** to confirm the grid position."
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
        st.markdown("**Legend:** green = grid origin · blue = preview points · red = your position")
    with col_finalize:
        if st.button("Finalize grid location", type="primary", use_container_width=True):
            finalize_grid_location(
                int(st.session_state.grid_dim_m),
                int(st.session_state.grid_dim_e),
                float(st.session_state.grid_spacing),
            )
            st.session_state.placement_map_mounted = False
            st.session_state.live_map_mounted = False
            st.session_state.live_nav_panel_mounted = False
            st.rerun()

else:
    df = st.session_state.grid_df

    if st.button("Adjust grid placement"):
        st.session_state.grid_finalized = False
        st.session_state.placement_map_mounted = False
        st.session_state.live_map_mounted = False
        st.session_state.live_nav_panel_mounted = False
        st.rerun()

    grid_points = [
        {"point": row["Point"], "lat": row["lat"], "lon": row["lon"]}
        for _, row in df.iterrows()
    ]

    st.subheader("Live Field View")
    st.caption(
        "Tap **Enable compass** on the map. Your position comes from the RTK receiver on "
        "the stick; you are guided to the nearest grid point (highlighted green), and "
        "directions are relative to the way you are facing."
    )
    if not st.session_state.get("live_map_mounted"):
        render_live_field_map(
            grid_points=grid_points,
            user_lat=current_lat,
            user_lon=current_lon,
            center_lat=current_lat,
            center_lon=current_lon,
            zoom=19,
            height=520,
        )
        st.session_state.live_map_mounted = True

    if not st.session_state.get("live_nav_panel_mounted"):
        render_live_nav_panel()
        st.session_state.live_nav_panel_mounted = True

    @st.fragment(run_every=0.5)
    def refresh_live_user_marker():
        msg = poll_gps()
        if msg is None:
            return
        if int(msg.gps_qual) in [0, 1]:
            st.warning("RTK fix quality is low. Please check your base station connection.")
        post_user_position_update(
            float(msg.latitude),
            float(msg.longitude),
            "live_user_pos",
        )

    refresh_live_user_marker()