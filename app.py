import base64
import json
import math
import re
import socket
import threading
import time
from pathlib import Path

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

# RTK2GO NTRIP caster (same settings as ntrip-rover.py)
NTRIP_HOST = "rtk2go.com"
NTRIP_PORT = 2101
NTRIP_USER_EMAIL = "voukich@gmail.com"
NTRIP_DEFAULT_MOUNTPOINT = "arnoldd"

# st.map size values are in meters (not pixels)
GRID_MARKER_SIZE_M = 0.8
TARGET_MARKER_SIZE_M = 1.2
USER_MARKER_SIZE_M = 0.6

# Shared GNSS serial port (GPS reads + NTRIP RTCM writes). Module-level so the
# NTRIP background thread can use it across Streamlit script reruns.
_gps_serial = None
_gps_serial_lock = threading.Lock()

# NTRIP background client state (survives Streamlit reruns).
_ntrip_lock = threading.Lock()
_ntrip_thread = None
_ntrip_stop = threading.Event()
_ntrip_desired_mountpoint = None  # None = disconnected / idle
_ntrip_status = {
    "state": "idle",  # idle | connecting | connected | error
    "mountpoint": None,
    "message": "Not connected",
}

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

def generate_grid(origin_lat, origin_lon, rows, cols, spacing_meters, orientation_deg=0):
    """Build a grid whose row axis points along orientation_deg (0=N, screen-up during placement)."""
    grid = []
    row_bearing = float(orientation_deg) % 360.0
    col_bearing = (row_bearing + 90.0) % 360.0
    for r in range(rows):
        stagger = (spacing_meters / 2.0) if r % 2 == 1 else 0.0
        for c in range(cols):
            point_lat, point_lon = offset_latlon(
                origin_lat, origin_lon, row_bearing, r * spacing_meters
            )
            point_lat, point_lon = offset_latlon(
                point_lat, point_lon, col_bearing, c * spacing_meters + stagger
            )
            grid.append({
                "Point": f"R{r+1}C{c+1}",
                "lat": point_lat,
                "lon": point_lon,
                "color": "#1E90FF",
                "size": GRID_MARKER_SIZE_M,
            })
    return pd.DataFrame(grid)

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
        /* Heading refreshes at 0.5 Hz; glide between steps instead of jumping. */
        transition: transform 1.8s ease;
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
      <div id="perm-banner" class="perm-banner">
        <p>Allow this device to use its compass so the map can rotate with your heading and show where you are facing. Your position comes from the RTK receiver on the stick.</p>
        <button id="enable-sensors" type="button">Enable compass</button>
      </div>
      <div id="sensor-warn" class="perm-banner desktop-warn hidden">
        <p id="sensor-warn-text"></p>
      </div>
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
          const anyActive = MAP_CONFIG.grid_points.some(
            (point) => !skippedPoints.has(point.point)
          );
          readout.textContent = anyActive
            ? "No grid points available."
            : "All grid points skipped — tap a yellow point to include it again.";
          readout.className = "nav-readout";
          publishNavUpdate({
            distance: null,
            direction: null,
            reached: false,
            target: null,
            accuracy: MAP_CONFIG.user_accuracy,
          });
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
      const HEADING_SMOOTHING = 0.2; // lower = smoother; filter runs every sensor tick
      // Cone refresh: 0.5 Hz. Map follow mode uses a separate ~30 fps loop so
      // terrain tracks the phone smoothly while the cone stays calm.
      const HEADING_REFRESH_MS = 2000;
      const MAP_BEARING_MS = 1000 / 30;
      // Cone deadband: ignore magnetometer wiggle while standing still.
      const HEADING_DEADBAND_DEG = 3;
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
        .getElementById("enable-sensors")
        .addEventListener("click", enableClientSensors);

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

def _open_gps_serial_if_needed():
    """Open the GNSS serial port if needed. Caller must hold `_gps_serial_lock`."""
    global _gps_serial
    if _gps_serial is not None:
        try:
            if _gps_serial.is_open:
                return _gps_serial
        except Exception:
            pass
    try:
        _gps_serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        _gps_serial.reset_input_buffer()
    except Exception:
        _gps_serial = None
    return _gps_serial


def _ensure_gps_serial():
    """Open (or re-open) the GNSS serial port used for NMEA reads and RTCM writes."""
    if "last_gps_msg" not in st.session_state:
        st.session_state.last_gps_msg = None

    with _gps_serial_lock:
        ser = _open_gps_serial_if_needed()
    st.session_state.gps_serial = ser


def _write_rtcm_to_gps(data):
    """Push RTCM correction bytes to the receiver (called from the NTRIP thread)."""
    with _gps_serial_lock:
        ser = _open_gps_serial_if_needed()
        if ser is None:
            return False
        try:
            ser.write(data)
            return True
        except Exception:
            return False


def get_ntrip_status():
    with _ntrip_lock:
        return dict(_ntrip_status)


def _set_ntrip_status(state, message, mountpoint=None):
    with _ntrip_lock:
        _ntrip_status["state"] = state
        _ntrip_status["message"] = message
        if mountpoint is not None:
            _ntrip_status["mountpoint"] = mountpoint


def _ntrip_request_headers(mountpoint):
    # RTK2GO: email as username, password "none" (same as ntrip-rover.py).
    credentials = f"{NTRIP_USER_EMAIL}:none"
    encoded_creds = base64.b64encode(credentials.encode("utf-8")).decode("utf-8")
    req = f"GET /{mountpoint} HTTP/1.0\r\n"
    req += "User-Agent: NTRIP PythonClient/1.0\r\n"
    req += f"Authorization: Basic {encoded_creds}\r\n"
    req += "Ntrip-Version: Ntrip/2.0\r\n"
    req += "Connection: close\r\n\r\n"
    return req.encode("utf-8")


def _ntrip_stream_once(mountpoint, stop_event):
    """Connect to one RTK2GO mountpoint and relay RTCM until stop or error."""
    _set_ntrip_status("connecting", f"Connecting to {mountpoint}…", mountpoint)
    with _gps_serial_lock:
        if _open_gps_serial_if_needed() is None:
            raise RuntimeError(f"Cannot open GNSS serial port {SERIAL_PORT}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(15.0)
    try:
        sock.connect((NTRIP_HOST, NTRIP_PORT))
        sock.sendall(_ntrip_request_headers(mountpoint))
        response = sock.recv(1024).decode("utf-8", errors="ignore")
        if not (
            "ICY 200 OK" in response
            or "HTTP/1.0 200 OK" in response
            or "HTTP/1.1 200 OK" in response
        ):
            raise RuntimeError(f"Caster rejected mountpoint: {response.strip()[:200]}")

        _set_ntrip_status(
            "connected",
            f"Streaming RTCM from {mountpoint}",
            mountpoint,
        )
        sock.settimeout(5.0)
        while not stop_event.is_set():
            with _ntrip_lock:
                desired = _ntrip_desired_mountpoint
            if desired != mountpoint:
                break
            try:
                data = sock.recv(2048)
            except socket.timeout:
                continue
            if not data:
                raise RuntimeError("Connection closed by caster")
            if not _write_rtcm_to_gps(data):
                raise RuntimeError("GNSS serial port write failed")
    finally:
        try:
            sock.close()
        except Exception:
            pass


def _ntrip_worker():
    """Background loop: keep streaming the selected mountpoint, reconnecting on errors."""
    delay = 5
    while not _ntrip_stop.is_set():
        with _ntrip_lock:
            mountpoint = _ntrip_desired_mountpoint
        if not mountpoint:
            _set_ntrip_status("idle", "Not connected", None)
            time.sleep(0.4)
            continue
        try:
            _ntrip_stream_once(mountpoint, _ntrip_stop)
        except Exception as exc:
            with _ntrip_lock:
                still_wanted = _ntrip_desired_mountpoint == mountpoint
            if still_wanted and not _ntrip_stop.is_set():
                delay = min(delay * 2, 300)
                _set_ntrip_status(
                    "error",
                    f"{exc}. Reconnecting in {delay}s…",
                    mountpoint,
                )
                # Interruptible wait so Disconnect is responsive.
                for _ in range(int(delay * 10)):
                    if _ntrip_stop.is_set():
                        break
                    with _ntrip_lock:
                        if _ntrip_desired_mountpoint != mountpoint:
                            break
                    time.sleep(0.1)


def _ensure_ntrip_thread():
    global _ntrip_thread
    with _ntrip_lock:
        if _ntrip_thread is not None and _ntrip_thread.is_alive():
            return
        _ntrip_stop.clear()
        _ntrip_thread = threading.Thread(
            target=_ntrip_worker,
            name="ntrip-rtcm",
            daemon=True,
        )
        _ntrip_thread.start()


def start_ntrip(mountpoint):
    """Start (or switch) RTK2GO streaming for the given mountpoint."""
    global _ntrip_desired_mountpoint
    cleaned = (mountpoint or "").strip().lstrip("/")
    if not cleaned:
        raise ValueError("Mountpoint cannot be empty")
    _ensure_ntrip_thread()
    with _ntrip_lock:
        _ntrip_desired_mountpoint = cleaned
    _set_ntrip_status("connecting", f"Connecting to {cleaned}…", cleaned)


def stop_ntrip():
    """Stop streaming corrections (worker thread stays idle for a later connect)."""
    global _ntrip_desired_mountpoint
    with _ntrip_lock:
        _ntrip_desired_mountpoint = None
    _set_ntrip_status("idle", "Not connected", None)


def poll_gps(timeout_s=0.15):
    """Read buffered serial data and return the latest valid GGA fix.

    Designed to be cheap on every Streamlit rerun/fragment tick: if a fix is
    already cached and the port has nothing waiting, return immediately.
    Never busy-spins the CPU while waiting for the first fix.
    """
    _ensure_gps_serial()
    ser = st.session_state.get("gps_serial")
    if ser is None:
        return st.session_state.get("last_gps_msg")

    deadline = time.time() + max(0.0, float(timeout_s))

    while True:
        line = None
        waiting = 0
        with _gps_serial_lock:
            try:
                waiting = ser.in_waiting
            except Exception:
                break

            if waiting > 0:
                try:
                    line = ser.readline().decode("ascii", errors="replace").strip()
                except Exception:
                    break

        if waiting <= 0:
            if st.session_state.get("last_gps_msg") is not None:
                break
            if time.time() >= deadline:
                break
            time.sleep(0.02)
            continue

        if line:
            for sentence in _iter_nmea_sentences(line):
                msg = _parse_gga_sentence(sentence)
                if msg is not None:
                    st.session_state.last_gps_msg = msg

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
    if "map_zoom" not in st.session_state:
        st.session_state.map_zoom = 19
    if "preview_orientation_deg" not in st.session_state:
        st.session_state.preview_orientation_deg = 0.0
    # Bumped whenever Python wants to force the placement map's view to the
    # preview origin (instead of leaving the user's pan/zoom alone).
    if "placement_view_seq" not in st.session_state:
        st.session_state.placement_view_seq = 0

def reset_grid_to_current_position(current_lat, current_lon, line_count_m, line_count_e, spacing):
    st.session_state.preview_origin_lat = current_lat
    st.session_state.preview_origin_lon = current_lon
    st.session_state.preview_orientation_deg = 0.0
    st.session_state.placement_view_seq += 1
    if st.session_state.grid_finalized:
        st.session_state.grid_df = generate_grid(
            current_lat,
            current_lon,
            line_count_m,
            line_count_e,
            spacing,
            st.session_state.preview_orientation_deg,
        )

def finalize_grid_location(line_count_m, line_count_e, spacing):
    orientation = float(st.session_state.get("preview_orientation_deg", 0.0)) % 360.0
    st.session_state.preview_orientation_deg = orientation
    # Frozen copy used by save so orientation can't drift after finalize.
    st.session_state.grid_orientation_deg = orientation
    st.session_state.grid_df = generate_grid(
        st.session_state.preview_origin_lat,
        st.session_state.preview_origin_lon,
        line_count_m,
        line_count_e,
        spacing,
        orientation,
    )
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
    except (KeyError, TypeError, ValueError):
        st.error("That saved grid file is invalid and can't be loaded.")
        return
    st.session_state.grid_dim_m = rows
    st.session_state.grid_dim_e = cols
    st.session_state.grid_spacing = spacing
    st.session_state.preview_origin_lat = origin_lat
    st.session_state.preview_origin_lon = origin_lon
    st.session_state.preview_orientation_deg = orientation
    st.session_state.grid_orientation_deg = orientation
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
if msg is None:
    # Brief blocking wait for the first fix, then yield back to Streamlit
    # instead of spinning forever in this process (which froze the UI).
    msg = poll_gps(timeout_s=1.0)
if msg is None:
    st.warning("Waiting for GPS fix from the receiver...")
    time.sleep(0.5)
    st.rerun()

current_lat, current_lon = float(msg.latitude), float(msg.longitude)
init_placement_state(current_lat, current_lon)
apply_pending_grid_load()
apply_pending_saved_grid_choice()


with st.sidebar:
    st.header("NTRIP (RTK2GO)")

    @st.fragment(run_every=2.0)
    def render_ntrip_controls():
        if "ntrip_mountpoint_input" not in st.session_state:
            st.session_state.ntrip_mountpoint_input = NTRIP_DEFAULT_MOUNTPOINT
        mountpoint = st.text_input(
            "Mountpoint",
            key="ntrip_mountpoint_input",
            help=f"RTK2GO caster mountpoint on {NTRIP_HOST}:{NTRIP_PORT}",
        )
        col_connect, col_disconnect = st.columns(2)
        with col_connect:
            if st.button("Connect", use_container_width=True, type="primary"):
                try:
                    start_ntrip(mountpoint)
                except ValueError as exc:
                    st.error(str(exc))
        with col_disconnect:
            if st.button("Disconnect", use_container_width=True):
                stop_ntrip()

        status = get_ntrip_status()
        state = status.get("state", "idle")
        message = status.get("message", "")
        if state == "connected":
            st.success(message)
        elif state == "connecting":
            st.info(message)
        elif state == "error":
            st.warning(message)
        else:
            st.caption(message)
        st.caption(f"Caster: {NTRIP_HOST}:{NTRIP_PORT} · user: {NTRIP_USER_EMAIL}")

    render_ntrip_controls()

    st.header("Grid Settings")

    @st.fragment
    def render_grid_settings(current_lat, current_lon):
        grid_lines_m = st.number_input("Rows", min_value=1, value=4, key="grid_dim_m")
        grid_lines_e = st.number_input("Columns", min_value=1, value=4, key="grid_dim_e")
        spacing = st.number_input(
            "Spacing (Meters)", min_value=0.5, value=2.0, step=0.5, key="grid_spacing"
        )

        if st.button("Reset grid to current position", use_container_width=True):
            # Fragment args are frozen at the last full-page run; read the
            # freshest fix (kept up to date by the GPS polling fragments).
            latest_msg = st.session_state.get("last_gps_msg")
            if latest_msg is not None:
                current_lat = float(latest_msg.latitude)
                current_lon = float(latest_msg.longitude)
            reset_grid_to_current_position(
                current_lat, current_lon, grid_lines_m, grid_lines_e, spacing
            )
            # reset_grid_to_current_position bumped placement_view_seq, which
            # makes the placement map snap to the new origin on the next full
            # run. The live map doesn't handle view/grid messages, so remount
            # it instead. The button sits in a fragment, so the rerun must be
            # app-scoped to reach the maps in the main body.
            st.session_state.live_map_mounted = False
            st.session_state.live_nav_panel_mounted = False
            st.rerun(scope="app")

        grid_config = (int(grid_lines_m), int(grid_lines_e), float(spacing))
        if st.session_state.get("last_grid_config_sent") != grid_config:
            st.session_state.last_grid_config_sent = grid_config
            post_map_message({
                "action": "updateGrid",
                "dim_m": grid_config[0],
                "dim_e": grid_config[1],
                "spacing_m": grid_config[2],
            })

        st.divider()
        saved_grids = list_saved_grids()
        if saved_grids:
            chosen_grid = st.selectbox(
                "Saved grids",
                [f.stem for f in saved_grids],
                key="saved_grid_choice",
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
                    # Widgets are already instantiated this run; stage the
                    # load and apply it at the top of the next full run.
                    st.session_state.pending_grid_load = grid_data
                    st.rerun(scope="app")
        else:
            st.caption("No saved grids yet.")

    render_grid_settings(current_lat, current_lon)

if not st.session_state.grid_finalized:
    st.subheader("Position Your Grid")
    st.caption(
        "Pan, rotate, and zoom map to achieve desired grid orientation and position "
        "The grid origin (in green) stays at the map center. "
        "Click **Finalize grid location** to confirm the grid position."
    )

    # The map lives in a fragment so each pan-end (which reports the new
    # origin as a component value) reruns only this block, not the whole
    # script. That keeps reruns fast on the Pi and the UI steady.
    @st.fragment
    def placement_map_fragment(user_lat, user_lon):
        map_state = render_placement_map(
            center_lat=st.session_state.preview_origin_lat,
            center_lon=st.session_state.preview_origin_lon,
            zoom=st.session_state.map_zoom,
            line_count_m=int(st.session_state.get("grid_dim_m", 4)),
            line_count_e=int(st.session_state.get("grid_dim_e", 4)),
            spacing=float(st.session_state.get("grid_spacing", 2.0)),
            user_lat=user_lat,
            user_lon=user_lon,
            height=520,
            view_seq=st.session_state.placement_view_seq,
            bearing=float(st.session_state.get("preview_orientation_deg", 0.0)),
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
            if map_state.get("bearing") is not None:
                st.session_state.preview_orientation_deg = (
                    float(map_state["bearing"]) % 360.0
                )

    placement_map_fragment(current_lat, current_lon)

    @st.fragment(run_every=1.0)
    def refresh_placement_user_marker():
        msg = poll_gps(timeout_s=0.1)
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

    flash = st.session_state.pop("grid_save_flash", None)
    if flash:
        st.success(flash)

    grid_points = [
        {"point": row["Point"], "lat": row["lat"], "lon": row["lon"]}
        for _, row in df.iterrows()
    ]

    st.subheader("Live Field View")
    st.caption(
        "Tap **Enable compass** on the map. Your position comes from the RTK receiver on "
        "the stick; you are guided to the nearest grid point (highlighted green), and "
        "directions are relative to the way you are facing. "
        "Tap a blue grid point to mark it yellow and skip it as a target; tap again to "
        "include it. "
        "After finalize, the grid is locked to the terrain — rotating the map (or "
        "compass-follow) turns the view while keeping grid points on the ground. "
        "To change grid orientation, use **Adjust grid placement**."
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
        msg = poll_gps(timeout_s=0.1)
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