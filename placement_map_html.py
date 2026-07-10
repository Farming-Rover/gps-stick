import base64
import json

import streamlit.components.v1 as components

MAP_MESSAGE_TYPE = "gps-stick-map"

_PLACEMENT_MAP_HTML = """<!DOCTYPE html>
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

      .center-reticle {
        pointer-events: none;
        position: absolute;
        top: 50%;
        left: 50%;
        width: 26px;
        height: 26px;
        margin-left: -13px;
        margin-top: -13px;
        z-index: 1000;
        border: 2px solid rgba(0, 200, 83, 0.95);
        border-radius: 50%;
        box-shadow: 0 0 0 1px rgba(255, 255, 255, 0.85);
      }

      .center-reticle::before,
      .center-reticle::after {
        content: "";
        position: absolute;
        background: rgba(0, 200, 83, 0.95);
      }

      .center-reticle::before {
        left: 50%;
        top: 3px;
        width: 2px;
        height: 20px;
        margin-left: -1px;
      }

      .center-reticle::after {
        top: 50%;
        left: 3px;
        width: 20px;
        height: 2px;
        margin-top: -1px;
      }

      .map-hud {
        position: absolute;
        left: 10px;
        bottom: 10px;
        z-index: 1000;
        display: flex;
        flex-direction: column;
        gap: 8px;
        align-items: flex-start;
      }

      .origin-readout {
        background: rgba(255, 255, 255, 0.92);
        border: 1px solid rgba(0, 0, 0, 0.15);
        border-radius: 6px;
        padding: 6px 10px;
        font: 12px/1.4 sans-serif;
        color: #222;
        pointer-events: none;
      }

      .apply-origin-btn {
        border: none;
        border-radius: 6px;
        padding: 8px 12px;
        font: 13px/1.2 sans-serif;
        font-weight: 600;
        color: #fff;
        background: #1565c0;
        box-shadow: 0 1px 4px rgba(0, 0, 0, 0.2);
        cursor: pointer;
      }

      .apply-origin-btn:hover {
        background: #0d47a1;
      }
    </style>
  </head>
  <body>
    <div class="map-shell">
      <div id="map-config" data-config-b64="__MAP_CONFIG_B64__" hidden></div>
      <div id="map"></div>
      <div class="center-reticle" title="Grid origin (R1C1)"></div>
      <div class="map-hud">
        <div id="origin-readout" class="origin-readout"></div>
        <button id="apply-origin" class="apply-origin-btn" type="button">
          Apply grid origin
        </button>
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

      let suppressEvents = false;

      function asCount(value, fallback) {
        const parsed = Math.round(Number(value));
        if (!Number.isFinite(parsed) || parsed < 1) {
          return fallback;
        }
        return parsed;
      }

      function generateGrid(originLat, originLon, lineCountM, lineCountE, spacing) {
        const countM = asCount(lineCountM, 4);
        const countE = asCount(lineCountE, 4);
        const spacingMeters = Number(spacing);
        const latDegreeMeters = 111132.92;
        const lonDegreeMeters =
          (40075000 * Math.cos((originLat * Math.PI) / 180)) / 360;
        const points = [];

        for (let indexM = 0; indexM < countM; indexM++) {
          const offset = indexM % 2 === 1 ? spacingMeters / 2 : 0;
          for (let indexE = 0; indexE < countE; indexE++) {
            const deltaNorth = indexM * spacingMeters;
            const deltaEast = indexE * spacingMeters;
            points.push({
              label: `R${indexM + 1}C${indexE + 1}`,
              lat: originLat + deltaNorth / latDegreeMeters,
              lon: originLon + deltaEast / lonDegreeMeters + offset,
              isOrigin: indexM === 0 && indexE === 0,
            });
          }
        }

        return points;
      }

      function updateOriginReadout(lat, lng, zoom) {
        const readout = document.getElementById("origin-readout");
        readout.textContent =
          `R1C1: ${lat.toFixed(8)}, ${lng.toFixed(8)} · ` +
          `${asCount(MAP_CONFIG.dim_m, 4)}x${asCount(MAP_CONFIG.dim_e, 4)} · zoom ${zoom}`;
      }

      function drawGrid(originLat, originLon) {
        gridLayer.clearLayers();

        const points = generateGrid(
          originLat,
          originLon,
          MAP_CONFIG.dim_m,
          MAP_CONFIG.dim_e,
          MAP_CONFIG.spacing_m
        );

        points.forEach((point) => {
          L.circleMarker([point.lat, point.lon], {
            radius: point.isOrigin ? 9 : 6,
            color: point.isOrigin ? "#00C853" : "#1E90FF",
            fillColor: point.isOrigin ? "#00C853" : "#1E90FF",
            fillOpacity: point.isOrigin ? 0.85 : 0.45,
            weight: 2,
          })
            .bindPopup(point.label)
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

      function pushViewportToStreamlit() {
        const center = map.getCenter();
        const baseUrl = window.location.origin + window.location.pathname;
        const nextUrl = "/?origin_lat=" + center.lat.toFixed(8) + "&origin_lon=" + center.lng.toFixed(8) + "&map_zoom=" + String(map.getZoom());
        try {
          window.top.location.href = nextUrl;
        } catch (e) {
          window.parent.location.href = nextUrl;
        }
      }

      function onViewportChanged() {
        if (suppressEvents) {
          return;
        }
        const center = map.getCenter();
        drawGrid(center.lat, center.lng);
        updateOriginReadout(center.lat, center.lng, map.getZoom());
      }

      function applyView(centerLat, centerLon, zoom) {
        suppressEvents = true;
        map.setView([centerLat, centerLon], zoom, { animate: false });
        suppressEvents = false;
        drawGrid(centerLat, centerLon);
        updateOriginReadout(centerLat, centerLon, zoom);
      }

      function applyUpdateGrid(data) {
        let changed = false;

        if (data.dim_m != null) {
          const nextM = asCount(data.dim_m, MAP_CONFIG.dim_m || 4);
          if (nextM !== MAP_CONFIG.dim_m) {
            MAP_CONFIG.dim_m = nextM;
            changed = true;
          }
        }
        if (data.dim_e != null) {
          const nextE = asCount(data.dim_e, MAP_CONFIG.dim_e || 4);
          if (nextE !== MAP_CONFIG.dim_e) {
            MAP_CONFIG.dim_e = nextE;
            changed = true;
          }
        }
        if (data.spacing_m != null) {
          const nextSpacing = Number(data.spacing_m);
          if (nextSpacing !== MAP_CONFIG.spacing_m) {
            MAP_CONFIG.spacing_m = nextSpacing;
            changed = true;
          }
        }

        if (!changed) {
          return;
        }

        const center = map.getCenter();
        drawGrid(center.lat, center.lng);
        updateOriginReadout(center.lat, center.lng, map.getZoom());
      }

      function handleMapMessage(data) {
        if (!data || data.type !== MAP_MESSAGE_TYPE || !map) {
          return;
        }

        if (data.action === "updateGrid") {
          applyUpdateGrid(data);
          return;
        }

        if (data.action === "updateView") {
          if (data.center_lat != null) {
            MAP_CONFIG.center_lat = data.center_lat;
          }
          if (data.center_lon != null) {
            MAP_CONFIG.center_lon = data.center_lon;
          }
          if (data.zoom != null) {
            MAP_CONFIG.zoom = data.zoom;
          }
          applyView(
            MAP_CONFIG.center_lat,
            MAP_CONFIG.center_lon,
            MAP_CONFIG.zoom
          );
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

      map.on("move", onViewportChanged);
      map.on("zoom", onViewportChanged);

      drawGrid(MAP_CONFIG.center_lat, MAP_CONFIG.center_lon);
      drawUserMarker();
      updateOriginReadout(
        MAP_CONFIG.center_lat,
        MAP_CONFIG.center_lon,
        MAP_CONFIG.zoom
      );

      document
        .getElementById("apply-origin")
        .addEventListener("click", pushViewportToStreamlit);

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


def _encode_payload(payload: dict) -> str:
    return base64.b64encode(json.dumps(payload).encode("utf-8")).decode("ascii")


def post_map_message(payload: dict) -> None:
    """Send a lightweight update to the mounted map iframe without remounting it."""
    message_b64 = _encode_payload({"type": MAP_MESSAGE_TYPE, **payload})
    channel_name = json.dumps(MAP_MESSAGE_TYPE)
    components.html(
        f"""
        <div data-msg-b64="{message_b64}" id="gps-stick-msg" hidden></div>
        <script>
        (function() {{
          const msg = JSON.parse(
            atob(document.getElementById("gps-stick-msg").dataset.msgB64)
          );
          function send() {{
            try {{
              window.parent.postMessage(msg, "*");
            }} catch (error) {{}}
            try {{
              if (typeof BroadcastChannel !== "undefined") {{
                const channel = new BroadcastChannel({channel_name});
                channel.postMessage(msg);
                channel.close();
              }}
            }} catch (error) {{}}
          }}
          send();
          setTimeout(send, 75);
          setTimeout(send, 250);
        }})();
        </script>
        """,
        height=0,
    )


def render_placement_map(
    center_lat,
    center_lon,
    zoom,
    line_count_m,
    line_count_e,
    spacing,
    user_lat,
    user_lon,
    height=520,
):
    config = {
        "center_lat": center_lat,
        "center_lon": center_lon,
        "zoom": zoom,
        "dim_m": int(line_count_m),
        "dim_e": int(line_count_e),
        "spacing_m": float(spacing),
        "user_lat": user_lat,
        "user_lon": user_lon,
    }
    html = (
        _PLACEMENT_MAP_HTML.replace("__MAP_CONFIG_B64__", _encode_payload(config))
        .replace("__MAP_MESSAGE_TYPE__", json.dumps(MAP_MESSAGE_TYPE))
    )
    components.html(html, height=height, scrolling=False)