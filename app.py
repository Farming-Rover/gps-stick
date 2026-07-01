import streamlit as st
import pandas as pd
import numpy as np
import time
import math

# st.map size values are in meters (not pixels)
GRID_MARKER_SIZE_M = 1.0
TARGET_MARKER_SIZE_M = 0.2
USER_MARKER_SIZE_M = 0.75

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
        for c in range(cols):
            dn = r * spacing_meters
            de = c * spacing_meters
            point_lat = origin_lat + (dn / lat_degree_meters)
            point_lon = origin_lon + (de / lon_degree_meters)
            grid.append({
                "Point": f"R{r+1}C{c+1}",
                "lat": point_lat,
                "lon": point_lon,
                "color": "#1E90FF",
                "size": GRID_MARKER_SIZE_M,
            })
    return pd.DataFrame(grid)

# --- LIVE HARDWARE / MOCK GPS ---
def get_current_rtk_gps():
    # Production note: Swap this mock data with your raw ZED-F9P serial parsing loop later
    if "mock_lat" not in st.session_state:
        st.session_state.mock_lat = 37.774929
        st.session_state.mock_lon = -122.419416
    return st.session_state.mock_lat, st.session_state.mock_lon

# --- STREAMLIT UI LAYOUT ---
st.set_page_config(page_title="RTK Live Map Guide", layout="centered")
st.title("🎯 RTK Live Map Guide")

if "grid_df" not in st.session_state:
    st.session_state.grid_df = None

current_lat, current_lon = get_current_rtk_gps()

# Sidebar Setup for Grid Layout Configuration
with st.sidebar:
    st.header("Grid Settings")
    grid_rows = st.number_input("Rows", min_value=1, value=4)
    grid_cols = st.number_input("Columns", min_value=1, value=4)
    spacing = st.number_input("Spacing (Meters)", min_value=0.5, value=2.0, step=0.5)
    
    if st.button("Lock Current Position as Origin & Build Grid"):
        st.session_state.grid_df = generate_grid(current_lat, current_lon, grid_rows, grid_cols, spacing)
        st.success("Grid Formed and Locked!")

# Main Real-Time Interface Loop
if st.session_state.grid_df is not None:
    df = st.session_state.grid_df
    
    # Let the user choose which node point they are navigating toward
    target_point = st.selectbox("Select Target Grid Point:", df["Point"].tolist())
    target_row = df[df["Point"] == target_point].iloc[0]
    target_lat, target_lon = target_row["lat"], target_row["lon"]

    # This UI Fragment isolated block auto-runs every 1.0 seconds
    @st.fragment(run_every=1.0)
    def render_live_dashboard(t_lat, t_lon, t_name):
        c_lat, c_lon = get_current_rtk_gps()
        dist, bear = get_distance_and_bearing(c_lat, c_lon, t_lat, t_lon)
        
        st.markdown("---")
        col1, col2 = st.columns(2)
        col1.metric(label="Distance to Target", value=f"{dist:.2f} m")
        col2.metric(label="Required Heading", value=f"{bear:.1f}°")
        
        if dist < 0.20:
            st.success("🎉 TARGET REACHED (Within 20cm RTK tolerance)!")
        else:
            st.info(f"Walk towards {bear:.1f}° for {dist:.2f} meters")

        # --- DYNAMIC MAP ASSEMBLY ---
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
            zoom=19  # High-level zoom for precise centimeter field steps
        )

    # Fire the live refresh component loop
    render_live_dashboard(target_lat, target_lon, target_point)

    # Manual field walking emulator controls for immediate testing
    st.markdown("---")
    st.write("**Simulate Movement Steps:**")
    step = 0.000015 # Rough equivalent of a 1.5-meter step sizing vector
    c_btn1, c_btn2, c_btn3, c_btn4 = st.columns(4)
    if c_btn1.button("⬆️ North"): st.session_state.mock_lat += step; st.rerun()
    if c_btn2.button("⬇️ South"): st.session_state.mock_lat -= step; st.rerun()
    if c_btn3.button("⬅️ West"): st.session_state.mock_lon -= step; st.rerun()
    if c_btn4.button("➡️ East"): st.session_state.mock_lon += step; st.rerun()

else:
    st.warning("Please configure your grid constraints in the sidebar panel and set your origin marker lock.")