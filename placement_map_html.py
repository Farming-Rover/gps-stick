import base64
import json
import time
from pathlib import Path

import streamlit as st
import streamlit.components.v1 as components

MAP_MESSAGE_TYPE = "gps-stick-map"

# The placement map is a real bidirectional custom component (it must report
# the panned grid origin back to Python). Its frontend lives in
# placement_map_component/index.html.
_placement_map_component = components.declare_component(
    "placement_map",
    path=str(Path(__file__).parent / "placement_map_component"),
)


def _call_component_json_only(component, *, default=None, key=None, **kwargs):
    """Invoke a custom component without Streamlit's PyArrow import guard.

    CustomComponent.create_instance refuses to run when PyArrow is missing,
    but PyArrow is only actually used to marshal dataframe args and unmarshal
    Arrow-table return values. This component exchanges plain JSON scalars,
    and the Raspberry Pi Zero (32-bit ARMv6) can't install PyArrow, so this
    replicates create_instance's JSON-only path (verified against Streamlit
    1.58.0, which must be the version on the Pi).

    Unlike the stock helper (which always uses main_dg), this enqueues into
    the active DeltaGenerator so a component inside an @st.fragment only
    triggers a fragment rerun — not a full script rerun — when its value
    changes. Full reruns of this app are too slow on a Pi Zero for that.
    """
    from streamlit.delta_generator_singletons import get_dg_singleton_instance
    from streamlit.elements.lib.form_utils import current_form_id
    from streamlit.elements.lib.utils import compute_and_register_element_id
    from streamlit.proto.Element_pb2 import Element
    from streamlit.runtime.scriptrunner_utils.script_run_context import (
        get_script_run_ctx,
    )
    from streamlit.runtime.state import register_widget

    serialized_json_args = json.dumps(dict(kwargs, default=default, key=key))

    # Prefer the fragment/container DG on the context stack when present.
    dg = get_dg_singleton_instance().main_dg._active_dg
    element = Element()
    instance = element.component_instance
    instance.component_name = component.name
    instance.form_id = current_form_id(dg)
    if component.url is not None:
        instance.url = component.url
    instance.json_args = serialized_json_args

    instance.id = compute_and_register_element_id(
        "component_instance",
        user_key=key,
        key_as_main_identity={"name", "url"},
        dg=dg,
        name=component.name,
        url=component.url,
        json_args=serialized_json_args,
        special_args=[],
    )

    component_state = register_widget(
        instance.id,
        deserializer=lambda ui_value: ui_value,
        serializer=lambda x: x,
        ctx=get_script_run_ctx(),
        value_type="json_value",
    )
    widget_value = component_state.value
    if widget_value is None:
        widget_value = default

    dg._enqueue("component_instance", instance)
    return widget_value


def render_html_embed(html: str, height: int) -> None:
    """Render raw HTML in an iframe via st.iframe (components.v1.html is deprecated).

    Falls back to components.html on Streamlit versions without st.iframe.
    """
    if hasattr(st, "iframe"):
        # st.iframe rejects 0; a 1 px iframe is invisible for the hidden messenger.
        st.iframe(html, height=max(int(height), 1))
    else:
        components.html(html, height=height, scrolling=False)


def _encode_payload(payload: dict) -> str:
    return base64.b64encode(json.dumps(payload).encode("utf-8")).decode("ascii")


def post_map_message(payload: dict) -> None:
    """Send a lightweight update to the mounted map iframe without remounting it."""
    # The nonce makes each messenger render unique. Without it, Streamlit sees
    # identical iframe HTML on repeated sends (e.g. pressing the same button
    # twice), skips remounting it, and the message never fires. The map-side
    # handlers ignore the extra key.
    message_b64 = _encode_payload(
        {"type": MAP_MESSAGE_TYPE, "_nonce": time.time_ns(), **payload}
    )
    channel_name = json.dumps(MAP_MESSAGE_TYPE)
    render_html_embed(
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
          setTimeout(send, 30);
          setTimeout(send, 120);
          setTimeout(send, 300);
        }})();
        </script>
        """,
        height=1,
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
    view_seq=0,
    bearing=0,
):
    """Render the grid placement map and return the current grid origin.

    Returns None until the user pans/zooms/rotates, then dicts like
    {"lat": ..., "lon": ..., "zoom": ..., "bearing": ..., "seq": ...}. The map
    only snaps its view to (center_lat, center_lon, bearing) when view_seq
    changes; otherwise reruns leave the user's current pan/zoom/rotate alone.
    """
    return _call_component_json_only(
        _placement_map_component,
        center_lat=float(center_lat),
        center_lon=float(center_lon),
        zoom=int(zoom),
        dim_m=int(line_count_m),
        dim_e=int(line_count_e),
        spacing_m=float(spacing),
        user_lat=float(user_lat),
        user_lon=float(user_lon),
        height=int(height),
        view_seq=int(view_seq),
        bearing=float(bearing) % 360.0,
        key="placement_map",
        default=None,
    )