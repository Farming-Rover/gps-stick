"""Process-wide NTRIP client state.

Kept in a non-main module so Streamlit script reruns do not re-bind these
names back to idle/None (which made the sidebar forget an active stream).
Background threads can also read/write this without calling Streamlit APIs.
"""

from __future__ import annotations

import threading

# How long after the last RTCM byte before the UI stops calling the link
# "streaming".
STREAM_STALE_S = 12.0

lock = threading.Lock()
thread = None
stop = threading.Event()
# Desired connection target, or None when idle:
# {
#   "source": "rtk2go" | "local",
#   "host": str,
#   "port": int,
#   "mountpoint": str,
#   "username": str,
#   "password": str,
# }
desired = None
status = {
    "state": "idle",  # idle | connecting | connected | error
    "mountpoint": None,
    "host": None,
    "port": None,
    "source": None,
    "message": "Not connected",
    "last_rtcm_at": 0.0,
    "bytes_total": 0,
}


def target_key(target):
    """Stable identity string for a caster target (used by UI switch prompts)."""
    if not target:
        return None
    host = target.get("host") or ""
    port = int(target.get("port") or 0)
    mount = (target.get("mountpoint") or "").strip().lstrip("/")
    return f"{host}:{port}/{mount}"


def target_label(target):
    """Short human-readable label for status banners."""
    key = target_key(target)
    return key or "caster"
