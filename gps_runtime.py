"""Process-wide GNSS fix quality from the latest GGA sentence.

Lives outside the Streamlit main script so all browser sessions share the
same RTK quality readout from the single receiver on the Pi.
"""

from __future__ import annotations

import threading

lock = threading.Lock()
gps_qual = None  # int GGA quality indicator, or None
num_sats = None
updated_at = 0.0
