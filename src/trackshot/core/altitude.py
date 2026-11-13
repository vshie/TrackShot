"""Altitude above the seabed from the OAK's stereo depth, for DISTANCE_SENSOR (a downward rangefinder).

With the camera looking straight down, the depth image is the distance to the seabed along the
camera's optical axis. The estimate is the median of the valid depth in the middle of the image:
robust to fish, edges and the sparse holes stereo leaves on featureless sand. ArduSub corrects for
the vehicle's tilt itself (ArduSub/sensors.cpp: × cos(tilt)), so the raw range along the axis is
what goes out.

DISTANCE_SENSOR.signal_quality: ArduSub treats a reading as healthy when the quality is 0
("unknown") or at least RNGFND_SQ_MIN (90 on the test vehicle). The quality here is the fraction of
valid pixels in the window, scaled so that half of them valid reads 100: a mostly empty depth
image falls below 90 and SURFTRAK stops trusting it, rather than following a guess.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

# Middle of the image the estimate uses (fraction of width and height)
WINDOW_FRACTION = 0.4
# Ranges the OAK's stereo depth reports reliably at TrackShot's resolutions, m
MIN_RANGE_M = 0.3
MAX_RANGE_M = 10.0
# Below this fraction of valid pixels in the window there's no estimate at all
MIN_VALID_FRACTION = 0.15
# This fraction of valid pixels or more is quality 100
FULL_QUALITY_FRACTION = 0.5


@dataclass(frozen=True)
class RangeEstimate:
    t: float                         # wall-clock s of the depth frame
    distance_m: Optional[float]      # None: no usable depth in the window
    quality: int                     # 1–100 (MAVLink: 1 = worst; never 0, which means "unknown")
    valid_fraction: float


def range_from_depth(depth_mm: np.ndarray, t: float) -> RangeEstimate:
    """Median valid depth (uint16 mm, 0 = no depth) in the middle of the image."""
    h, w = depth_mm.shape[:2]
    dh, dw = max(1, int(h * WINDOW_FRACTION / 2)), max(1, int(w * WINDOW_FRACTION / 2))
    window = depth_mm[h // 2 - dh:h // 2 + dh, w // 2 - dw:w // 2 + dw].astype(np.float32) / 1000.0
    valid = window[(window >= MIN_RANGE_M) & (window <= MAX_RANGE_M)]
    fraction = valid.size / window.size if window.size else 0.0
    if fraction < MIN_VALID_FRACTION:
        return RangeEstimate(t, None, 1, fraction)
    quality = int(round(100 * min(1.0, fraction / FULL_QUALITY_FRACTION)))
    return RangeEstimate(t, float(np.median(valid)), max(1, quality), fraction)
