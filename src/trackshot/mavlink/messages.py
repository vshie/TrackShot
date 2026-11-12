"""MAVLink message builders: (type name, fields dict) pairs any `Transport` can send.

Field names are pymavlink's (mavlink2rest's JSON uses the same ones). `char[N]` fields are plain
`str` here; each transport encodes them for its wire format (`char_fields()` says which they are).
"""

from __future__ import annotations

from functools import lru_cache

from pymavlink.dialects.v20 import ardupilotmega

from trackshot.core.ned import NedEstimate
from trackshot.core.odometry import DeltaMessage, OdometryHealth

NAMED_VALUE_NAME_MAX = 10   # NAMED_VALUE_FLOAT.name is char[10]

# Cockpit shows these as NAMED_VALUE_FLOAT; tracking as a number so it can be graphed/alerted on.
# Below 0.5, TrackShot is not sending odometry (acquiring, lost or not started).
TRACKING_VALUES = {"good": 1.0, "degraded": 0.5, "acquiring": 0.25, "lost": 0.0, "unknown": 0.0}


def vision_position_delta(d: DeltaMessage) -> tuple[str, dict]:
    return "VISION_POSITION_DELTA", {
        "time_usec": int(d.time_usec),
        "time_delta_usec": int(d.time_delta_usec),
        "angle_delta": [float(v) for v in d.angle_delta],
        "position_delta": [float(v) for v in d.position_delta],
        "confidence": float(d.confidence),
    }


def vision_position_estimate(e: NedEstimate) -> tuple[str, dict]:
    """Horizontal position in the EKF's NED frame (see trackshot.core.ned). Zero covariance makes
    ArduSub use VISO_POS_M_NSE; MAVLink's NaN "unknown" can't travel in mavlink2rest's JSON."""
    return "VISION_POSITION_ESTIMATE", {
        "usec": int(e.usec),
        "x": float(e.position[0]),
        "y": float(e.position[1]),
        "z": float(e.position[2]),
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": float(e.yaw),
        "covariance": [0.0] * 21,
        "reset_counter": 0,
    }


def vision_speed_estimate(e: NedEstimate) -> tuple[str, dict]:
    """NED velocity; down is 0 and unused (EK3_SRC1_VELZ 0). ArduSub uses VISO_VEL_M_NSE."""
    return "VISION_SPEED_ESTIMATE", {
        "usec": int(e.usec),
        "x": float(e.velocity[0]),
        "y": float(e.velocity[1]),
        "z": float(e.velocity[2]),
        "covariance": [0.0] * 9,
        "reset_counter": 0,
    }


def named_value_float(name: str, value: float, time_boot_ms: int) -> tuple[str, dict]:
    if not name or len(name) > NAMED_VALUE_NAME_MAX or not name.isascii():
        raise ValueError(f"NAMED_VALUE_FLOAT name must be 1–{NAMED_VALUE_NAME_MAX} ASCII characters: {name!r}")
    return "NAMED_VALUE_FLOAT", {
        "time_boot_ms": int(time_boot_ms) & 0xFFFFFFFF,   # uint32: wraps after ~49 days
        "name": name,
        "value": float(value),
    }


def health_values(h: OdometryHealth, time_boot_ms: int) -> list[tuple[str, dict]]:
    """TS_* values for Cockpit. TS_LAT_MS is left out until a latency has been measured."""
    values = [named_value_float("TS_FPS", h.fps, time_boot_ms)]
    if h.latency_ms is not None:
        values.append(named_value_float("TS_LAT_MS", h.latency_ms, time_boot_ms))
    values.append(named_value_float("TS_JUMPS", h.jumps, time_boot_ms))
    values.append(named_value_float("TS_TRACK", TRACKING_VALUES.get(h.tracking, 0.0), time_boot_ms))
    return values


@lru_cache(maxsize=None)
def char_fields(msg_type: str) -> dict[str, int]:
    """{field: N} for the message's char[N] fields, from pymavlink's ardupilotmega definition."""
    cls = getattr(ardupilotmega, f"MAVLink_{msg_type.lower()}_message")
    lengths = dict(zip(cls.ordered_fieldnames, cls.array_lengths))
    return {name: lengths[name] for name, kind in zip(cls.fieldnames, cls.fieldtypes) if kind == "char"}


def encode(mav, msg_type: str, fields: dict):
    """pymavlink MAVLink_message via `mav.<type>_encode(**fields)`; char fields go in as ASCII bytes."""
    chars = char_fields(msg_type)
    kwargs = {k: (v.encode("ascii") if k in chars and isinstance(v, str) else v) for k, v in fields.items()}
    return getattr(mav, f"{msg_type.lower()}_encode")(**kwargs)
