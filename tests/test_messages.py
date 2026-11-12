"""MAVLink builders, verified by encoding with pymavlink and decoding the bytes back."""

import pytest
from pymavlink.dialects.v20 import ardupilotmega

from trackshot.core.odometry import DeltaMessage, OdometryHealth
from trackshot.mavlink.messages import (
    char_fields,
    encode,
    health_values,
    named_value_float,
    vision_position_delta,
)


def roundtrip(msg_type, fields):
    """Encode as TrackShot would send it (sys 1, comp 197), then parse the wire bytes."""
    mav = ardupilotmega.MAVLink(None, srcSystem=1, srcComponent=197)
    wire = encode(mav, msg_type, fields).pack(mav)
    assert wire[0] == 0xFD   # MAVLink 2 frame
    decoded = ardupilotmega.MAVLink(None).parse_buffer(wire)
    assert len(decoded) == 1
    msg = decoded[0]
    assert msg.get_type() == msg_type
    assert (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 197)
    return msg


def health(**overrides):
    values = dict(fps=9.5, latency_ms=62.4, suggested_viso_delay_ms=60, poses=100, deltas=99, jumps=2,
                  rejected_spans=0, last_pose=None, last_pose_age_s=0.1, stalled=False, tracking="good")
    values.update(overrides)
    return OdometryHealth(**values)


def test_vision_position_delta_roundtrip():
    d = DeltaMessage(time_usec=1_700_000_000_123_456, time_delta_usec=100_000,
                     position_delta=(0.1, -0.02, 0.003), angle_delta=(0.0, 0.01, -0.17), confidence=100.0)
    msg_type, fields = vision_position_delta(d)
    assert msg_type == "VISION_POSITION_DELTA"
    assert set(fields) == {"time_usec", "time_delta_usec", "angle_delta", "position_delta", "confidence"}
    msg = roundtrip(msg_type, fields)
    assert msg.time_usec == 1_700_000_000_123_456
    assert msg.time_delta_usec == 100_000
    assert msg.position_delta == pytest.approx([0.1, -0.02, 0.003], rel=1e-6)   # float32 on the wire
    assert msg.angle_delta == pytest.approx([0.0, 0.01, -0.17], rel=1e-6)
    assert msg.confidence == 100.0


def test_vision_position_delta_fields_are_json_types():
    # mavlink2rest gets these through json.dumps: plain lists, ints and floats only
    _, fields = vision_position_delta(DeltaMessage(1, 2, (0, 0, 0), (0, 0, 0), 0))
    assert isinstance(fields["position_delta"], list) and isinstance(fields["angle_delta"], list)
    assert all(isinstance(v, float) for v in fields["position_delta"] + fields["angle_delta"])


def test_named_value_float_roundtrip():
    msg_type, fields = named_value_float("TS_LAT_MS", 62.5, 123_456)
    assert fields == {"time_boot_ms": 123_456, "name": "TS_LAT_MS", "value": 62.5}
    msg = roundtrip(msg_type, fields)
    assert (msg.name, msg.value, msg.time_boot_ms) == ("TS_LAT_MS", 62.5, 123_456)


def test_named_value_float_ten_char_name_and_wrap():
    msg = roundtrip(*named_value_float("ABCDEFGHIJ", 1.0, 2**32 + 5))
    assert msg.name == "ABCDEFGHIJ"
    assert msg.time_boot_ms == 5   # uint32 wraps


@pytest.mark.parametrize("name", ["", "ELEVENCHARS", "TS_FPS_é"])
def test_named_value_float_rejects_bad_names(name):
    with pytest.raises(ValueError):
        named_value_float(name, 1.0, 0)


def test_health_values():
    values = dict((f["name"], f["value"]) for _, f in health_values(health(), 1000))
    assert values == {"TS_FPS": 9.5, "TS_LAT_MS": 62.4, "TS_JUMPS": 2.0, "TS_TRACK": 1.0}
    for msg_type, fields in health_values(health(), 1000):
        assert roundtrip(msg_type, fields).time_boot_ms == 1000


@pytest.mark.parametrize("tracking,value", [("good", 1.0), ("degraded", 0.5), ("lost", 0.0), ("unknown", 0.0)])
def test_health_tracking_value(tracking, value):
    values = dict((f["name"], f["value"]) for _, f in health_values(health(tracking=tracking), 0))
    assert values["TS_TRACK"] == value


def test_health_without_latency_leaves_it_out():
    names = [f["name"] for _, f in health_values(health(latency_ms=None), 0)]
    assert names == ["TS_FPS", "TS_JUMPS", "TS_TRACK"]


def test_char_fields():
    assert char_fields("NAMED_VALUE_FLOAT") == {"name": 10}
    assert char_fields("PARAM_SET") == {"param_id": 16}
    assert char_fields("VISION_POSITION_DELTA") == {}
