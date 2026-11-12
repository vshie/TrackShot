"""read_autopilot_telemetry and the VehicleTelemetry poller against the fake mavlink2rest."""

import socket
import time

import pytest

from trackshot.autopilot.telemetry import (TELEMETRY_KEYS, VehicleTelemetry, ardusub_mode_name,
                                           is_armed_heartbeat, read_autopilot_telemetry)
from trackshot.testing.fake_autopilot import FakeAutopilot, VehicleState

SNAPSHOT_KEYS = set(TELEMETRY_KEYS) | {"connected", "armed", "mode", "heartbeat_age_s", "age_s"}


@pytest.fixture(params=["0.11", "1.1"])
def fake(request):
    with FakeAutopilot(flavor=request.param) as fake:
        yield fake


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_read_autopilot_telemetry(fake):
    t = read_autopilot_telemetry(fake.http_base)
    v = fake.vehicle
    assert list(t) == ["lat", "lon", "heading_deg", "roll_deg", "pitch_deg", "depth_m",
                       "temperature_c", "altitude_m", "mission_seq"]
    assert t["lat"] == pytest.approx(v.lat) and t["lon"] == pytest.approx(v.lon)
    assert t["heading_deg"] == pytest.approx(v.heading_deg)
    assert t["roll_deg"] == pytest.approx(v.roll_deg) and t["pitch_deg"] == pytest.approx(v.pitch_deg)
    assert t["depth_m"] == pytest.approx(v.depth_m)
    assert t["temperature_c"] == pytest.approx(v.temperature_c)
    assert t["altitude_m"] == pytest.approx(v.altitude_m)
    assert t["mission_seq"] == 0


def test_no_origin_and_no_rangefinder():
    with FakeAutopilot(vehicle=VehicleState(lat=0.0, lon=0.0, altitude_m=None)) as fake:
        t = read_autopilot_telemetry(fake.http_base)
    assert t["lat"] is None and t["lon"] is None
    assert t["altitude_m"] is None
    assert t["depth_m"] == pytest.approx(fake.vehicle.depth_m)


def test_unreachable_gives_all_none():
    t = read_autopilot_telemetry(f"http://127.0.0.1:{free_port()}/mavlink2rest")
    assert t == {key: None for key in TELEMETRY_KEYS}


def test_skewed_clock_old_server():
    # A Pi clock an hour behind makes every 0.11.x timestamp look stale unless the skew is known
    with FakeAutopilot(flavor="0.11", clock_skew_s=-3600.0) as fake:
        assert read_autopilot_telemetry(fake.http_base)["depth_m"] is None
        assert read_autopilot_telemetry(fake.http_base, clock_skew_s=3600.0)["depth_m"] == pytest.approx(4.2)

        poller = VehicleTelemetry(fake.http_base)
        poller.poll_once()
        fake.set_armed(False)                 # publishes a HEARTBEAT, so its counter moves
        snapshot = poller.poll_once()
    assert snapshot["connected"]
    assert 0.0 <= snapshot["heartbeat_age_s"] < 1.5
    assert snapshot["depth_m"] == pytest.approx(4.2)


def test_skewed_clock_new_server():
    # mavlink2rest 1.1 reports the age itself
    with FakeAutopilot(flavor="1.1", clock_skew_s=-3600.0) as fake:
        assert read_autopilot_telemetry(fake.http_base)["depth_m"] == pytest.approx(4.2)
        snapshot = VehicleTelemetry(fake.http_base).poll_once()
    assert snapshot["connected"] and snapshot["heartbeat_age_s"] < 1.5


def test_poller_snapshot(fake):
    poller = VehicleTelemetry(fake.http_base)
    before = poller.latest()
    assert set(before) == SNAPSHOT_KEYS
    assert before["connected"] is False and before["age_s"] is None

    poller.poll_once()
    snapshot = poller.latest()
    assert set(snapshot) == SNAPSHOT_KEYS
    assert snapshot["connected"] is True
    assert snapshot["armed"] is False
    assert snapshot["mode"] == "MANUAL"
    assert 0.0 <= snapshot["heartbeat_age_s"] < 1.5
    assert 0.0 <= snapshot["age_s"] < 1.0
    assert snapshot["depth_m"] == pytest.approx(fake.vehicle.depth_m)

    fake.update_vehicle(armed=True, custom_mode=2)
    poller.poll_once()
    snapshot = poller.latest()
    assert snapshot["armed"] is True and snapshot["mode"] == "ALT_HOLD"


def test_poller_rides_out_a_stalled_request(monkeypatch):
    # Measured on BlueOS SITL: mavlink2rest stalls 1–1.7 s every ~40 s. One timed-out request
    # must not report the vehicle lost; a HEARTBEAT that really stops updating still must.
    import trackshot.autopilot.telemetry as telemetry
    with FakeAutopilot(flavor="0.11") as fake:
        poller = VehicleTelemetry(fake.http_base)
        assert poller.poll_once()["connected"]

        real_get_messages = telemetry.get_messages

        def stalled(base, names, timeout=0.5):
            return {name: None for name in names}
        monkeypatch.setattr(telemetry, "get_messages", stalled)
        snapshot = poller.poll_once()
        assert snapshot["connected"], "one failed request reported the vehicle lost"
        assert snapshot["depth_m"] == pytest.approx(fake.vehicle.depth_m)

        # The cached reply keeps its server stamp, so it goes stale like a real silent vehicle:
        # backdate it past STALE_AFTER_S and the poller must report the vehicle lost
        from datetime import datetime, timedelta, timezone
        old = datetime.now(timezone.utc) - timedelta(seconds=telemetry.STALE_AFTER_S + 2.0)
        poller._last_raw["HEARTBEAT"]["status"]["time"]["last_update"] = old.isoformat()
        assert poller.poll_once()["connected"] is False

        monkeypatch.setattr(telemetry, "get_messages", real_get_messages)
        fake.set_armed(False)
        assert poller.poll_once()["connected"]


def test_poller_without_heartbeat():
    with FakeAutopilot(heartbeat=False) as fake:
        snapshot = VehicleTelemetry(fake.http_base).poll_once()
    assert snapshot["connected"] is False
    assert snapshot["armed"] is None and snapshot["mode"] is None and snapshot["heartbeat_age_s"] is None
    assert snapshot["depth_m"] is not None


def test_poller_thread_start_stop(fake):
    poller = VehicleTelemetry(fake.http_base, poll_hz=20.0)
    poller.start()
    try:
        deadline = time.monotonic() + 2.0
        while not poller.latest()["connected"] and time.monotonic() < deadline:
            time.sleep(0.02)
        assert poller.latest()["connected"]
        assert poller._thread.daemon
    finally:
        started = time.monotonic()
        poller.stop()
    assert time.monotonic() - started < 1.0
    assert not poller.is_running()


def test_poller_unreachable_never_blocks_latest():
    poller = VehicleTelemetry(f"http://127.0.0.1:{free_port()}/mavlink2rest", poll_hz=10.0)
    poller.start()
    try:
        started = time.monotonic()
        for _ in range(50):
            snapshot = poller.latest()
        assert time.monotonic() - started < 0.1
        assert snapshot["connected"] is False
    finally:
        poller.stop()
    assert not poller.is_running()


@pytest.mark.parametrize("base_mode, armed", [
    ({"bits": 209}, True), ({"bits": 81}, False), (209, True), (81, False),
    ("MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED", True),
    ("MAV_MODE_FLAG_CUSTOM_MODE_ENABLED", False), ("", False),
])
def test_is_armed_heartbeat_formats(base_mode, armed):
    assert is_armed_heartbeat({"base_mode": base_mode}) is armed


def test_ardusub_mode_name():
    assert ardusub_mode_name(19) == "MANUAL"
    assert ardusub_mode_name(0) == "STABILIZE"
    assert ardusub_mode_name(42) == "MODE_42"
    assert ardusub_mode_name(None) is None
