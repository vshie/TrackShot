"""AutopilotParams against the fake mavlink2rest (both JSON flavours)."""

import socket
import sys
import time

import pytest

from trackshot.autopilot.params import (GCS_COMPONENT_ID, GCS_SYSTEM_ID, VISUAL_ODOMETRY_PARAMS,
                                        AutopilotParams, AutopilotUnreachable, NoHeartbeat,
                                        VehicleArmed, suggest_viso_delay_ms)
from trackshot.testing.fake_autopilot import FakeAutopilot


@pytest.fixture(params=["0.11", "1.1"])
def fake(request):
    with FakeAutopilot(flavor=request.param) as fake:
        yield fake


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.mark.parametrize("latency, expected", [
    (None, None), (float("nan"), None), (0.0, 0), (4.9, 0), (5.0, 10), (54.0, 50), (55.0, 60),
    (65.0, 70), (-30.0, 0), (249.0, 250), (1000.0, 250),
])
def test_suggest_viso_delay_ms(latency, expected):
    assert suggest_viso_delay_ms(latency) == expected


def test_check_apply_round_trip(fake):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    rows = autopilot.check()
    assert [r["name"] for r in rows] == [name for name, _, _ in VISUAL_ODOMETRY_PARAMS]
    by_name = {r["name"]: r for r in rows}
    assert set(by_name["VISO_TYPE"]) >= {"name", "current", "recommended", "needs_reboot", "ok"}
    assert by_name["VISO_TYPE"] == {"name": "VISO_TYPE", "current": 0.0, "recommended": 1,
                                    "needs_reboot": True, "ok": False,
                                    "param_type": "MAV_PARAM_TYPE_INT8"}
    assert by_name["AHRS_EKF_TYPE"]["ok"] and by_name["EK3_SRC1_POSZ"]["ok"]
    assert not by_name["EK3_SRC1_POSXY"]["ok"]

    results = autopilot.apply(rows)
    assert [r["name"] for r in results] == [r["name"] for r in rows if not r["ok"]]
    assert all(r["ok"] and r["error"] is None for r in results)
    assert {r["name"] for r in results if r["needs_reboot"]} == {"VISO_TYPE"}
    for name, value, _ in VISUAL_ODOMETRY_PARAMS:
        assert fake.params[name] == value

    assert all(r["ok"] for r in autopilot.check())
    # PARAM_SETs went out as a ground station, in the flavour's char-array form
    sets = fake.received("PARAM_SET")
    assert sets and all(m["_header"] == {"system_id": GCS_SYSTEM_ID, "component_id": GCS_COMPONENT_ID}
                        for m in sets)
    raw_id = sets[0]["_raw"]["message"]["param_id"]
    assert isinstance(raw_id, list if fake.flavor == "0.11" else str)


def test_viso_delay_row_only_when_suggested(fake):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    assert "VISO_DELAY_MS" not in {r["name"] for r in autopilot.check()}

    fake.params["VISO_DELAY_MS"] = 10.0
    row = autopilot.check(suggested_viso_delay_ms=20)[-1]
    assert row["name"] == "VISO_DELAY_MS" and row["ok"]          # within 10 ms
    assert row["needs_reboot"] is False
    row = autopilot.check(suggested_viso_delay_ms=30)[-1]
    assert row["recommended"] == 30 and not row["ok"]

    results = autopilot.apply([row])
    assert results == [{"name": "VISO_DELAY_MS", "ok": True, "needs_reboot": False, "error": None}]
    assert fake.params["VISO_DELAY_MS"] == 30.0


def test_apply_refused_while_armed(fake):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    rows = autopilot.check()
    fake.set_armed(True)
    assert autopilot.is_armed()
    with pytest.raises(VehicleArmed):
        autopilot.apply(rows)
    assert fake.params["VISO_TYPE"] == 0.0
    assert fake.received("PARAM_SET") == []


def test_unreachable_is_reported_quickly():
    autopilot = AutopilotParams(f"http://127.0.0.1:{free_port()}/mavlink2rest", timeout=3.0)
    started = time.monotonic()
    with pytest.raises(AutopilotUnreachable):
        autopilot.check()
    # Windows retries a refused TCP connect for about 2 s before failing it
    assert time.monotonic() - started < (3.0 if sys.platform == "win32" else 1.0)


def test_no_heartbeat():
    # mavlink2rest answers HTTP 200 "None" for a message it hasn't seen
    with FakeAutopilot(heartbeat=False) as fake:
        with pytest.raises(NoHeartbeat):
            AutopilotParams(fake.http_base, timeout=1.0).check()


def test_missing_parameter(fake):
    del fake.params["VISO_DELAY_MS"]     # ArduPilot hides VISO_* until VISO_TYPE is set and rebooted
    autopilot = AutopilotParams(fake.http_base, timeout=0.2)
    row = autopilot.check(suggested_viso_delay_ms=50)[-1]
    assert row["current"] is None and row["param_type"] is None and not row["ok"]
    [result] = autopilot.apply([row])
    assert not result["ok"] and "VISO_TYPE" in result["error"]


def test_read_and_write(fake):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    assert autopilot.read("SURFACE_DEPTH") == -10.0
    assert autopilot.write("SURFACE_DEPTH", -5.5, "MAV_PARAM_TYPE_REAL32") == -5.5
    assert fake.params["SURFACE_DEPTH"] == -5.5
    # ArduPilot ignores requests for parameters it doesn't have
    assert AutopilotParams(fake.http_base, timeout=0.2).read("NO_SUCH_PARAM") is None


def test_unconfirmed_write(fake):
    autopilot = AutopilotParams(fake.http_base, timeout=0.2)
    rows = [{"name": "VISO_TYPE", "current": 0.0, "recommended": 1, "needs_reboot": True,
             "ok": False, "param_type": "MAV_PARAM_TYPE_INT8"}]
    del fake.params["VISO_TYPE"]          # the autopilot never answers
    [result] = autopilot.apply(rows)
    assert result == {"name": "VISO_TYPE", "ok": False, "needs_reboot": False,
                      "error": "write not confirmed"}


def test_get_origin(fake):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    lat, lon = autopilot.get_origin()
    assert (lat, lon) == pytest.approx(fake.vehicle.origin)
    command = fake.received("COMMAND_LONG")[-1]
    assert command["command"] == {"type": "MAV_CMD_REQUEST_MESSAGE"} and command["param1"] == 49.0


def test_get_origin_none():
    with FakeAutopilot() as fake:
        fake.vehicle.origin = None
        assert AutopilotParams(fake.http_base, timeout=1.0).get_origin(request_timeout=0.3) is None


def test_a_check_against_a_silent_autopilot_can_be_cancelled():
    """An autopilot that never answers PARAM_REQUEST_READ makes check() take a minute (7 params × 3
    attempts × the timeout). Closing the app cancels it within one poll, not one HTTP timeout."""
    import threading

    from trackshot.autopilot.params import Cancelled
    with FakeAutopilot(params={}) as silent:
        autopilot = AutopilotParams(silent.http_base, timeout=3.0)
        outcome = {}

        def run():
            try:
                autopilot.check()
            except Cancelled as e:
                outcome["cancelled"] = e
        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        time.sleep(0.5)
        t0 = time.monotonic()
        autopilot.cancel.set()
        thread.join(5.0)
        assert not thread.is_alive() and "cancelled" in outcome
        assert time.monotonic() - t0 < 1.0


def test_cancel_never_cuts_an_apply_short(fake):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    rows = autopilot.check()
    autopilot.cancel.set()
    results = autopilot.apply(rows)
    assert results and all(r["ok"] for r in results)


# --- Vehicle identity, parameter backup and revert -------------------------------------------------

from trackshot.autopilot.backup import BackupError, ParamBackup  # noqa: E402
from trackshot.autopilot.params import AUTOPILOT_VERSION_MSG_ID  # noqa: E402


def param_sets(fake) -> list[tuple[str, float]]:
    return [(m["param_id"], m["param_value"]) for m in fake.received("PARAM_SET")]


def test_vehicle_key_requests_autopilot_version(fake):
    # Like the SITL: mavlink2rest has no AUTOPILOT_VERSION until it is requested
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    key, description = autopilot.vehicle_identity()
    assert key == f"ap-{fake.uid:016x}" and "uid" in description
    command = fake.received("COMMAND_LONG")[-1]
    assert command["command"] == {"type": "MAV_CMD_REQUEST_MESSAGE"}
    assert command["param1"] == float(AUTOPILOT_VERSION_MSG_ID)
    # Now cached: no second request
    assert autopilot.vehicle_key() == key
    assert len(fake.received("COMMAND_LONG")) == 1


def test_vehicle_key_prefers_uid2_when_cached():
    with FakeAutopilot(uid=0, uid2=bytes.fromhex("3a0025000851323238383136"),
                       autopilot_version_cached=True) as fake:
        assert AutopilotParams(fake.http_base).vehicle_key() == "ap-3a0025000851323238383136"
        assert fake.received("COMMAND_LONG") == []


def test_vehicle_key_falls_back_to_the_address():
    with FakeAutopilot(uid=0) as fake:     # answers, but has no hardware id
        assert AutopilotParams(fake.http_base).vehicle_key() == f"host-127.0.0.1_{fake.port}"
    with FakeAutopilot(params={}) as fake:
        fake._act_on = lambda message: None   # never answers the request
        autopilot = AutopilotParams(fake.http_base)
        started = time.monotonic()
        assert autopilot.vehicle_key(request_timeout=0.3) == f"host-127.0.0.1_{fake.port}"
        assert time.monotonic() - started < 2.0


def test_vehicle_key_unreachable():
    with pytest.raises(AutopilotUnreachable):
        AutopilotParams(f"http://127.0.0.1:{free_port()}/mavlink2rest", timeout=0.5).vehicle_key()


class SpyBackup(ParamBackup):
    """Records what the autopilot held at the moment each original was saved."""

    def __init__(self, fake, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fake = fake
        self.at_record: dict[str, float] = {}

    def record_original(self, name, original, param_type, needs_reboot):
        self.at_record[name] = self.fake.params.get(name)
        return super().record_original(name, original, param_type, needs_reboot)


def test_apply_records_originals_before_writing_and_only_once(fake, tmp_path):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    backup = SpyBackup(fake, autopilot.vehicle_key(), tmp_path)
    rows = autopilot.check(suggested_viso_delay_ms=120)
    results = autopilot.apply(rows, backup=backup)
    assert all(r["ok"] for r in results)
    changed = {"VISO_TYPE": 0.0, "EK3_SRC1_POSXY": 3.0, "EK3_SRC1_VELXY": 3.0, "EK3_SRC_OPTIONS": 1.0,
               "VISO_DELAY_MS": 10.0}
    # Saved while the autopilot still held the old value
    assert backup.at_record == changed
    entries = {e["name"]: e for e in ParamBackup(backup.vehicle_key, tmp_path).entries()}
    assert {n: e["original"] for n, e in entries.items()} == changed
    assert entries["VISO_TYPE"]["written"] == 1.0 and entries["VISO_TYPE"]["needs_reboot"]
    assert all(e["status"] == "written" for e in entries.values())

    # A later apply (a new VISO_DELAY_MS suggestion) keeps the pre-TrackShot value
    row = autopilot.check(suggested_viso_delay_ms=200)[-1]
    assert autopilot.apply([row], backup=backup)[0]["ok"]
    entry = ParamBackup(backup.vehicle_key, tmp_path).get("VISO_DELAY_MS")
    assert entry["original"] == 10.0 and entry["written"] == 200.0


def test_apply_without_a_working_backup_writes_nothing(fake, tmp_path, monkeypatch):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    backup = ParamBackup("ap-x", tmp_path)

    def fail(*_args, **_kwargs):
        raise BackupError("disk full")
    monkeypatch.setattr(backup, "record_original", fail)
    results = autopilot.apply(autopilot.check(), backup=backup)
    assert results and not any(r["ok"] for r in results)
    assert all("back up" in r["error"] for r in results)
    assert fake.received("PARAM_SET") == []


def test_failed_write_drops_its_new_original_only_if_nothing_changed(fake, tmp_path, monkeypatch):
    autopilot = AutopilotParams(fake.http_base, timeout=0.3)
    backup = ParamBackup("ap-x", tmp_path)
    rows = [r for r in autopilot.check() if r["name"] in ("VISO_TYPE", "EK3_SRC1_POSXY")]

    def lost_reply(name, value, param_type):
        if name == "EK3_SRC1_POSXY":       # the write lands but its confirmation is lost
            fake.params[name] = float(value)
        return None
    monkeypatch.setattr(autopilot, "write", lost_reply)
    results = autopilot.apply(rows, backup=backup)
    assert [r["error"] for r in results] == ["write not confirmed"] * 2
    # VISO_TYPE is still 0: nothing to revert. EK3_SRC1_POSXY changed: its original is kept.
    assert "VISO_TYPE" not in backup
    assert backup.get("EK3_SRC1_POSXY")["original"] == 3.0
    assert backup.get("EK3_SRC1_POSXY")["status"] == "pending"


def test_revert_restores_in_reverse_order_and_empties_the_backup(fake, tmp_path):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    backup = ParamBackup(autopilot.vehicle_key(), tmp_path)
    before = dict(fake.params)
    autopilot.apply(autopilot.check(), backup=backup)
    applied_order = [name for name, _ in param_sets(fake)]
    fake.clear_received()

    preview = autopilot.preview_revert(backup)
    assert [r["name"] for r in preview] == applied_order[::-1]
    assert all(r["action"] == "restore" and not r["drifted"] for r in preview)
    assert fake.received("PARAM_SET") == []                        # the preview only reads

    results = autopilot.revert(backup)
    assert [name for name, _ in param_sets(fake)] == applied_order[::-1]
    assert all(r["ok"] for r in results)
    assert {r["name"] for r in results if r["needs_reboot"]} == {"VISO_TYPE"}
    assert fake.params == before
    assert len(ParamBackup(backup.vehicle_key, tmp_path)) == 0


def test_revert_reports_drift_and_still_restores(fake, tmp_path):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    backup = ParamBackup("ap-x", tmp_path)
    autopilot.apply(autopilot.check(), backup=backup)
    fake.params["EK3_SRC1_POSXY"] = 2.0                             # someone changed it by hand
    preview = {r["name"]: r for r in autopilot.preview_revert(backup)}
    assert preview["EK3_SRC1_POSXY"]["drifted"] and preview["EK3_SRC1_POSXY"]["current"] == 2.0
    assert not preview["VISO_TYPE"]["drifted"]
    results = {r["name"]: r for r in autopilot.revert(backup)}
    assert results["EK3_SRC1_POSXY"]["ok"] and results["EK3_SRC1_POSXY"]["drifted"]
    assert fake.params["EK3_SRC1_POSXY"] == 3.0


def test_revert_refused_while_armed(fake, tmp_path):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    backup = ParamBackup("ap-x", tmp_path)
    autopilot.apply(autopilot.check(), backup=backup)
    fake.clear_received()
    fake.set_armed(True)
    with pytest.raises(VehicleArmed):
        autopilot.revert(backup)
    assert fake.received("PARAM_SET") == [] and len(backup) == 4


def test_partial_revert_stops_at_the_first_failure_and_keeps_the_rest(fake, tmp_path):
    """Older changes are only undone after newer ones: a failed restore leaves every older entry
    (here EK3_SRC1_POSXY and VISO_TYPE) in place and in the backup."""
    autopilot = AutopilotParams(fake.http_base, timeout=0.2)
    backup = ParamBackup("ap-x", tmp_path)
    autopilot.apply(autopilot.check(), backup=backup)
    del fake.params["EK3_SRC1_VELXY"]                               # stops answering
    results = autopilot.revert(backup)
    by_name = {r["name"]: r for r in results}
    assert [r["name"] for r in results] == ["EK3_SRC_OPTIONS", "EK3_SRC1_VELXY", "EK3_SRC1_POSXY", "VISO_TYPE"]
    assert by_name["EK3_SRC_OPTIONS"]["ok"] and fake.params["EK3_SRC_OPTIONS"] == 1.0
    assert not by_name["EK3_SRC1_VELXY"]["ok"] and by_name["EK3_SRC1_VELXY"]["error"]
    for name in ("EK3_SRC1_POSXY", "VISO_TYPE"):
        assert not by_name[name]["ok"] and by_name[name]["not_attempted"]
        assert "not attempted" in by_name[name]["error"]
    assert fake.params["EK3_SRC1_POSXY"] == 6.0 and fake.params["VISO_TYPE"] == 1.0
    assert [e["name"] for e in ParamBackup("ap-x", tmp_path).entries()] == \
        ["VISO_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY"]

    fake.params["EK3_SRC1_VELXY"] = 6.0                             # answers again: retry finishes
    assert all(r["ok"] for r in autopilot.revert(backup))
    assert len(backup) == 0 and fake.params["VISO_TYPE"] == 0.0


def test_failed_viso_restore_keeps_viso_type_so_it_can_be_retried(monkeypatch):
    """If VISO_DELAY_MS isn't restored, VISO_TYPE stays 1: restoring it would hide VISO_DELAY_MS
    after the next reboot, and then it could never be reverted."""
    with FakeAutopilot(hide_viso_until_reboot=True) as fake:
        autopilot = AutopilotParams(fake.http_base, timeout=0.3)
        backup = ParamBackup(autopilot.vehicle_key(), None)
        autopilot.apply(autopilot.check(), backup=backup)
        fake.reboot()
        autopilot.apply([autopilot.check(suggested_viso_delay_ms=120)[-1]], backup=backup)
        assert backup.get("VISO_DELAY_MS")["original"] == 10.0

        real_write = autopilot.write
        monkeypatch.setattr(autopilot, "write", lambda name, value, t: None if name == "VISO_DELAY_MS"
                            else real_write(name, value, t))
        results = autopilot.revert(backup)
        assert results[0]["name"] == "VISO_DELAY_MS" and results[0]["error"] == "write not confirmed"
        assert all(r.get("not_attempted") for r in results[1:])
        assert fake.params["VISO_TYPE"] == 1.0 and fake.params["VISO_DELAY_MS"] == 120.0
        assert not any(r["needs_reboot"] for r in results)

        fake.reboot()                                               # VISO_DELAY_MS stays visible
        monkeypatch.setattr(autopilot, "write", real_write)
        assert all(r["ok"] for r in autopilot.revert(backup))
        assert fake.params["VISO_DELAY_MS"] == 10.0 and fake.params["VISO_TYPE"] == 0.0
        assert len(backup) == 0


def arm_after_first_write(fake, autopilot, monkeypatch):
    real_write = autopilot.write
    writes = []

    def write(name, value, param_type):
        result = real_write(name, value, param_type)
        writes.append(name)
        if len(writes) == 1:
            fake.set_armed(True)                                    # the pilot arms meanwhile
        return result
    monkeypatch.setattr(autopilot, "write", write)
    return writes


def test_apply_stops_when_the_vehicle_arms_mid_run(fake, tmp_path, monkeypatch):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    backup = ParamBackup("ap-x", tmp_path)
    rows = autopilot.check()
    writes = arm_after_first_write(fake, autopilot, monkeypatch)
    results = autopilot.apply(rows, backup=backup)
    assert writes == ["VISO_TYPE"]
    assert [(r["name"], r["ok"]) for r in results] == [("VISO_TYPE", True), ("EK3_SRC1_POSXY", False),
                                                       ("EK3_SRC1_VELXY", False), ("EK3_SRC_OPTIONS", False)]
    assert all(r["not_attempted"] and "armed" in r["error"] for r in results[1:])
    assert fake.params["EK3_SRC1_POSXY"] == 3.0 and fake.params["EK3_SRC_OPTIONS"] == 1.0
    assert [n for n, _ in param_sets(fake)] == ["VISO_TYPE"]
    # Nothing recorded for parameters that weren't written
    assert [e["name"] for e in backup.entries()] == ["VISO_TYPE"]
    assert results[0]["needs_reboot"]


def test_revert_stops_when_the_vehicle_arms_mid_run(fake, tmp_path, monkeypatch):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    backup = ParamBackup("ap-x", tmp_path)
    autopilot.apply(autopilot.check(), backup=backup)
    fake.clear_received()
    arm_after_first_write(fake, autopilot, monkeypatch)
    results = autopilot.revert(backup)
    assert [(r["name"], r["ok"]) for r in results] == [("EK3_SRC_OPTIONS", True), ("EK3_SRC1_VELXY", False),
                                                       ("EK3_SRC1_POSXY", False), ("VISO_TYPE", False)]
    assert all(r["not_attempted"] and "armed" in r["error"] for r in results[1:])
    assert [n for n, _ in param_sets(fake)] == ["EK3_SRC_OPTIONS"]
    assert fake.params["EK3_SRC1_POSXY"] == 6.0 and fake.params["VISO_TYPE"] == 1.0
    assert [e["name"] for e in backup.entries()] == ["VISO_TYPE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY"]


def test_apply_stops_when_the_armed_state_cant_be_read(fake, tmp_path, monkeypatch):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    rows = autopilot.check()
    calls = []
    real_is_armed = autopilot.is_armed

    def is_armed():
        calls.append(1)
        if len(calls) > 2:                                          # the start, then before the 1st write
            raise NoHeartbeat("no autopilot HEARTBEAT in mavlink2rest")
        return real_is_armed()
    monkeypatch.setattr(autopilot, "is_armed", is_armed)
    results = autopilot.apply(rows, backup=ParamBackup("ap-x", tmp_path))
    assert results[0]["ok"] and all(r["not_attempted"] for r in results[1:])
    assert "couldn't confirm the vehicle is disarmed" in results[1]["error"]
    assert [n for n, _ in param_sets(fake)] == ["VISO_TYPE"]


def test_apply_without_a_fresh_read_doesnt_record_the_stale_check_value(fake, tmp_path):
    """The value changed after the check and the fresh read gets no reply: the check's value
    must not become the "before TrackShot" value, so the parameter isn't written."""
    autopilot = AutopilotParams(fake.http_base, timeout=0.2)
    rows = [r for r in autopilot.check() if r["name"] in ("EK3_SRC1_POSXY", "EK3_SRC1_VELXY")]
    assert rows[0]["current"] == 3.0
    fake.params["EK3_SRC1_POSXY"] = 0.0                             # changed by hand after Check
    real_act = fake._act_on
    fake._act_on = lambda m: None if (m["type"] == "PARAM_REQUEST_READ"
                                      and "".join(m["param_id"]).startswith("EK3_SRC1_POSXY")) else real_act(m)
    backup = ParamBackup("ap-x", tmp_path)
    results = autopilot.apply(rows, backup=backup)
    assert not results[0]["ok"] and "couldn't read its current value" in results[0]["error"]
    assert fake.params["EK3_SRC1_POSXY"] == 0.0 and "EK3_SRC1_POSXY" not in backup
    assert results[1]["ok"] and backup.get("EK3_SRC1_VELXY")["original"] == 3.0   # the rest goes on


def drop_link_on(autopilot, monkeypatch, msg_type, name):
    """The POST of `msg_type` for `name` times out once (a tether glitch)."""
    import urllib.error
    real_send = autopilot._send
    dropped = []

    def send(message):
        if (not dropped and message["type"] == msg_type
                and "".join(message["param_id"]).rstrip("\u0000") == name):
            dropped.append(name)
            raise urllib.error.URLError("timed out")
        return real_send(message)
    monkeypatch.setattr(autopilot, "_send", send)
    return dropped


def test_a_dropped_link_mid_apply_still_reports_what_was_written(fake, tmp_path, monkeypatch):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    rows = autopilot.check()
    backup = ParamBackup("ap-x", tmp_path)
    dropped = drop_link_on(autopilot, monkeypatch, "PARAM_REQUEST_READ", "EK3_SRC1_POSXY")
    results = {r["name"]: r for r in autopilot.apply(rows, backup=backup)}     # doesn't raise
    assert dropped
    assert results["VISO_TYPE"]["ok"] and results["VISO_TYPE"]["needs_reboot"]
    assert not results["EK3_SRC1_POSXY"]["ok"] and "mavlink2rest" in results["EK3_SRC1_POSXY"]["error"]
    assert results["EK3_SRC1_VELXY"]["ok"] and results["EK3_SRC_OPTIONS"]["ok"]
    assert fake.params["EK3_SRC1_POSXY"] == 3.0 and "EK3_SRC1_POSXY" not in backup


def test_a_dropped_link_mid_revert_stops_and_reports_what_was_restored(fake, tmp_path, monkeypatch):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    backup = ParamBackup("ap-x", tmp_path)
    autopilot.apply(autopilot.check(), backup=backup)
    drop_link_on(autopilot, monkeypatch, "PARAM_REQUEST_READ", "EK3_SRC1_POSXY")
    results = autopilot.revert(backup)                              # doesn't raise
    assert [(r["name"], r["ok"]) for r in results] == [("EK3_SRC_OPTIONS", True), ("EK3_SRC1_VELXY", True),
                                                       ("EK3_SRC1_POSXY", False), ("VISO_TYPE", False)]
    assert "mavlink2rest" in results[2]["error"] and results[3]["not_attempted"]
    assert [e["name"] for e in backup.entries()] == ["VISO_TYPE", "EK3_SRC1_POSXY"]
    assert all(r["ok"] for r in autopilot.revert(backup)) and len(backup) == 0


def test_absent_originals_are_skipped(fake, tmp_path):
    autopilot = AutopilotParams(fake.http_base, timeout=1.0)
    backup = ParamBackup("ap-x", tmp_path)
    backup.record_original("VISO_TYPE", 0.0, "MAV_PARAM_TYPE_INT8", True)
    fake.params["VISO_TYPE"] = 1.0
    backup.mark_written("VISO_TYPE", 1.0)
    backup.record_original("VISO_DELAY_MS", None, None, False)     # was hidden before TrackShot
    backup.mark_written("VISO_DELAY_MS", 120.0)
    fake.params["VISO_DELAY_MS"] = 120.0
    preview = autopilot.preview_revert(backup)
    assert [(r["name"], r["action"]) for r in preview] == [("VISO_DELAY_MS", "skip"), ("VISO_TYPE", "restore")]
    results = autopilot.revert(backup)
    assert results[0]["skipped"] and "VISO_TYPE" in results[0]["error"]
    assert results[1]["ok"] and fake.params["VISO_TYPE"] == 0.0
    assert fake.params["VISO_DELAY_MS"] == 120.0                   # left alone
    assert len(backup) == 0                                         # everything else was restored


def test_hidden_viso_params_round_trip():
    """VISO_DELAY_MS only exists after VISO_TYPE=1 and a reboot (as in ArduPilot). Reverting
    restores it before VISO_TYPE, and after the next reboot it is hidden again."""
    with FakeAutopilot(hide_viso_until_reboot=True) as fake:
        autopilot = AutopilotParams(fake.http_base, timeout=0.3)
        backup = ParamBackup(autopilot.vehicle_key(), None)
        rows = autopilot.check(suggested_viso_delay_ms=120)
        assert rows[-1]["current"] is None                         # hidden
        results = {r["name"]: r for r in autopilot.apply(rows, backup=backup)}
        assert not results["VISO_DELAY_MS"]["ok"] and "VISO_DELAY_MS" not in backup
        fake.reboot()
        row = autopilot.check(suggested_viso_delay_ms=120)[-1]
        assert row["current"] == 10.0                               # ArduPilot's default
        assert autopilot.apply([row], backup=backup)[0]["ok"]
        assert backup.get("VISO_DELAY_MS")["original"] == 10.0

        fake.clear_received()
        assert all(r["ok"] for r in autopilot.revert(backup))
        assert [n for n, _ in param_sets(fake)][:2] == ["VISO_DELAY_MS", "EK3_SRC_OPTIONS"]
        assert param_sets(fake)[-1] == ("VISO_TYPE", 0.0)
        fake.reboot()
        assert autopilot.read("VISO_DELAY_MS") is None
