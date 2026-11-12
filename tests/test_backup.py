"""ParamBackup: the on-disk record of parameter values from before TrackShot changed them."""

import json
import os

import pytest

from trackshot.autopilot import backup as backup_module
from trackshot.autopilot.backup import BackupError, ParamBackup, safe_key


def test_default_location_is_isolated_in_tests(isolated_param_backups):
    b = ParamBackup("ap-1234")
    assert b.path == isolated_param_backups / "ap-1234.json"


def test_first_change_only_and_persisted(tmp_path):
    b = ParamBackup("ap-1", tmp_path)
    assert b.record_original("VISO_TYPE", 0.0, "MAV_PARAM_TYPE_INT8", True)
    # On disk before any write is marked
    on_disk = json.loads((tmp_path / "ap-1.json").read_text())
    assert on_disk["params"]["VISO_TYPE"]["original"] == 0.0
    assert on_disk["params"]["VISO_TYPE"]["status"] == "pending"
    b.mark_written("VISO_TYPE", 1.0)
    # A later apply must never replace the pre-TrackShot value
    assert not b.record_original("VISO_TYPE", 1.0, "MAV_PARAM_TYPE_INT8", True)
    assert b.record_original("EK3_SRC1_POSXY", 3.0, "MAV_PARAM_TYPE_INT8", False)

    again = ParamBackup("ap-1", tmp_path)
    entries = again.entries()
    assert [e["name"] for e in entries] == ["VISO_TYPE", "EK3_SRC1_POSXY"]
    assert entries[0]["original"] == 0.0 and entries[0]["written"] == 1.0 and entries[0]["status"] == "written"
    assert entries[0]["order"] < entries[1]["order"]
    again.remove("VISO_TYPE")
    assert "VISO_TYPE" not in ParamBackup("ap-1", tmp_path)
    assert not list(tmp_path.glob(".*.tmp"))


def test_absent_original(tmp_path):
    b = ParamBackup("ap-1", tmp_path)
    b.record_original("VISO_DELAY_MS", None, None, False)
    entry = ParamBackup("ap-1", tmp_path).get("VISO_DELAY_MS")
    assert entry["absent"] is True and entry["original"] is None


def test_corrupt_file_is_set_aside_not_discarded(tmp_path):
    (tmp_path / "ap-1.json").write_text("{not json")
    b = ParamBackup("ap-1", tmp_path)
    assert len(b) == 0 and b.problem and "unreadable" in b.problem
    [aside] = b.set_aside_files()
    assert aside.read_text() == "{not json"
    # A second corrupt file in the same second doesn't overwrite the first one set aside
    (tmp_path / "ap-1.json").write_text('{"params": {"X": 1}}')
    ParamBackup("ap-1", tmp_path)
    assert {p.read_text() for p in b.set_aside_files()} == {"{not json", '{"params": {"X": 1}}'}


def test_failed_save_remembers_nothing(tmp_path, monkeypatch):
    b = ParamBackup("ap-1", tmp_path)

    def broken_replace(*_):
        raise OSError("disk full")
    monkeypatch.setattr(backup_module.os, "replace", broken_replace)
    with pytest.raises(BackupError):
        b.record_original("VISO_TYPE", 0.0, "MAV_PARAM_TYPE_INT8", True)
    assert "VISO_TYPE" not in b
    assert not list(tmp_path.glob(".*.tmp"))
    monkeypatch.setattr(backup_module.os, "replace", os.replace)
    assert not (tmp_path / "ap-1.json").exists()


def test_safe_key():
    assert safe_key("host-192.168.2.2:8088") == "host-192.168.2.2_8088"
    assert safe_key("../../etc") == "etc"
    assert safe_key("") == "unknown"
