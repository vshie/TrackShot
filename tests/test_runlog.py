"""trackshot.runlog: the per-run CSV data log."""

import csv
from datetime import datetime

from trackshot import runlog
from trackshot.runlog import COLUMNS, RunLog


def test_rows_are_written_with_the_header(tmp_path):
    log = RunLog(tmp_path, started=datetime(2026, 11, 12, 13, 24, 5))
    log.row(t_wall=1.5, d_fwd=0.1234567, sent="estimate", sending=True, tracking="tracking", ap_depth_m=None)
    log.row(t_wall=1.6, cam_x=2.0)
    log.close()
    assert log.path.name == "run_20261112_132405.csv"
    rows = list(csv.DictReader(open(log.path)))
    assert list(rows[0]) == list(COLUMNS)
    assert rows[0]["d_fwd"] == "0.123457" and rows[0]["sent"] == "estimate" and rows[0]["sending"] == "1"
    assert rows[0]["ap_depth_m"] == ""
    assert rows[1]["cam_x"] == "2"


def test_old_runs_are_pruned(tmp_path, monkeypatch):
    monkeypatch.setattr(runlog, "KEEP_RUNS", 3)
    for i in range(5):
        (tmp_path / f"run_2026010{i}_000000.csv").write_text("x")
    RunLog(tmp_path, started=datetime(2026, 2, 1)).close()
    assert sorted(p.name for p in tmp_path.glob("run_*.csv")) == [
        "run_20260103_000000.csv", "run_20260104_000000.csv", "run_20260201_000000.csv"]


def test_an_unwritable_folder_is_reported_not_raised(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("not a folder")
    log = RunLog(blocker / "runs")
    log.row(t_wall=1.0)
    log.close()
    assert not log.path.exists()
