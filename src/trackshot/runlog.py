"""Per-run data log: one CSV row per VIO pose, for reviewing a dive afterwards.

Each run (Start → Stop) writes `<log dir>/runs/run_<YYYYmmdd_HHMMSS>.csv` next to trackshot.log
(macOS: ~/Library/Logs/TrackShot/runs/). A row holds the VIO's raw camera pose, the vehicle-frame
pose after the camera mounting rotation, the tracking state, what was sent to the autopilot (the
body-frame delta and/or the NED estimate) and the autopilot's own attitude and depth at the time.
That is enough to see, after the fact, whether forward, sideways and vertical motion came out on
the right axes, and what the EKF was given.

Rows are queued and written by a background thread, so the camera thread never waits on the disk.
The oldest run files beyond KEEP_RUNS are deleted when a new run starts.
"""

from __future__ import annotations

import csv
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from loguru import logger

KEEP_RUNS = 50
QUEUE_MAX = 10_000          # ~8 minutes at 20 Hz if the disk stalls; beyond that rows are dropped

COLUMNS = (
    "t_wall", "t_capture_us",
    # the VIO's camera pose as reported (FLU camera axes)
    "cam_x", "cam_y", "cam_z", "cam_qw", "cam_qx", "cam_qy", "cam_qz",
    # the vehicle pose after the mounting rotation (FLU: x forward, y left, z up at start)
    "veh_x", "veh_y", "veh_z", "veh_qw", "veh_qx", "veh_qy", "veh_qz",
    "tracking", "sending",
    # the vehicle-frame delta (FRD: forward, right, down) when one was made
    "d_fwd", "d_right", "d_down", "d_span_us",
    # what went to the autopilot: "delta", "estimate" or empty
    "sent",
    # the NED track (north, east), its velocity, and the heading used to rotate into it
    "ned_n", "ned_e", "ned_vn", "ned_ve", "ned_heading_deg",
    # the autopilot's own attitude and depth (telemetry, ~2 Hz)
    "ap_heading_deg", "ap_roll_deg", "ap_pitch_deg", "ap_depth_m",
)


def runs_dir() -> Path:
    from platformdirs import user_log_dir
    return Path(user_log_dir("TrackShot", appauthor=False)) / "runs"


class RunLog:
    """CSV writer on its own thread. `row()` never blocks; `close()` flushes and stops."""

    def __init__(self, directory: Optional[Path] = None, started: Optional[datetime] = None):
        self.directory = Path(directory) if directory is not None else runs_dir()
        stamp = (started or datetime.now()).strftime("%Y%m%d_%H%M%S")
        self.path = self.directory / f"run_{stamp}.csv"
        self._queue: queue.Queue = queue.Queue(maxsize=QUEUE_MAX)
        self.dropped = 0
        self.rows = 0
        self._thread = threading.Thread(target=self._run, name="trackshot-runlog", daemon=True)
        self._thread.start()

    def row(self, **values: Any) -> None:
        try:
            self._queue.put_nowait(values)
        except queue.Full:
            self.dropped += 1

    def close(self, timeout: float = 5.0) -> None:
        try:
            self._queue.put(None, timeout=timeout)
        except queue.Full:
            return
        self._thread.join(timeout)

    def _run(self) -> None:
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            _prune(self.directory, KEEP_RUNS - 1)
            with open(self.path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
                writer.writeheader()
                last_flush = time.monotonic()
                while True:
                    item = self._queue.get()
                    if item is None:
                        break
                    writer.writerow({k: _fmt(v) for k, v in item.items()})
                    self.rows += 1
                    if time.monotonic() - last_flush > 1.0:
                        f.flush()
                        last_flush = time.monotonic()
            logger.info(f"Run log: {self.path} ({self.rows} rows" +
                        (f", {self.dropped} dropped)" if self.dropped else ")"))
        except OSError as e:
            logger.warning(f"Run log not written ({self.path}): {e}")
            while self._queue.get() is not None:   # keep draining so close() returns
                pass


def _fmt(v: Any) -> Any:
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, bool):
        return int(v)
    return "" if v is None else v


def _prune(directory: Path, keep: int) -> None:
    runs = sorted(directory.glob("run_*.csv"))
    for old in runs[:max(0, len(runs) - keep)]:
        try:
            old.unlink()
        except OSError:
            pass
