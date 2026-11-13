"""Test doubles for the UI: an Engine with the documented interface, and realistic data for it.

FakeEngine mirrors trackshot.engine.Engine's documented signals and methods (ARCHITECTURE.md) and
records every call, so the UI can be driven and checked without hardware or the real engine.
The status helpers build the statusUpdated dict from the real SourceStats / TransportStats /
BandwidthEstimate dataclasses; OdometryHealth comes from trackshot.core.odometry when that module
exists, otherwise a dict with the same fields (the UI reads either).
"""

from __future__ import annotations

import copy
import math
import threading
import time
from typing import Optional

import numpy as np
from PySide6.QtCore import QObject, Signal

from trackshot import bandwidth
from trackshot.config import Settings
from trackshot.mavlink.base import TransportStats
from trackshot.vio.source import (STATE_CONNECTING, STATE_IDLE, STATE_RUNNING, STATE_STOPPED, PoseSample,
                                  PreviewFrame, SourceStats)


class FakeAutopilotParams:
    """Stands in for trackshot.autopilot.params.AutopilotParams. Records the thread each call ran on.

    apply()/revert() follow the real contract with a ParamBackup: originals are recorded before a
    write (first change only), and revert restores them newest first and drops them from the backup.
    """

    def __init__(self, rows: Optional[list[dict]] = None, error: Optional[BaseException] = None,
                 delay_s: float = 0.0, key: str = "ap-fake0001", armed: bool = False):
        self.rows = rows if rows is not None else [
            {"name": "AHRS_EKF_TYPE", "current": 3.0, "recommended": 3, "needs_reboot": True, "ok": True},
            {"name": "VISO_TYPE", "current": 0.0, "recommended": 1, "needs_reboot": True, "ok": False},
            {"name": "EK3_SRC1_POSXY", "current": 3.0, "recommended": 6, "needs_reboot": False, "ok": False},
            {"name": "EK3_SRC1_POSZ", "current": 1.0, "recommended": 1, "needs_reboot": False, "ok": True},
        ]
        for row in self.rows:
            row.setdefault("param_type", "MAV_PARAM_TYPE_INT8")
        self.error = error
        self.delay_s = delay_s
        self.key = key
        self.armed = armed
        self.cancel = threading.Event()
        self.check_calls: list[Optional[int]] = []
        self.rangefinder_calls: list[bool] = []
        self.apply_calls: list[list[dict]] = []
        self.revert_calls: int = 0
        self.preview_calls: int = 0
        self.threads: list[int] = []

    def _row(self, name: str) -> Optional[dict]:
        return next((r for r in self.rows if r["name"] == name), None)

    def _set(self, name: str, value: float) -> None:
        row = self._row(name)
        if row is not None:
            row["current"] = float(value)
            row["ok"] = row["current"] == row["recommended"]

    def is_armed(self) -> bool:
        self.threads.append(threading.get_ident())
        if self.error is not None:
            raise self.error
        return self.armed

    def vehicle_identity(self, request_timeout: float = 2.0) -> tuple[str, str]:
        self.threads.append(threading.get_ident())
        return self.key, f"fake autopilot {self.key}"

    def vehicle_key(self, request_timeout: float = 2.0) -> str:
        return self.vehicle_identity()[0]

    def check(self, suggested_viso_delay_ms: Optional[int] = None, include_rangefinder: bool = False) -> list[dict]:
        self.threads.append(threading.get_ident())
        self.check_calls.append(suggested_viso_delay_ms)
        self.rangefinder_calls.append(include_rangefinder)
        time.sleep(self.delay_s)
        if self.error is not None:
            raise self.error
        rows = [dict(r) for r in self.rows]
        if suggested_viso_delay_ms is not None:
            rows.append({"name": "VISO_DELAY_MS", "current": 10.0, "recommended": suggested_viso_delay_ms,
                         "needs_reboot": False, "ok": suggested_viso_delay_ms == 10,
                         "param_type": "MAV_PARAM_TYPE_INT16"})
        return rows

    def apply(self, diffs: list[dict], backup=None) -> list[dict]:
        self.threads.append(threading.get_ident())
        self.apply_calls.append(diffs)
        if self.error is not None:
            raise self.error
        results = []
        for d in diffs:
            name = d["name"]
            if backup is not None:
                backup.record_original(name, d["current"], d.get("param_type"), d["needs_reboot"])
            self._set(name, d["recommended"])
            if backup is not None:
                backup.mark_written(name, float(d["recommended"]))
            results.append({"name": name, "ok": True, "needs_reboot": bool(d["needs_reboot"]), "error": None})
        return results

    def preview_revert(self, backup) -> list[dict]:
        self.threads.append(threading.get_ident())
        self.preview_calls += 1
        rows = []
        for e in reversed(backup.entries()):
            row = self._row(e["name"])
            current = None if row is None else row["current"]
            rows.append({"name": e["name"], "current": current, "original": e["original"],
                         "absent": e["absent"], "written_by_trackshot": e["written"],
                         "drifted": e["written"] is not None and current is not None and current != e["written"],
                         "unconfirmed": e["written"] is None, "needs_reboot": e["needs_reboot"],
                         "action": "skip" if e["absent"] else "restore"})
        return rows

    def revert(self, backup) -> list[dict]:
        self.threads.append(threading.get_ident())
        self.revert_calls += 1
        if self.armed:
            from trackshot.autopilot.params import VehicleArmed
            raise VehicleArmed("armed")
        results = []
        for e in reversed(backup.entries()):
            row = self._row(e["name"])
            current = None if row is None else row["current"]
            drifted = e["written"] is not None and current is not None and current != e["written"]
            if e["absent"]:
                results.append({"name": e["name"], "ok": False, "skipped": True, "needs_reboot": False,
                                "drifted": False, "error": "didn't exist before TrackShot"})
                continue
            self._set(e["name"], e["original"])
            backup.remove(e["name"])
            results.append({"name": e["name"], "ok": True, "skipped": False, "drifted": drifted,
                            "needs_reboot": e["needs_reboot"], "error": None, "original": e["original"]})
        return results


class FakeEngine(QObject):
    stateChanged = Signal(str, str)
    statusUpdated = Signal(dict)
    poseUpdated = Signal(object)
    previewUpdated = Signal(object)
    logMessage = Signal(str, str)

    def __init__(self, settings: Settings, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._settings = copy.deepcopy(settings)
        self._state = STATE_IDLE
        self.params = FakeAutopilotParams()
        self.calls: list[tuple] = []
        self.applied: list[Settings] = []
        self.restart_blocked: Optional[str] = None   # what restart_vio_blocked() reports while running

    @property
    def settings(self) -> Settings:
        return copy.deepcopy(self._settings)

    @property
    def state(self) -> str:
        return self._state

    def _set_state(self, state: str, message: str) -> None:
        self._state = state
        self.stateChanged.emit(state, message)

    def start(self) -> None:
        self.calls.append(("start",))
        self._set_state(STATE_CONNECTING, "Connecting to the camera…")
        self._set_state(STATE_RUNNING, "VIO running")

    def stop(self) -> None:
        self.calls.append(("stop",))
        self._set_state(STATE_STOPPED, "Stopped")

    def restart_vio(self) -> bool:
        self.calls.append(("restart_vio",))
        if self.restart_blocked is not None:
            return False
        self.stop()
        self.start()
        return True

    def restart_vio_blocked(self) -> Optional[str]:
        if self._state != STATE_RUNNING:
            return "VIO isn't running"
        return self.restart_blocked

    def apply_settings(self, s: Settings) -> None:
        self.calls.append(("apply_settings", s))
        self.applied.append(s)
        self._settings = copy.deepcopy(s)

    def set_recording(self, on: bool) -> None:
        self.calls.append(("set_recording", on))

    def set_preview(self, vio: bool, color: bool) -> None:
        self.calls.append(("set_preview", vio, color))

    def autopilot_params(self) -> FakeAutopilotParams:
        return self.params

    def called(self, name: str) -> list[tuple]:
        return [c for c in self.calls if c[0] == name]


# --- Realistic data ----------------------------------------------------------------------------

def odometry_health(**overrides):
    fields = dict(fps=9.8, latency_ms=64.0, suggested_viso_delay_ms=60, poses=1200, deltas=1195, jumps=1,
                  rejected_spans=2, last_pose=(3.2, -1.1, -0.4, 1.0, 0.0, 0.0, 0.0), last_pose_age_s=0.08,
                  stalled=False, tracking="good", sending=True)
    fields.update(overrides)
    try:
        from trackshot.core.odometry import OdometryHealth
        return OdometryHealth(**fields)
    except (ImportError, TypeError):
        return fields


def make_status(settings: Optional[Settings] = None, state: str = STATE_RUNNING, *, odometry=None,
                recording: bool = False, armed: bool = False, rx_mbit: Optional[float] = 18.4) -> dict:
    settings = settings or Settings()
    return {
        "state": state,
        "source": SourceStats(fps=9.8, rx_mbit=rx_mbit, chip_temp_c=58.5, camera_name="OAK-D-W-PoE",
                              camera_id="192.168.2.118", imu="BNO086", backend=settings.backend),
        "odometry": odometry if odometry is not None else odometry_health(),
        "transport": TransportStats(kind="mavlink2rest", target=settings.mavlink2rest_ws_url(), connected=True,
                                    sent=11942, dropped=3, heartbeat_age_s=0.4),
        "capture": {"recording": recording, "folder": "/tmp/TrackShot Captures/2026-11-10_1432",
                    "counts": {"center": 412, "stereo": 388}, "dropped": 0, "free_gb": 211.4,
                    "disk_full": False, "last_error": None,
                    "last_file": "/tmp/TrackShot Captures/2026-11-10_1432/center/tl000000_000412.jpg"},
        "telemetry": {"lat": -33.86, "lon": 151.21, "heading_deg": 87.0, "roll_deg": 1.2, "pitch_deg": -3.4,
                      "depth_m": 4.37, "temperature_c": 18.2, "altitude_m": None, "mission_seq": None,
                      "connected": True, "armed": armed, "mode": "POSHOLD", "heartbeat_age_s": 0.4, "age_s": 0.3},
        "bandwidth": bandwidth.estimate(settings),
    }


_T0 = time.time()


def lawnmower_pose(i: int, t0: Optional[float] = None, rate_hz: float = 10.0) -> tuple:
    """poseUpdated payload for step `i` of a survey: 10 m legs, 2 m apart, slight depth wander.
    Times are wall-clock (from import time), like the engine's."""
    t0 = _T0 if t0 is None else t0
    leg_len, spacing, speed = 10.0, 2.0, 0.5
    s = i * speed / rate_hz
    period = leg_len + spacing
    leg, along = divmod(s, period)
    if along < leg_len:
        x = along if int(leg) % 2 == 0 else leg_len - along
        y = leg * spacing
        yaw = 0.0 if int(leg) % 2 == 0 else math.pi
    else:
        x = leg_len if int(leg) % 2 == 0 else 0.0
        y = leg * spacing + (along - leg_len)
        yaw = math.pi / 2
    z = -0.3 * math.sin(s / 7.0)
    q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    t = t0 + i / rate_hz
    sample = PoseSample(t_capture_us=int((t - 0.06) * 1e6), t_received=t, position=(x, y, z), quaternion=q,
                        tracking=True)
    return sample, (x, y, z, *q)


def make_preview(kind: str = "vio", width: int = 320, height: int = 200, n_features: int = 60,
                 seed: int = 0) -> PreviewFrame:
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:height, 0:width]
    texture = 110 + 60 * np.sin(xx / 9.0) * np.cos(yy / 13.0) + rng.normal(0, 12, (height, width))
    grey = np.clip(texture, 0, 255).astype(np.uint8)
    if kind == "color":
        image = np.stack([grey // 3, (grey * 0.8).astype(np.uint8), grey], axis=-1)
        return PreviewFrame(kind="color", image=image, t_received=time.time())
    features = np.column_stack([rng.uniform(0, width, n_features), rng.uniform(0, height, n_features)])
    return PreviewFrame(kind="vio", image=grey, t_received=time.time(), features=features)
