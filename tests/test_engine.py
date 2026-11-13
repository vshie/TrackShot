"""Engine integration tests: SimPoseSource → Engine → FakeAutopilot over mavlink2rest WS and UDP.

These run the real modules end to end (simulator, odometry, transports, telemetry poller, capture
writer) against the in-process fake BlueOS, on the real clock. The simulator flies its first
survey leg straight along +x at 0.3 m/s with the depth rising, so every forwarded delta has a
known sign in the FRD body frame.
"""

from __future__ import annotations

import math
import os
import statistics
import threading
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from loguru import logger  # noqa: E402
from PySide6.QtCore import QSettings  # noqa: E402

from trackshot import engine as engine_module  # noqa: E402
from trackshot.config import Settings  # noqa: E402
from trackshot.engine import Engine  # noqa: E402
from trackshot.testing.fake_autopilot import FakeAutopilot  # noqa: E402
from trackshot.vio.sim import SPEED_MPS, SimPoseSource  # noqa: E402
from trackshot.vio.source import (STATE_ERROR, STATE_RUNNING, STATE_STOPPED, PoseSource,  # noqa: E402
                                  SourceStats)

FPS = 20
STATUS_KEYS = {"state", "source", "odometry", "transport", "capture", "telemetry", "bandwidth", "altitude"}
# Simulated seconds after RUNNING: a 0.8 m VIO jump (16 m/s implied), then 2.3 s with no poses
JUMP_AT_S, DROPOUT_AT_S, DROPOUT_S = 1.0, 1.5, 2.3
FAULT_SCRIPT = [
    {"at_s": JUMP_AT_S, "event": "jump", "offset": [0.8, 0.0, 0.0]},
    {"at_s": DROPOUT_AT_S, "event": "dropout", "duration_s": DROPOUT_S},
]
MAIN_THREAD = threading.get_ident()


def sim_factory(script=None, seed=1):
    def make(settings, callbacks):
        return SimPoseSource(settings, callbacks, seed=seed, realtime=True, script=script)
    return make


@pytest.fixture
def fake():
    with FakeAutopilot() as autopilot:
        yield autopilot


@pytest.fixture
def make_settings(fake, tmp_path):
    def make(**overrides) -> Settings:
        values = dict(source="sim", fps=FPS, vehicle_ip=fake.vehicle_ip, udp_url=fake.udp_url,
                      capture_dir=str(tmp_path / "captures"), health_rate_hz=4.0)
        values.update(overrides)
        return Settings(**values)
    return make


@pytest.fixture
def make_engine(qtbot):
    engines = []

    def make(settings: Settings, factory=None) -> Engine:
        engine = Engine(settings, source_factory=factory or sim_factory())
        engines.append(engine)
        return engine

    yield make
    for engine in engines:
        engine.shutdown(timeout=8.0)


class Recorder:
    """Collects an engine's signals, noting which thread delivered them."""

    def __init__(self, engine: Engine):
        self.states: list[tuple[str, str]] = []
        self.statuses: list[dict] = []
        self.poses: list[tuple] = []
        self.previews: list = []
        self.logs: list[tuple[str, str]] = []
        self.threads: set[int] = set()
        self.first_running: float | None = None
        engine.stateChanged.connect(self._state)
        engine.statusUpdated.connect(lambda s: self._add(self.statuses, s))
        engine.poseUpdated.connect(lambda p: self._add(self.poses, (time.monotonic(), p)))
        engine.previewUpdated.connect(lambda f: self._add(self.previews, (time.monotonic(), f)))
        engine.logMessage.connect(lambda level, text: self._add(self.logs, (level, text)))

    def _add(self, bucket: list, item) -> None:
        self.threads.add(threading.get_ident())
        bucket.append(item)

    def _state(self, state: str, message: str) -> None:
        self._add(self.states, (state, message))
        if state == STATE_RUNNING and self.first_running is None:
            self.first_running = time.time()


def run_until_sim_time(qtbot, engine: Engine, sim_s: float, timeout_s: float = 15.0) -> None:
    def reached():
        source = engine.status()["source"]
        assert source is not None and source.extra["sim_time_s"] >= sim_s
    qtbot.waitUntil(reached, timeout=int(timeout_s * 1000))


def body_speed(msg: dict) -> float:
    return math.dist((0, 0, 0), msg["position_delta"]) / (msg["time_delta_usec"] / 1e6)


# --- End to end ------------------------------------------------------------------------------------

@pytest.mark.parametrize("transport", ["mavlink2rest", "udp"])
def test_sim_to_fake_autopilot_end_to_end(qtbot, fake, make_settings, make_engine, transport):
    engine = make_engine(make_settings(transport=transport, odometry_output="position_delta"), sim_factory(FAULT_SCRIPT))
    rec = Recorder(engine)
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    run_until_sim_time(qtbot, engine, DROPOUT_AT_S + DROPOUT_S + 0.8)
    health = engine.status()["odometry"]
    transport_stats = engine.status()["transport"]
    engine.stop()
    assert engine.wait_idle(8.0)

    deltas = fake.received("VISION_POSITION_DELTA")
    assert len(deltas) > 30
    assert {m["_via"] for m in deltas} == {"ws" if transport == "mavlink2rest" else "udp"}
    # MAVLink header: the autopilot's system, MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
    assert all(m["_header"] == {"system_id": 1, "component_id": 197} for m in deltas)

    # Spans come from capture times, exactly one frame apart; none spans a jump or the dropout
    spans = [m["time_delta_usec"] for m in deltas]
    assert statistics.median(spans) == pytest.approx(1e6 / FPS, abs=1000)
    assert max(spans) < 2_000_000
    assert max(body_speed(m) for m in deltas) < 3.0
    assert health.jumps >= 1 and health.rejected_spans >= 1

    # Deltas resumed after the dropout, so "no spanning delta" is a real check
    resumed_after = rec.first_running + DROPOUT_AT_S + DROPOUT_S
    assert any(m["_time"] > resumed_after + 0.2 for m in deltas)

    # Straight along the heading (+x), depth rising (FLU +z) → FRD +x, ~0 y, −z
    xs, ys, zs = zip(*(m["position_delta"] for m in deltas))
    assert statistics.median(xs) == pytest.approx(SPEED_MPS / FPS, rel=0.15)
    assert abs(statistics.median(ys)) < 0.002
    assert statistics.median(zs) < 0
    assert all(abs(a) < 0.01 for m in deltas for a in m["angle_delta"])
    assert all(m["confidence"] == 100 for m in deltas)
    # time_usec is the host clock at send time
    assert all(abs(m["time_usec"] / 1e6 - m["_time"]) < 1.0 for m in deltas)

    # Health values for Cockpit
    names = {m["name"] for m in fake.received("NAMED_VALUE_FLOAT")}
    assert {"TS_FPS", "TS_LAT_MS", "TS_JUMPS", "TS_TRACK"} <= names
    assert transport_stats.errors == 0

    # Status at 4 Hz with every documented key; the UI got everything on the GUI thread
    running = [s for s in rec.statuses if s["state"] == STATE_RUNNING]
    assert running and all(set(s) == STATUS_KEYS for s in rec.statuses)
    assert all(s["source"] is not None and s["odometry"] is not None and s["transport"] is not None
               for s in running)
    assert any(s["telemetry"] and s["telemetry"]["connected"] for s in running)
    # Suggested VISO_DELAY_MS = latency + half a frame interval + transit, to the nearest 10 ms
    # the simulator adds ~50 ms; a busy CI runner adds scheduling delay on top
    assert 40 <= health.latency_ms <= 150
    assert abs(health.suggested_viso_delay_ms - (health.latency_ms + 500 / FPS + 5)) <= 10
    assert rec.threads == {MAIN_THREAD}
    assert [s for s, _ in rec.states][-1] == STATE_STOPPED


def test_pose_and_preview_signals_are_throttled(qtbot, make_settings, make_engine):
    engine = make_engine(make_settings(fps=40, width=160, height=100))
    rec = Recorder(engine)
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    qtbot.wait(1500)
    engine.stop()

    def max_rate(events):
        times = [t for t, _ in events]
        # Most events inside any 1 s window
        return max(sum(1 for u in times if t <= u < t + 1.0) for t in times)

    assert len(rec.poses) > 10 and max_rate(rec.poses) <= 21
    assert rec.previews and max_rate(rec.previews) <= 11
    sample, pose, nav = rec.poses[-1][1]
    assert len(pose) == 7 and sample.position[0] > 0
    assert rec.threads == {MAIN_THREAD}


def test_capture_writes_files(qtbot, make_settings, make_engine, tmp_path):
    engine = make_engine(make_settings(center_capture_fps=4.0, stereo_capture_fps=4.0, center_width=320,
                                       center_height=180, mono_width=320, mono_height=200))
    engine.set_recording(True)
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)

    def written():
        snap = engine.status()["capture"]
        assert snap["counts"]["center"] >= 3 and snap["counts"]["stereo"] >= 3
    qtbot.waitUntil(written, timeout=8000)
    snap = engine.status()["capture"]
    assert snap["recording"] and snap["dropped"] == 0
    engine.shutdown()

    captures = tmp_path / "captures"
    assert len(list(captures.rglob("center/*.jpg"))) >= 3
    assert len(list(captures.rglob("stereo/left/*.jpg"))) >= 3
    assert len(list(captures.rglob("stereo/right/*.jpg"))) >= 3
    assert list(captures.rglob("*.csv"))


def test_repeated_start_stop_does_not_leak_threads(qtbot, make_settings, make_engine):
    engine = make_engine(make_settings())
    baseline = threading.active_count()
    for _ in range(3):
        engine.start()
        qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
        qtbot.wait(200)
        engine.stop()
        assert engine.state == STATE_STOPPED
        assert engine.wait_idle(8.0)
    # The telemetry poller's short-lived HTTP fetch threads finish within their 0.5 s timeout
    qtbot.waitUntil(lambda: threading.active_count() <= baseline + 1, timeout=3000)
    engine.stop()   # idempotent
    assert engine.state == STATE_STOPPED


def test_start_while_stopping_starts_once_released(qtbot, make_settings, make_engine):
    engine = make_engine(make_settings())
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    engine.stop()
    engine.start()   # the previous run is probably still closing
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=8000)
    assert engine.status()["odometry"].poses >= 0


def test_a_pending_start_survives_done_arriving_while_the_teardown_thread_lives(
        qtbot, make_settings, make_engine, monkeypatch):
    """_teardownDone is emitted from the teardown thread's last line; the GUI thread can handle it
    before that thread has exited. The pending start must still begin (it used to be skipped,
    leaving the engine in CONNECTING for good)."""
    original = Engine._teardown_worker

    def slow_exit(self, run):
        original(self, run)
        time.sleep(0.5)          # "done" is delivered while this thread is still alive
    monkeypatch.setattr(Engine, "_teardown_worker", slow_exit)
    engine = make_engine(make_settings())
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    engine.apply_settings(make_settings(fps=FPS + 1))           # restart: stop() then start()
    assert engine._start_pending
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    assert engine.settings.fps == FPS + 1 and not engine._start_pending


def test_shutdown_flushes_a_replaced_capture_writer(qtbot, make_settings, make_engine, tmp_path, monkeypatch):
    """Changing the capture folder hands the old writer's backlog to a closing thread; quitting
    straight after must still write it (the interpreter's exit kills daemon threads)."""
    from trackshot.capture import writer as writer_module
    from trackshot.vio.source import CapturedJpeg
    real_embed = writer_module.meta.embed_metadata

    def slow_embed(*args, **kwargs):
        time.sleep(0.05)
        return real_embed(*args, **kwargs)
    monkeypatch.setattr(writer_module.meta, "embed_metadata", slow_embed)
    first, second = tmp_path / "first", tmp_path / "second"
    engine = make_engine(make_settings(capture_dir=str(first)))
    engine.set_recording(True)
    jpeg = _tiny_jpeg()
    writer = engine._writer
    for i in range(20):
        writer.submit(CapturedJpeg(stream="center", data=jpeg, t_capture=time.time(), sequence=i), None, None, None)
    engine.apply_settings(make_settings(capture_dir=str(second)))
    assert engine._writer is not writer
    engine.shutdown(timeout=10.0)
    assert len(list(first.rglob("*.jpg"))) == 20


def _tiny_jpeg() -> bytes:
    import io
    from PIL import Image
    buf = io.BytesIO()
    Image.new("RGB", (32, 24), (90, 120, 150)).save(buf, format="JPEG")
    return buf.getvalue()


def test_apply_settings_restarts_only_when_needed(qtbot, make_settings, make_engine):
    settings = make_settings()
    engine = make_engine(settings)
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    first_source = engine._run.source

    live = engine.settings
    live.preview_enabled = False
    live.health_rate_hz = 2.0
    live.source_tag = "dive01"
    engine.apply_settings(live)
    assert engine._run is not None and engine._run.source is first_source
    assert engine.settings.preview_enabled is False

    restart = engine.settings
    restart.fps = 10
    engine.apply_settings(restart)
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING and engine._run.source is not first_source,
                    timeout=8000)
    assert engine._run.settings.fps == 10

    stopped = engine.settings
    engine.stop()
    stopped.fps = 15
    engine.apply_settings(stopped)   # not running: stored, nothing started
    assert engine._run is None and engine.settings.fps == 15


def test_invalid_settings_are_refused(make_settings, make_engine):
    engine = make_engine(make_settings())
    bad = engine.settings
    bad.fps = 0
    engine.apply_settings(bad)
    assert engine.settings.fps == FPS


def test_set_preview_is_live_and_updates_settings(qtbot, make_settings, make_engine):
    engine = make_engine(make_settings())
    rec = Recorder(engine)
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING and bool(rec.previews), timeout=5000)
    engine.set_preview(False, False)
    assert engine.settings.preview_enabled is False
    qtbot.wait(150)   # frames already in flight
    count = len(rec.previews)
    qtbot.wait(500)
    assert len(rec.previews) == count


class FailingSource(PoseSource):
    """Reports an error from its own thread, as OakPoseSource does when the camera is busy."""

    def start(self):
        threading.Thread(target=self.callbacks.on_state, args=(STATE_ERROR, "camera busy"), daemon=True).start()

    def stop(self, timeout=5.0):
        pass

    def is_running(self):
        return False

    def stats(self):
        return SourceStats()


def test_source_error_surfaces_and_tears_down(qtbot, make_settings, make_engine):
    engine = make_engine(make_settings(), factory=lambda s, cb: FailingSource(s, cb))
    rec = Recorder(engine)
    with qtbot.waitSignal(engine.stateChanged, check_params_cb=lambda st, msg: st == STATE_ERROR, timeout=3000):
        engine.start()
    assert engine.state == STATE_ERROR and engine._run is None
    qtbot.waitUntil(lambda: ("error", "camera busy") in rec.logs, timeout=2000)
    assert engine.wait_idle(5.0)
    engine.start()   # can try again
    qtbot.waitUntil(lambda: engine.state == STATE_ERROR, timeout=3000)


def test_source_that_fails_to_construct(qtbot, make_settings, make_engine):
    def broken(settings, callbacks):
        raise RuntimeError("no depthai")
    engine = make_engine(make_settings(), factory=broken)
    rec = Recorder(engine)
    engine.start()
    assert engine.state == STATE_ERROR and "no depthai" in rec.states[-1][1]
    assert engine.wait_idle(5.0)


def test_loguru_reaches_log_signal_on_gui_thread(qtbot, make_settings, make_engine):
    engine = make_engine(make_settings())
    rec = Recorder(engine)
    worker = threading.Thread(target=lambda: logger.warning("from a worker thread"))
    worker.start()
    worker.join()
    logger.debug("debug is not forwarded")
    qtbot.waitUntil(lambda: ("warning", "from a worker thread") in rec.logs, timeout=2000)
    assert not any("debug is not forwarded" in text for _, text in rec.logs)
    assert rec.threads == {MAIN_THREAD}


def test_status_when_idle_has_every_key(make_settings, make_engine):
    status = make_engine(make_settings()).status()
    assert set(status) == STATUS_KEYS
    assert status["source"] is None and status["capture"] is None
    assert status["bandwidth"].total_mbit > 0


def test_default_factory_picks_the_source(make_settings):
    from trackshot.vio.oak_pipeline import OakPoseSource
    from trackshot.vio.source import SourceCallbacks
    assert isinstance(engine_module.default_source_factory(make_settings(source="sim"), SourceCallbacks()),
                      SimPoseSource)
    assert isinstance(engine_module.default_source_factory(make_settings(source="oak"), SourceCallbacks()),
                      OakPoseSource)


def test_autopilot_params_check_against_the_fake(fake, make_settings, make_engine):
    engine = make_engine(make_settings())
    rows = {r["name"]: r for r in engine.autopilot_params().check(suggested_viso_delay_ms=60)}
    assert rows["VISO_TYPE"]["ok"] is False
    assert rows["VISO_DELAY_MS"]["recommended"] == 60


def test_main_window_with_real_engine(qtbot, make_settings, make_engine, tmp_path):
    """The real window driven by the real engine: Start, poses on the trajectory, Stop."""
    from trackshot.ui.main_window import MainWindow
    QSettings.setDefaultFormat(QSettings.Format.IniFormat)
    QSettings.setPath(QSettings.Format.IniFormat, QSettings.Scope.UserScope, str(tmp_path / "qsettings"))
    try:
        engine = make_engine(make_settings())
        window = MainWindow(engine, settings_path=tmp_path / "settings.json")
        qtbot.addWidget(window)
        window.start_action.trigger()
        qtbot.waitUntil(lambda: engine.state == STATE_RUNNING and window.trajectory.point_count() > 5,
                        timeout=6000)
        assert not window.start_action.isEnabled() and window.stop_action.isEnabled()
        window.stop_action.trigger()
        assert window.start_action.isEnabled()
        window.close()
    finally:
        QSettings.setDefaultFormat(QSettings.Format.NativeFormat)


def test_shutdown_is_idempotent(make_settings, make_engine):
    engine = make_engine(make_settings())
    engine.set_recording(True)
    engine.shutdown()
    engine.shutdown()
    assert Path(engine.settings.capture_dir).parent.exists()


def test_simulator_never_sends_to_a_real_vehicle(qtbot, make_settings, make_engine):
    # 192.0.2.1 is TEST-NET-1: never routed, so nothing real is touched
    engine = make_engine(make_settings(vehicle_ip="192.0.2.1", udp_url="udpout:192.0.2.1:14550"))
    rec = Recorder(engine)
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    assert engine.status()["transport"] is None and engine._run.transport is None
    # The SITL check has to time out on the unroutable address before it gives up
    qtbot.waitUntil(lambda: any("not sending VISION_POSITION_DELTA" in t for _, t in rec.logs), timeout=15000)
    # poses flow (and the health timer ticks) without a transport, once tracking is established
    qtbot.waitUntil(lambda: engine.status()["odometry"].deltas > 0, timeout=5000)
    assert engine._run.transport is None


@pytest.mark.parametrize("is_sitl", [True, False])
def test_simulator_feeds_a_remote_autopilot_only_if_it_is_sitl(qtbot, fake, make_settings, make_engine,
                                                               monkeypatch, is_sitl):
    # Pretend the fake is a remote host, so the SITL check is what decides
    monkeypatch.setattr(engine_module, "is_loopback", lambda host: False)
    if is_sitl:
        fake.params[engine_module.SITL_MARKER_PARAM] = 1.0
    else:
        fake.params.pop(engine_module.SITL_MARKER_PARAM, None)
    engine = make_engine(make_settings(odometry_output="position_delta"))
    rec = Recorder(engine)
    engine.start()
    if is_sitl:
        qtbot.waitUntil(lambda: len(fake.received("VISION_POSITION_DELTA")) >= 3, timeout=10000)
        assert any("is an ArduPilot SITL" in t for _, t in rec.logs)
    else:
        qtbot.waitUntil(lambda: any("is not an ArduPilot SITL" in t for _, t in rec.logs), timeout=15000)
        qtbot.wait(500)
        assert engine._run.transport is None
        assert fake.received("VISION_POSITION_DELTA") == []


def test_hover_pattern_reports_a_stationary_vehicle():
    from trackshot.vio.sim import SimPoseSource
    from trackshot.vio.source import SourceCallbacks
    poses = []
    settings = Settings(source="sim", sim_pattern="hover", fps=10, preview_enabled=False)
    source = SimPoseSource(settings, SourceCallbacks(on_pose=poses.append), realtime=False, duration_s=30.0)
    source.start()
    source._thread.join(10.0)      # realtime=False ends by itself after duration_s
    source.stop()
    assert len(poses) > 250
    spread = max(abs(c) for p in poses for c in p.position)
    assert spread < 0.01, f"hover drifted {spread} m"


@pytest.mark.parametrize("transport, vehicle_ip, udp_url, host, loopback", [
    ("mavlink2rest", "127.0.0.1:8088", "", "127.0.0.1", True),
    ("mavlink2rest", "localhost", "", "localhost", True),
    ("mavlink2rest", "192.168.2.2", "", "192.168.2.2", False),
    ("udp", "127.0.0.1", "udpout:192.168.2.2:14550", "192.168.2.2", False),
    ("udp", "192.168.2.2", "udpout:127.0.0.1:14550", "127.0.0.1", True),
])
def test_transport_host(transport, vehicle_ip, udp_url, host, loopback):
    settings = Settings(transport=transport, vehicle_ip=vehicle_ip, udp_url=udp_url or Settings().udp_url)
    assert engine_module.transport_host(settings) == host
    assert engine_module.is_loopback(host) is loopback


# --- Frozen VIO and Restart VIO ---------------------------------------------------------------------

FREEZE_AT_S, FREEZE_S = 2.0, 3.0


def test_a_frozen_vio_stops_the_deltas_until_it_recovers(qtbot, fake, make_settings, make_engine):
    """The field failure: RTAB-Map lost, reporting its last pose over and over. Before, that went out
    as zero deltas at confidence 100; now nothing goes out until the VIO tracks again."""
    engine = make_engine(make_settings(odometry_output="position_delta"), sim_factory([{"at_s": FREEZE_AT_S, "event": "freeze",
                                                         "duration_s": FREEZE_S}]))
    rec = Recorder(engine)
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    run_until_sim_time(qtbot, engine, FREEZE_AT_S + FREEZE_S + 2.5)
    health = engine.status()["odometry"]
    engine.stop()
    assert engine.wait_idle(8.0)

    deltas = fake.received("VISION_POSITION_DELTA")
    assert len(deltas) > 40
    assert not any(m["position_delta"] == [0.0, 0.0, 0.0] for m in deltas), "a frozen pose went out"
    # One gap covering the freeze and the re-acquisition, nothing else missing
    times = sorted(m["_time"] for m in deltas)
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert FREEZE_S <= max(gaps) < FREEZE_S + 2.0
    assert sorted(gaps)[-2] < 0.5
    assert health.tracking == "good" and health.sending and health.losses == 1
    lost = [s for s in rec.statuses if s["odometry"] is not None and s["odometry"].tracking == "lost"]
    assert lost and not any(s["odometry"].sending for s in lost)
    texts = [t for _, t in rec.logs]
    assert any("VIO tracking lost (pose frozen" in t for t in texts)
    assert any("VIO tracking regained" in t for t in texts)
    # Cockpit saw the loss too
    track = [m["value"] for m in fake.received("NAMED_VALUE_FLOAT") if "".join(m["name"]).rstrip("\x00") == "TS_TRACK"]
    assert 0.0 in track and 1.0 in track


def test_restart_vio_reconnects_once_and_then_cools_down(qtbot, make_settings, make_engine, monkeypatch):
    engine = make_engine(make_settings())
    rec = Recorder(engine)
    assert engine.restart_vio_blocked() == "VIO isn't running"
    assert engine.restart_vio() is False
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    assert engine.restart_vio_blocked() is None
    first_run = engine._run

    assert engine.restart_vio() is True
    assert engine._run is not first_run
    # A second press straight away does nothing: one restart at a time, then the cooldown
    assert engine.restart_vio() is False
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=8000)
    blocked = engine.restart_vio_blocked()
    assert blocked is not None and "available again in" in blocked
    assert engine.restart_vio() is False
    assert sum("Restarting VIO (pilot request)" in t for _, t in rec.logs) == 1

    # After the cooldown it is available again
    monkeypatch.setattr(engine_module, "RESTART_VIO_COOLDOWN_S", 0.0)
    assert engine.restart_vio_blocked() is None


# --- Horizontal-only output (VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE) ---------------------

def test_horizontal_only_output_never_carries_vertical_motion(qtbot, fake, make_settings, make_engine):
    """The default output. The simulator flies forward at 0.3 m/s with the depth rising, so the VIO
    sees vertical motion; none of it may reach the autopilot, and the horizontal motion must come out
    in the autopilot's north/east frame (the fake heads 87.5°), anchored on the EKF's position."""
    engine = make_engine(make_settings())
    assert engine.settings.odometry_output == "position_speed"
    rec = Recorder(engine)
    engine.start()
    qtbot.waitUntil(lambda: engine.state == STATE_RUNNING, timeout=5000)
    qtbot.waitUntil(lambda: len(fake.received("VISION_SPEED_ESTIMATE")) >= 40, timeout=10000)
    engine.stop()
    assert engine.wait_idle(8.0)

    assert fake.received("VISION_POSITION_DELTA") == []
    speeds = fake.received("VISION_SPEED_ESTIMATE")
    positions = fake.received("VISION_POSITION_ESTIMATE")
    assert len(positions) == len(speeds)
    assert all(m["z"] == 0.0 for m in speeds), "vertical velocity reached the autopilot"
    assert all(m["z"] == 0.0 for m in positions)
    # Forward at 0.3 m/s along the fake's 87.5° heading: almost all east, a little north
    heading = math.radians(fake.vehicle.heading_deg)
    vn = statistics.median(m["x"] for m in speeds)
    ve = statistics.median(m["y"] for m in speeds)
    assert math.hypot(vn, ve) == pytest.approx(SPEED_MPS, rel=0.15)
    assert math.atan2(ve, vn) == pytest.approx(heading, abs=math.radians(3))
    # Anchored on the EKF's local position, then moving along the heading
    first, last = positions[0], positions[-1]
    assert (first["x"], first["y"]) == pytest.approx(fake.vehicle.local_ne, abs=0.05)
    moved = math.atan2(last["y"] - first["y"], last["x"] - first["x"])
    assert moved == pytest.approx(heading, abs=math.radians(3))
    assert all(m["_header"] == {"system_id": 1, "component_id": 197} for m in speeds)
    assert any("horizontal only" in t for _, t in rec.logs)


def test_a_run_writes_its_data_log(qtbot, fake, make_settings, make_engine, isolated_run_logs):
    import csv
    engine = make_engine(make_settings())
    engine.start()
    qtbot.waitUntil(lambda: len(fake.received("VISION_SPEED_ESTIMATE")) >= 10, timeout=10000)
    engine.stop()
    assert engine.wait_idle(8.0)
    files = list(isolated_run_logs.glob("run_*.csv"))
    assert len(files) == 1
    rows = list(csv.DictReader(open(files[0])))
    assert len(rows) > 20
    sent = [r for r in rows if r["sent"] == "estimate"]
    assert sent and all(r["ned_n"] and r["d_fwd"] and r["ap_heading_deg"] for r in sent)
    assert {r["tracking"] for r in rows} >= {"acquiring", "tracking"}


# --- Altitude (DISTANCE_SENSOR) and SURFTRAK ----------------------------------------------------------

def test_altitude_goes_out_as_a_downward_rangefinder_when_enabled(qtbot, fake, make_settings, make_engine):
    engine = make_engine(make_settings(send_distance_sensor=True, camera_angle_deg=90.0))
    engine.start()
    qtbot.waitUntil(lambda: len(fake.received("DISTANCE_SENSOR")) >= 10, timeout=10000)
    status = engine.status()["altitude"]
    engine.stop()
    assert engine.wait_idle(8.0)
    readings = fake.received("DISTANCE_SENSOR")
    # mavlink2rest's form: enums as {"type": NAME}, the `type` field as `mavtype`
    assert all(m["orientation"] == {"type": "MAV_SENSOR_ROTATION_PITCH_270"}
               and m["mavtype"] == {"type": "MAV_DISTANCE_SENSOR_UNKNOWN"}
               and m["min_distance"] == 30 and m["max_distance"] == 1000 for m in readings)
    assert all(m["_header"] == {"system_id": 1, "component_id": 197} for m in readings)
    # The simulator's seabed is 2.5 m below the start
    assert np.median([m["current_distance"] for m in readings]) == pytest.approx(250, abs=15)
    assert status["sending"] and status["distance_m"] == pytest.approx(2.5, abs=0.2)


@pytest.mark.parametrize("overrides", [dict(send_distance_sensor=False, camera_angle_deg=90.0),
                                       dict(send_distance_sensor=True, camera_angle_deg=0.0)])
def test_no_rangefinder_unless_enabled_and_looking_down(qtbot, fake, make_settings, make_engine, overrides):
    engine = make_engine(make_settings(**overrides))
    rec = Recorder(engine)
    engine.start()
    qtbot.waitUntil(lambda: engine.status()["altitude"] is not None, timeout=8000)
    qtbot.wait(1000)
    engine.stop()
    assert engine.wait_idle(8.0)
    assert fake.received("DISTANCE_SENSOR") == []
    if overrides["send_distance_sensor"]:
        assert any("Altitude not sent" in t for _, t in rec.logs)
