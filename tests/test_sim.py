"""SimPoseSource: determinism, timing, scripted faults, preview/capture rates, lifecycle."""

from __future__ import annotations

import io
import threading
import time

import numpy as np
import pytest
from PIL import Image

from trackshot.config import Settings
from trackshot.vio.sim import (
    LATENCY_MEAN_S,
    LATENCY_SPREAD_S,
    SIM_EPOCH_S,
    SPEED_MPS,
    SimPoseSource,
    survey_pose,
)
from trackshot.vio.source import (
    STATE_CONNECTING,
    STATE_RUNNING,
    STATE_STOPPED,
    SourceCallbacks,
)


class Recorder:
    """Collects everything a source reports; thread-safe enough for appends from one thread."""

    def __init__(self):
        self.poses, self.previews, self.captures, self.states, self.logs = [], [], [], [], []
        self.done = threading.Event()

    def callbacks(self) -> SourceCallbacks:
        def on_state(state, message):
            self.states.append(state)
            if state == STATE_STOPPED:
                self.done.set()

        return SourceCallbacks(
            on_pose=self.poses.append,
            on_preview=self.previews.append,
            on_capture=self.captures.append,
            on_state=on_state,
            on_log=lambda level, msg: self.logs.append((level, msg)),
        )


def settings(**overrides) -> Settings:
    s = Settings(source="sim", fps=10, width=320, height=200, preview_enabled=False,
                 color_preview_enabled=False, center_capture_fps=0.0, stereo_capture_fps=0.0)
    for k, v in overrides.items():
        setattr(s, k, v)
    return s


def run_sim(s: Settings, duration_s: float = 20.0, seed: int = 0, script=None) -> tuple[Recorder, SimPoseSource]:
    rec = Recorder()
    src = SimPoseSource(s, rec.callbacks(), seed=seed, realtime=False, script=script, duration_s=duration_s)
    src.start()
    assert rec.done.wait(30), "simulation did not finish"
    src.stop()
    return rec, src


def t_cap(sample) -> float:
    """Capture time in simulated seconds since the start."""
    return sample.t_capture_us / 1e6 - SIM_EPOCH_S


# --- Path -----------------------------------------------------------------------------------
def test_survey_path_is_continuous_and_starts_at_origin():
    p0, yaw0 = survey_pose(0.0)
    assert np.allclose(p0, 0.0) and yaw0 == 0.0
    dt = 0.01
    prev_p, prev_yaw = p0, yaw0
    for t in np.arange(dt, 200.0, dt):
        p, yaw = survey_pose(float(t))
        step = np.linalg.norm(p - prev_p)
        assert step < 1.5 * SPEED_MPS * dt + 1e-6        # constant speed, no jumps (plus depth)
        assert abs(yaw - prev_yaw) < 0.02                  # yaw turns smoothly
        prev_p, prev_yaw = p, yaw
    xs = [survey_pose(float(t))[0][0] for t in np.arange(0, 200, 0.5)]
    zs = [survey_pose(float(t))[0][2] for t in np.arange(0, 200, 0.5)]
    assert -1.1 <= min(xs) and max(xs) <= 11.1             # 10 m legs plus 1 m turn radius
    assert max(abs(z) for z in zs) <= 0.2 + 1e-9


# --- Determinism and timing -----------------------------------------------------------------
def test_deterministic_for_seed_and_fast():
    t0 = time.monotonic()
    a, _ = run_sim(settings(), duration_s=20.0, seed=7)
    elapsed = time.monotonic() - t0
    b, _ = run_sim(settings(), duration_s=20.0, seed=7)
    c, _ = run_sim(settings(), duration_s=20.0, seed=8)
    assert elapsed < 5.0
    assert a.poses == b.poses
    assert a.poses != c.poses
    assert a.states == [STATE_CONNECTING, STATE_RUNNING, STATE_STOPPED]


def test_pose_rate_matches_fps_by_timestamps():
    rec, src = run_sim(settings(fps=15), duration_s=20.0)
    caps = np.array([t_cap(s) for s in rec.poses])
    assert 295 <= len(caps) <= 301
    assert np.allclose(np.diff(caps), 1 / 15, atol=2e-6)
    rates = (len(caps) - 1) / (caps[-1] - caps[0])
    assert rates == pytest.approx(15.0, rel=0.01)
    stats = src.stats()
    assert stats.fps == pytest.approx(15.0, rel=0.1)
    assert stats.backend == "sim" and stats.camera_name == "Simulator"
    assert stats.rx_mbit is None and stats.chip_temp_c is None


def test_latency_within_bounds_and_receive_order():
    rec, _ = run_sim(settings(fps=30), duration_s=10.0)
    lat = np.array([s.t_received - s.t_capture_us / 1e6 for s in rec.poses])
    assert lat.min() >= LATENCY_MEAN_S - LATENCY_SPREAD_S - 1e-6
    assert lat.max() <= LATENCY_MEAN_S + LATENCY_SPREAD_S + 1e-6
    assert lat.mean() == pytest.approx(LATENCY_MEAN_S, abs=0.005)
    assert lat.std() > 0.002                                   # it does vary
    received = [s.t_received for s in rec.poses]
    assert received == sorted(received)


def test_pose_noise_is_small_and_quaternions_unit():
    rec, _ = run_sim(settings(), duration_s=10.0)
    for s in rec.poses:
        true_p, _ = survey_pose(t_cap(s))
        assert np.linalg.norm(np.array(s.position) - true_p) < 0.01
        assert np.linalg.norm(s.quaternion) == pytest.approx(1.0, abs=1e-9)
        assert s.tracking is True


# --- Script ---------------------------------------------------------------------------------
def test_scripted_jump_dropout_reset_lost():
    script = [
        {"at_s": 5.0, "event": "jump", "offset": [2, 0, 0]},
        {"at_s": 8, "event": "dropout", "duration_s": 3},
        {"at_s": 12, "event": "reset"},
        {"at_s": 15, "event": "lost", "duration_s": 2},
    ]
    rec, _ = run_sim(settings(), duration_s=20.0, script=script)
    by_t = {round(t_cap(s), 3): s for s in rec.poses}

    # Jump: a one-sample 2 m step that persists
    before, after = by_t[4.9], by_t[5.0]
    assert after.position[0] - before.position[0] == pytest.approx(2.0 + SPEED_MPS * 0.1, abs=0.01)
    assert by_t[6.0].position[0] == pytest.approx(survey_pose(6.0)[0][0] + 2.0, abs=0.01)

    # Dropout: nothing captured in [8, 11)
    times = sorted(by_t)
    assert not [t for t in times if 8.0 <= t < 11.0]
    assert 7.9 in by_t and 11.0 in by_t

    # Reset: exactly the origin, then continues from there
    reset = by_t[12.0]
    assert reset.position == (0.0, 0.0, 0.0) and reset.quaternion == (1.0, 0.0, 0.0, 0.0)
    assert np.linalg.norm(by_t[12.1].position) == pytest.approx(SPEED_MPS * 0.1, abs=0.02)

    # Lost: tracking False exactly in [15, 17)
    lost = [t for t, s in by_t.items() if s.tracking is False]
    assert min(lost) == 15.0 and max(lost) == pytest.approx(16.9)
    assert len(lost) == 20


def test_bad_script_is_rejected():
    with pytest.raises(ValueError):
        SimPoseSource(settings(), SourceCallbacks(), realtime=False, duration_s=1, script=[{"at_s": 1, "event": "boom"}])
    with pytest.raises(ValueError):
        SimPoseSource(settings(), SourceCallbacks(), realtime=False, duration_s=1, script=[{"at_s": 1, "event": "dropout"}])
    with pytest.raises(ValueError):
        SimPoseSource(settings(), SourceCallbacks(), realtime=False)   # would never end


# --- Preview --------------------------------------------------------------------------------
def test_vio_preview_rate_size_and_motion():
    rec, _ = run_sim(settings(fps=20, preview_enabled=True), duration_s=10.0)
    vio = [f for f in rec.previews if f.kind == "vio"]
    assert len(vio) == pytest.approx(100, abs=2)                 # throttled to 10 Hz from 20 fps
    gaps = np.diff([f.t_received for f in vio])
    assert gaps.min() >= 0.1 - 0.031                              # latency jitter only
    f0, f1 = vio[0], vio[-1]
    assert f0.image.shape == (200, 320) and f0.image.dtype == np.uint8
    assert f0.features.shape == (100, 2)
    assert not np.array_equal(f0.image, f1.image)                 # moving texture
    assert (f0.features[:, 0] < 320).all() and (f0.features[:, 1] < 200).all()
    assert not [f for f in rec.previews if f.kind == "color"]


def test_color_preview_and_live_toggle():
    rec = Recorder()
    s = settings(preview_enabled=True, color_preview_enabled=True, color_preview_fps=5.0,
                 color_preview_width=160, color_preview_height=90)
    src = SimPoseSource(s, rec.callbacks(), realtime=False, duration_s=10.0)
    toggled = threading.Event()

    def on_pose(sample):
        # Switch every preview off live, halfway through
        if not toggled.is_set() and t_cap(sample) >= 5.0:
            src.set_preview(False, False)
            toggled.set()

    rec_pose = src.callbacks.on_pose
    src.callbacks.on_pose = lambda smp: (rec_pose(smp), on_pose(smp))
    src.start()
    assert rec.done.wait(30)
    color = [f for f in rec.previews if f.kind == "color"]
    vio = [f for f in rec.previews if f.kind == "vio"]
    assert color[0].image.shape == (90, 160, 3)
    assert len(color) == pytest.approx(25, abs=2)                # 5 Hz for the first 5 s
    assert max(f.t_received for f in rec.previews) - SIM_EPOCH_S < 5.2
    assert len(vio) == pytest.approx(50, abs=2)


def test_preview_off_means_no_frames():
    rec, _ = run_sim(settings(preview_enabled=False, color_preview_enabled=True), duration_s=5.0)
    assert rec.previews == []


# --- Capture --------------------------------------------------------------------------------
def test_capture_rates_sizes_and_pairing():
    s = settings(fps=10, center_capture_fps=1.0, stereo_capture_fps=2.0,
                 center_width=640, center_height=360, mono_width=320, mono_height=200, jpeg_quality=80)
    rec, _ = run_sim(s, duration_s=10.0)
    center = [c for c in rec.captures if c.stream == "center"]
    left = [c for c in rec.captures if c.stream == "left"]
    right = [c for c in rec.captures if c.stream == "right"]
    assert len(center) == pytest.approx(10, abs=1)
    assert len(left) == len(right) == pytest.approx(20, abs=1)
    assert [c.sequence for c in left] == [c.sequence for c in right]
    assert all(lc.t_capture == rc.t_capture for lc, rc in zip(left, right))
    assert [c.sequence for c in center] == list(range(len(center)))
    assert all(c.lens is None for c in rec.captures)
    with Image.open(io.BytesIO(center[0].data)) as im:
        assert im.format == "JPEG" and im.size == (640, 360) and im.mode == "RGB"
    with Image.open(io.BytesIO(left[0].data)) as im:
        assert im.size == (320, 200) and im.mode == "L"


def test_stereo_capture_capped_at_vio_fps():
    s = settings(fps=5, stereo_capture_fps=25.0, mono_width=160, mono_height=100)
    rec, _ = run_sim(s, duration_s=4.0)
    left = [c for c in rec.captures if c.stream == "left"]
    assert len(left) == pytest.approx(20, abs=1)


# --- Lifecycle ------------------------------------------------------------------------------
def test_stop_is_idempotent_and_safe_before_start():
    src = SimPoseSource(settings(), SourceCallbacks())
    src.stop()                     # before start
    assert not src.is_running()
    src.start()
    src.stop(timeout=2.0)
    src.stop(timeout=2.0)
    assert not src.is_running()


def test_callback_exception_does_not_kill_source():
    rec = Recorder()
    cbs = rec.callbacks()

    def bad(sample):
        raise RuntimeError("consumer bug")

    cbs.on_pose = bad
    src = SimPoseSource(settings(preview_enabled=True), cbs, realtime=False, duration_s=2.0)
    src.start()
    assert rec.done.wait(10)
    assert len(rec.previews) > 10
    assert rec.states[-1] == STATE_STOPPED


def test_realtime_smoke():
    rec = Recorder()
    src = SimPoseSource(settings(fps=20, preview_enabled=True), rec.callbacks(), realtime=True)
    t0 = time.time()
    src.start()
    assert time.time() - t0 < 0.1                               # start() doesn't block
    deadline = time.time() + 1.2
    while time.time() < deadline and len(rec.poses) < 10:
        time.sleep(0.02)
    assert src.is_running()
    stopping = time.time()
    src.stop(timeout=2.0)
    assert time.time() - stopping < 0.5
    assert not src.is_running()
    assert rec.states[:2] == [STATE_CONNECTING, STATE_RUNNING] and rec.states[-1] == STATE_STOPPED
    assert rec.states.count(STATE_STOPPED) == 1
    assert len(rec.poses) >= 10
    for s in rec.poses:
        latency = s.t_received - s.t_capture_us / 1e6
        assert 0.04 < latency < 0.2                              # real clock: allow scheduling slack
        assert abs(s.t_received - time.time()) < 5
    assert src.stats().fps > 5
