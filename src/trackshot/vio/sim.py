"""SimPoseSource: a synthetic ROV, so the whole app can be exercised without a camera or vehicle.

The simulated vehicle flies a lawnmower survey (10 m legs, 2 m lane spacing, 0.3 m/s, smooth
180° yaw turns at the leg ends, ±0.2 m depth oscillation) in the VIO world frame (FLU, x forward,
y left, z up), starting at the origin with identity attitude like a freshly started VIO.

It behaves like OakPoseSource from the engine's point of view:
- poses at `settings.fps`, each captured `60 ± 15 ms` before it reaches the host, with mm-level
  noise, so the odometry's latency measurement and VISO_DELAY_MS suggestion have something real
  to chew on;
- a grey VIO preview (a seeded periodic texture scrolled by the vehicle's position, with ~100 fake
  features) and an optional RGB colour preview, both ≤ 10 Hz;
- survey JPEGs (Pillow-encoded) at the centre/stereo capture rates, left/right sharing a sequence
  number as the OAK's paired mono frames do.

A `script` injects the faults the odometry chain must survive — VIO relocalisation jumps, pose
dropouts, VIO resets to the origin, and lost tracking — at given simulated times. A dropout stops
poses and the VIO preview (they are one stream on the camera); survey capture carries on.

`realtime=False` runs the same timeline as fast as possible on a simulated clock that starts at a
fixed epoch, so tests get identical samples for the same seed and finish in milliseconds.
"""

from __future__ import annotations

import heapq
import io
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Optional

import numpy as np
from loguru import logger
from PIL import Image

from trackshot.vio.source import (
    STATE_CONNECTING,
    STATE_ERROR,
    STATE_RUNNING,
    STATE_STOPPED,
    CapturedJpeg,
    PoseSample,
    PoseSource,
    PreviewFrame,
    SourceCallbacks,
    SourceStats,
)

# --- Survey path ----------------------------------------------------------------------------
LEG_M = 10.0                 # straight leg length along ±x
LANE_SPACING_M = 2.0         # distance between legs along +y (each turn is a semicircle of r = spacing/2)
SPEED_MPS = 0.3
DEPTH_AMPLITUDE_M = 0.2
DEPTH_PERIOD_S = 20.0

# --- Sensor imperfections -------------------------------------------------------------------
POSITION_NOISE_M = 0.001     # 1 mm σ: what a well-tracking VIO jitters by frame to frame
ATTITUDE_NOISE_RAD = 0.0005
LATENCY_MEAN_S = 0.060       # capture→host, what an OAK over the tether shows with host-side VIO
LATENCY_SIGMA_S = 0.0075
LATENCY_SPREAD_S = 0.015     # hard clamp: latency stays within mean ± spread
CAPTURE_TRANSFER_S = 0.080   # survey JPEGs are bigger than VIO frames, so they arrive a bit later
CONNECT_DELAY_S = 0.3        # realtime only: long enough for the UI to show "connecting"

# --- Preview --------------------------------------------------------------------------------
PREVIEW_MAX_HZ = 10.0        # the engine throttles to 10 Hz anyway; don't make frames it will drop
FEATURE_COUNT = 100
PIXELS_PER_METRE_PER_WIDTH = 0.6   # scroll speed: 0.6 image widths per metre travelled
STEREO_DISPARITY_FRACTION = 0.03   # right image offset, so a stereo pair isn't two identical images
MAX_TEXTURE_WIDTH = 480      # big capture images upscale a smaller tile: cheap to scroll and encode

# Simulated wall clock for realtime=False: 2026-01-01T00:00:00Z. A plausible epoch time keeps
# anything that formats timestamps (capture filenames, logs) behaving as with a real camera.
SIM_EPOCH_S = 1_767_225_600.0

FPS_WINDOW_S = 2.0
_EPS = 1e-9

SCRIPT_EVENTS = ("jump", "dropout", "reset", "lost")


@dataclass(frozen=True)
class ScriptEvent:
    at_s: float
    event: str
    duration_s: float = 0.0
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)


def parse_script(script: Optional[list[dict]]) -> list[ScriptEvent]:
    """Validate script dicts up front so a typo fails at construction, not mid-run."""
    events = []
    for item in script or []:
        kind = item.get("event")
        if kind not in SCRIPT_EVENTS:
            raise ValueError(f"Unknown sim script event {kind!r}; expected one of {SCRIPT_EVENTS}")
        if "at_s" not in item:
            raise ValueError(f"Sim script event {item!r} has no 'at_s'")
        duration = float(item.get("duration_s", 0.0))
        if kind in ("dropout", "lost") and duration <= 0:
            raise ValueError(f"Sim script {kind} at {item['at_s']} s needs a positive 'duration_s'")
        offset = tuple(float(v) for v in item.get("offset", (0.0, 0.0, 0.0)))
        if len(offset) != 3:
            raise ValueError(f"Sim script jump offset must be [x, y, z], got {item.get('offset')!r}")
        events.append(ScriptEvent(float(item["at_s"]), kind, duration, offset))  # type: ignore[arg-type]
    return sorted(events, key=lambda e: e.at_s)


# --- Geometry -------------------------------------------------------------------------------
def hover_pose(t: float) -> tuple[np.ndarray, float]:
    """A vehicle holding station: no motion beyond the sensor noise added later. This is what a
    stationary camera reports, and what a POSHOLD test against SITL should see."""
    return np.zeros(3), 0.0


def pattern_pose(pattern: str):
    """The true-pose function for a Settings.sim_pattern."""
    return hover_pose if pattern == "hover" else survey_pose


def survey_pose(t: float) -> tuple[np.ndarray, float]:
    """True vehicle position (FLU, m) and yaw (rad) at `t` seconds into the survey.

    One cycle is: leg along +x, left semicircle turn up one lane, leg back along −x, right
    semicircle turn up the next lane. Position and yaw are continuous, and the speed is constant,
    so frame-to-frame deltas never look like a jump unless the script adds one.
    """
    leg_t = LEG_M / SPEED_MPS
    r = LANE_SPACING_M / 2.0
    turn_t = math.pi * r / SPEED_MPS
    cycle_t = 2.0 * (leg_t + turn_t)
    n, s = divmod(t, cycle_t)
    y0 = n * 2.0 * LANE_SPACING_M
    if s < leg_t:                                   # out along +x
        x, y, yaw = SPEED_MPS * s, y0, 0.0
    elif s < leg_t + turn_t:                        # left turn: yaw 0 → π
        theta = SPEED_MPS * (s - leg_t) / r
        x, y, yaw = LEG_M + r * math.sin(theta), y0 + r - r * math.cos(theta), theta
    elif s < 2.0 * leg_t + turn_t:                  # back along −x
        x, y, yaw = LEG_M - SPEED_MPS * (s - leg_t - turn_t), y0 + LANE_SPACING_M, math.pi
    else:                                           # right turn: yaw π → 0
        theta = SPEED_MPS * (s - 2.0 * leg_t - turn_t) / r
        x = -r * math.sin(theta)
        y = y0 + LANE_SPACING_M + r - r * math.cos(theta)
        yaw = math.pi - theta
    z = DEPTH_AMPLITUDE_M * math.sin(2.0 * math.pi * t / DEPTH_PERIOD_S)
    return np.array([x, y, z]), yaw


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    return quat_mul(quat_mul(q, np.array([0.0, *v])), quat_conj(q))[1:]


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


# --- Synthetic images -----------------------------------------------------------------------
def periodic_texture(height: int, width: int, rng: np.random.Generator, blur_px: float = 2.0) -> np.ndarray:
    """Seamlessly tiling grey texture (uint8): white noise low-passed in the frequency domain.

    Being periodic, it can be scrolled with np.roll indefinitely without a visible seam, which is
    what makes the vehicle's motion readable in the preview.
    """
    noise = rng.standard_normal((height, width))
    fy = np.fft.fftfreq(height)[:, None]
    fx = np.fft.rfftfreq(width)[None, :]
    gain = np.exp(-2.0 * (math.pi * blur_px) ** 2 * (fx ** 2 + fy ** 2))
    smooth = np.fft.irfft2(np.fft.rfft2(noise) * gain, s=(height, width))
    smooth -= smooth.min()
    smooth /= max(float(smooth.max()), _EPS)
    return (40.0 + 180.0 * smooth).astype(np.uint8)


class _Scene:
    """A synthetic camera view of a given size that scrolls with the vehicle's position."""

    def __init__(self, width: int, height: int, rng: np.random.Generator, color: bool):
        self.width, self.height, self.color = width, height, color
        self.scale = max(1, math.ceil(width / MAX_TEXTURE_WIDTH))
        tile = periodic_texture(max(8, height // self.scale), max(8, width // self.scale), rng)
        if color:
            # Blue-green seabed tint: water absorbs red first
            g = tile.astype(np.float32)
            tile = np.clip(np.stack([0.45 * g + 10, 0.8 * g + 30, 0.75 * g + 45], axis=-1), 0, 255).astype(np.uint8)
        self.tile = tile
        self.pixels_per_metre = PIXELS_PER_METRE_PER_WIDTH * width

    def shift_px(self, x_m: float, y_m: float) -> tuple[int, int]:
        """Image shift (dx, dy) in output pixels for a vehicle at (x, y), in whole tile pixels so
        feature positions computed from it match the rendered image exactly."""
        q = self.scale
        return int(round(-x_m * self.pixels_per_metre / q)) * q, int(round(y_m * self.pixels_per_metre / q)) * q

    def render(self, x_m: float, y_m: float, extra_dx_px: int = 0) -> np.ndarray:
        dx, dy = self.shift_px(x_m, y_m)
        dx += extra_dx_px
        img = np.roll(self.tile, (dy // self.scale, dx // self.scale), axis=(0, 1))
        if self.scale > 1 or img.shape[:2] != (self.height, self.width):
            img = np.asarray(Image.fromarray(img).resize((self.width, self.height), Image.BILINEAR))
        return img


def encode_jpeg(image: np.ndarray, quality: int) -> bytes:
    buf = io.BytesIO()
    Image.fromarray(image).save(buf, format="JPEG", quality=int(quality))
    return buf.getvalue()


# --- The source -----------------------------------------------------------------------------
class SimPoseSource(PoseSource):
    """Deterministic synthetic pose/preview/capture source. See the module docstring."""

    def __init__(self, settings, callbacks: SourceCallbacks, seed: int = 0, realtime: bool = True,
                 script: Optional[list[dict]] = None, duration_s: Optional[float] = None):
        super().__init__(settings, callbacks)
        if not realtime and duration_s is None:
            # As fast as possible with no end would spin a core forever
            raise ValueError("SimPoseSource(realtime=False) needs duration_s")
        self.seed = int(seed)
        self.realtime = realtime
        self.duration_s = duration_s
        self.script = parse_script(script)
        self._true_pose = pattern_pose(getattr(settings, "sim_pattern", "survey"))
        self._preview_vio = bool(settings.preview_enabled)
        # Same rule as the bandwidth estimate: the colour stream is part of the preview
        self._preview_color = bool(settings.preview_enabled and settings.color_preview_enabled)
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._pose_times: deque[float] = deque()
        self._poses = 0
        self._sim_t = 0.0
        self._wall_t0 = 0.0

    # --- PoseSource API -----------------------------------------------------------------------
    def start(self) -> None:
        if self.is_running():
            logger.warning("Simulator already running")
            return
        self._stop_event.clear()
        with self._lock:
            self._pose_times.clear()
            self._poses = 0
            self._sim_t = 0.0
            self._wall_t0 = 0.0
        self._thread = threading.Thread(target=self._run, name="trackshot-sim", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout)
            if thread.is_alive():
                logger.warning(f"Simulator thread did not stop within {timeout} s")

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive() and not self._stop_event.is_set()

    def stats(self) -> SourceStats:
        with self._lock:
            now = time.time() if self.realtime else self._wall_t0 + self._sim_t
            while self._pose_times and self._pose_times[0] < now - FPS_WINDOW_S:
                self._pose_times.popleft()
            window = min(FPS_WINDOW_S, max(now - self._wall_t0, _EPS)) if self._wall_t0 else 0.0
            fps = len(self._pose_times) / window if window > 0 else 0.0
            return SourceStats(
                fps=fps, rx_mbit=None, chip_temp_c=None, camera_name="Simulator", camera_id="sim",
                imu="simulated", backend="sim",
                extra={"seed": self.seed, "sim_time_s": self._sim_t, "poses": self._poses,
                       "realtime": self.realtime},
            )

    def set_preview(self, vio: bool, color: bool) -> None:
        # Plain bool assignments are atomic; the sim thread reads them per frame
        self._preview_vio = bool(vio)
        self._preview_color = bool(color)

    # --- Thread -------------------------------------------------------------------------------
    def _run(self) -> None:
        failed = False
        try:
            self._emit_state(STATE_CONNECTING, "Starting the simulator")
            self._setup()
            if self.realtime and self._stop_event.wait(CONNECT_DELAY_S):
                return
            self._wall_t0 = time.time() if self.realtime else SIM_EPOCH_S
            s = self.settings
            self._emit_state(STATE_RUNNING, f"Simulator running: {s.width}x{s.height} at {s.fps} fps, seed {self.seed}")
            motion = ("stationary hover" if getattr(s, "sim_pattern", "survey") == "hover"
                      else "lawnmower survey")
            self._log("info", f"Simulated {motion}, seed {self.seed}"
                      + (f", script {[e.event for e in self.script]}" if self.script else ""))
            self._loop()
        except Exception as e:  # noqa: BLE001 - any failure must reach the UI as an error state
            failed = True
            logger.exception("Simulator failed")
            self._emit_state(STATE_ERROR, f"Simulator failed: {e}")
        finally:
            if not failed:
                self._emit_state(STATE_STOPPED, "Simulator stopped")

    def _setup(self) -> None:
        """Per-run state. Independent RNG streams per purpose, so toggling the preview or
        capture never changes the pose sequence for a seed."""
        s = self.settings
        pose_ss, latency_ss, texture_ss, feature_ss = np.random.SeedSequence(self.seed).spawn(4)
        self._pose_rng = np.random.default_rng(pose_ss)
        self._latency_rng = np.random.default_rng(latency_ss)
        self._texture_seeds = dict(zip(("vio", "color", "center", "mono"), texture_ss.spawn(4)))
        self._scenes: dict[str, _Scene] = {}
        self._features = np.random.default_rng(feature_ss).uniform(
            (0.0, 0.0), (s.width, s.height), size=(FEATURE_COUNT, 2))

        self._heap: list[tuple[float, int, Callable[..., None], tuple]] = []
        self._order = 0
        self._q_off = np.array([1.0, 0.0, 0.0, 0.0])   # reported = off ∘ true (VIO's own world frame)
        self._p_off = np.zeros(3)
        self._pending = list(self.script)             # jump/reset not yet applied
        self._last_received = -math.inf
        self._last_preview_t = -math.inf
        self._next_stereo_t = 0.0
        self._center_seq = 0

        self._frame_period = 1.0 / s.fps
        self._stereo_period = 1.0 / s.effective_stereo_capture_fps() if s.effective_stereo_capture_fps() > 0 else None
        self._schedule(0.0, self._on_frame, 0)
        if s.center_capture_fps > 0:
            self._schedule(0.0, self._on_center_capture)
        if s.color_preview_fps > 0:
            self._schedule(0.0, self._on_color_preview)

    def _scene(self, kind: str) -> _Scene:
        """Scenes are built on first use (a colour preview may never be turned on)."""
        if kind not in self._scenes:
            s = self.settings
            size = {"vio": (s.width, s.height), "color": (s.color_preview_width, s.color_preview_height),
                    "center": (s.center_width, s.center_height), "mono": (s.mono_width, s.mono_height)}[kind]
            rng = np.random.default_rng(self._texture_seeds[kind])
            self._scenes[kind] = _Scene(*size, rng, color=kind in ("color", "center"))
        return self._scenes[kind]

    def _schedule(self, due_s: float, fn: Callable[..., None], *args: Any) -> None:
        self._order += 1   # tie-breaker: same-time events run in the order they were scheduled
        heapq.heappush(self._heap, (due_s, self._order, fn, args))

    def _loop(self) -> None:
        while self._heap and not self._stop_event.is_set():
            due, _, fn, args = heapq.heappop(self._heap)
            if self.duration_s is not None and due > self.duration_s:
                break
            if self.realtime:
                delay = self._wall_t0 + due - time.time()
                if delay > 0 and self._stop_event.wait(delay):
                    break
            with self._lock:
                self._sim_t = due
            fn(due, *args)

    def _now_wall(self, sim_t: float) -> float:
        """Arrival time stamp: the real clock when realtime, else the simulated one."""
        return time.time() if self.realtime else self._wall_t0 + sim_t

    # --- Event handlers (all on the sim thread) -------------------------------------------------
    def _on_frame(self, t: float, index: int) -> None:
        """One camera frame at capture time t: pose (+ preview) unless dropped out, stereo capture."""
        self._schedule((index + 1) * self._frame_period, self._on_frame, index + 1)
        true_p, yaw = self._true_pose(t)
        true_q = yaw_quat(yaw)
        snapped = self._apply_script_steps(t, true_p, true_q)

        p = quat_rotate(self._q_off, true_p) + self._p_off
        q = quat_mul(self._q_off, true_q)
        if snapped:
            # A VIO reset reports exactly the origin, which is what the odometry checks for
            p, q = np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
        else:
            p = p + self._pose_rng.normal(0.0, POSITION_NOISE_M, 3)
            half = 0.5 * self._pose_rng.normal(0.0, ATTITUDE_NOISE_RAD, 3)
            q = quat_mul(q, np.array([1.0, *half]))
            q /= np.linalg.norm(q)

        latency = LATENCY_MEAN_S + float(np.clip(self._latency_rng.normal(0.0, LATENCY_SIGMA_S),
                                                 -LATENCY_SPREAD_S, LATENCY_SPREAD_S))
        # Frames arrive in order: at fps > ~30 a short latency must not overtake the frame before
        received = max(t + latency, self._last_received)
        self._last_received = received

        if not self._in_window(t, "dropout"):
            sample_args = (t, tuple(float(v) for v in p), tuple(float(v) for v in q),
                           not self._in_window(t, "lost"))
            self._schedule(received, self._deliver_pose, *sample_args)
            if self._preview_vio and t - self._last_preview_t >= 1.0 / PREVIEW_MAX_HZ - _EPS:
                self._last_preview_t = t
                self._schedule(received, self._deliver_vio_preview, true_p.copy(), sample_args[3])

        if self._stereo_period is not None and t >= self._next_stereo_t - _EPS:
            # Stereo pairs come from the VIO cameras: they share the frame's sequence and capture time
            self._next_stereo_t += self._stereo_period
            self._schedule(t + CAPTURE_TRANSFER_S, self._deliver_stereo, t, index, true_p.copy())

    def _apply_script_steps(self, t: float, true_p: np.ndarray, true_q: np.ndarray) -> bool:
        """Apply jump/reset events now due. Returns True if a reset happened on this frame."""
        snapped = False
        while self._pending and self._pending[0].at_s <= t + _EPS:
            event = self._pending.pop(0)
            if event.event == "jump":
                self._p_off = self._p_off + np.array(event.offset)
                self._log("info", f"Sim: VIO jump by {list(event.offset)} m at {t:.2f} s")
            elif event.event == "reset":
                # Re-anchor the reported world frame at the vehicle's current pose
                self._q_off = quat_conj(true_q)
                self._p_off = -quat_rotate(self._q_off, true_p)
                snapped = True
                self._log("info", f"Sim: VIO reset to the origin at {t:.2f} s")
        return snapped

    def _in_window(self, t: float, kind: str) -> bool:
        return any(e.event == kind and e.at_s - _EPS <= t < e.at_s + e.duration_s - _EPS for e in self.script)

    def _deliver_pose(self, due: float, t_capture: float, position: tuple, quaternion: tuple, tracking: bool) -> None:
        received = self._now_wall(due)
        sample = PoseSample(
            t_capture_us=int(round((self._wall_t0 + t_capture) * 1e6)),
            t_received=received,
            position=position,
            quaternion=quaternion,
            tracking=tracking,
        )
        with self._lock:
            self._pose_times.append(received)
            self._poses += 1
        self._call(self.callbacks.on_pose, sample)

    def _deliver_vio_preview(self, due: float, true_p: np.ndarray, tracking: bool) -> None:
        if not self._preview_vio:
            return
        scene = self._scene("vio")
        image = scene.render(true_p[0], true_p[1])
        if tracking:
            dx, dy = scene.shift_px(true_p[0], true_p[1])
            features = np.mod(self._features + (dx, dy), (scene.width, scene.height))
        else:
            features = np.empty((0, 2))   # a lost VIO has nothing to track
        self._call(self.callbacks.on_preview, PreviewFrame("vio", image, self._now_wall(due), features))

    def _on_color_preview(self, t: float) -> None:
        self._schedule(t + 1.0 / min(self.settings.color_preview_fps, PREVIEW_MAX_HZ), self._on_color_preview)
        if not self._preview_color:
            return
        true_p, _ = self._true_pose(t)
        image = self._scene("color").render(true_p[0], true_p[1])
        self._call(self.callbacks.on_preview, PreviewFrame("color", image, self._now_wall(t)))

    def _on_center_capture(self, t: float) -> None:
        self._schedule(t + 1.0 / self.settings.center_capture_fps, self._on_center_capture)
        self._schedule(t + CAPTURE_TRANSFER_S, self._deliver_center, t, self._center_seq, self._true_pose(t)[0])
        self._center_seq += 1

    def _deliver_center(self, due: float, t_capture: float, sequence: int, true_p: np.ndarray) -> None:
        data = encode_jpeg(self._scene("center").render(true_p[0], true_p[1]), self.settings.jpeg_quality)
        self._call(self.callbacks.on_capture, CapturedJpeg("center", data, self._wall_t0 + t_capture, sequence))

    def _deliver_stereo(self, due: float, t_capture: float, sequence: int, true_p: np.ndarray) -> None:
        scene = self._scene("mono")
        disparity = int(STEREO_DISPARITY_FRACTION * scene.width)
        for stream, extra in (("left", 0), ("right", -disparity)):
            data = encode_jpeg(scene.render(true_p[0], true_p[1], extra), self.settings.jpeg_quality)
            self._call(self.callbacks.on_capture, CapturedJpeg(stream, data, self._wall_t0 + t_capture, sequence))

    # --- Reporting ----------------------------------------------------------------------------
    def _call(self, fn: Callable[[Any], None], arg: Any) -> None:
        """A failing consumer callback is logged, not fatal: one bad frame shouldn't end a dive."""
        try:
            fn(arg)
        except Exception:  # noqa: BLE001
            logger.exception("Simulator callback raised")

    def _emit_state(self, state: str, message: str) -> None:
        logger.info(f"Simulator state: {state} ({message})")
        try:
            self.callbacks.on_state(state, message)
        except Exception:  # noqa: BLE001
            logger.exception("on_state callback raised")

    def _log(self, level: str, message: str) -> None:
        logger.log(level.upper(), message)
        try:
            self.callbacks.on_log(level, message)
        except Exception:  # noqa: BLE001
            logger.exception("on_log callback raised")
