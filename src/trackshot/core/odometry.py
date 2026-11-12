"""VIO pose → VISION_POSITION_DELTA math, ported from the BlueOS extension's `app/mavlink_node.py`.

Pure Python (no depthai, no Qt) so it can be tested by hand-derived golden values and fed by any
`PoseSource`. One `OdometryProcessor` is fed every pose from the source's thread; `health()` is read
from the GUI's stats timer, so all state sits behind one lock.

Frames (see docs/ARCHITECTURE.md):
- the VIO world frame is FLU (x forward, y left, z up) and quaternions are (qw, qx, qy, qz),
  body→world;
- ArduPilot wants `position_delta` and `angle_delta` in the vehicle body frame, FRD
  (x forward, y right, z down).
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

from loguru import logger

from trackshot.vio.source import PoseSample

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]   # (qw, qx, qy, qz)

# Latency EMA weight: the extension's 0.9/0.1 smoothing, steady enough to suggest VISO_DELAY_MS.
# The delta spans are smoothed the same way.
LATENCY_EMA_ALPHA = 0.1
# Host → autopilot transit of a queued VISION_POSITION_DELTA (sender thread, mavlink2rest or UDP,
# BlueOS's serial link to the flight controller): a few milliseconds, not measured
HOST_TO_AUTOPILOT_MS = 5.0
# A capture→host latency outside this is an unset or bogus timestamp, not a measurement
MAX_PLAUSIBLE_LATENCY_S = 5.0
# Deltas shorter than this would divide noise by almost nothing (the extension's 1 ms floor)
MIN_DELTA_SPAN_US = 1000
# ArduPilot's VISO_DELAY_MS range
VISO_DELAY_MIN_MS = 0
VISO_DELAY_MAX_MS = 250
FPS_WINDOW_S = 2.0
STALL_AFTER_S = 1.0
# "degraded" when the VIO keeps jumping: this many rejected jumps inside the window
DEGRADED_JUMPS = 3
DEGRADED_WINDOW_S = 10.0
JUMP_LOG_INTERVAL_S = 5.0
# A frozen VIO: RTAB-Map, once it has lost tracking, keeps reporting its last pose bit for bit, and
# sent as deltas that is "not moving" at full confidence, which ArduSub fuses (it doesn't gate on
# confidence) while the vehicle drifts. Real VIO output always carries noise, even with the vehicle
# still, so an exactly repeated pose is never a measurement. This long a run of repeats (or of
# invalid poses) is "lost": nothing is sent until tracking is established again.
FROZEN_AFTER_S = 1.0
FROZEN_MIN_REPEATS = 5
# Tracking is "established" (deltas are sent) after this many consecutive distinct, valid poses
# spanning at least this long: at start-up and after every loss. A VIO started before the camera
# sees the seabed sends nothing until it has something to track.
ACQUIRE_POSES = 10
ACQUIRE_MIN_S = 0.5
# Lost/regained messages at most this often; the ones in between are counted into the next
TRACKING_LOG_INTERVAL_S = 10.0


@dataclass(frozen=True)
class DeltaMessage:
    time_usec: int                               # host wall clock µs when the delta was made
    time_delta_usec: int                         # span between the two poses' capture times
    position_delta: Vec3                         # FRD body, m
    angle_delta: Vec3                            # FRD body rotation vector prev→current, rad
    confidence: float                            # 0–100
    # For the horizontal-only output (trackshot.core.ned): the same motion in the VIO's world frame
    world_delta: Vec3 = (0.0, 0.0, 0.0)          # FLU world, m (after the camera rotation)
    vio_yaw: Optional[float] = None              # VIO world yaw of the current pose, FLU (CCW +), rad
    stamp_us: Optional[int] = None               # the current pose's capture (or arrival) time, µs


@dataclass
class OdometryHealth:
    fps: float
    latency_ms: Optional[float]
    suggested_viso_delay_ms: Optional[int]
    poses: int
    deltas: int
    jumps: int
    rejected_spans: int
    last_pose: Optional[tuple]                   # (x, y, z, qw, qx, qy, qz) after camera rotation
    last_pose_age_s: Optional[float]
    stalled: bool                                # no pose for > 1 s
    tracking: str                                # "good" | "degraded" | "acquiring" | "lost" | "unknown"
    sending: bool = False                        # deltas are going out (tracking established, not lost)
    lost_for_s: Optional[float] = None           # how long the VIO has been frozen/lost, if it is
    losses: int = 0                              # times tracking was lost this run


# --- Frame math (straight ports of the extension's MavlinkNode methods) ---------------------------

def create_camera_rotation_matrix(camera_angle_deg: float) -> list[list[float]]:
    """Pitch (about y) by the camera's mounting angle, exactly as the extension builds it."""
    angle_rad = math.radians(camera_angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return [
        [cos_a, 0.0, sin_a],
        [0.0, 1.0, 0.0],
        [-sin_a, 0.0, cos_a],
    ]


def apply_camera_rotation(position: Sequence[float], quaternion: Sequence[float],
                          rotation: Optional[list[list[float]]]) -> tuple[Vec3, Quat]:
    """The VIO's camera pose → the vehicle's pose, for a camera fixed to the vehicle at a pitch.

    `rotation` (R) maps camera-body axes to vehicle-body axes. For a camera looking straight down
    with the top of the image towards the bow (90°) that is the camera's optical axis (VIO x) →
    vehicle down, its left (y) → left, and its image-up (z) → forward.

    The VIO's world frame is the camera's frame when VIO started, so it gets the same rotation:
    position p' = R·p and orientation q' = R ⊗ q ⊗ R⁻¹. That makes the body-frame deltas, the
    angle deltas and the yaw the vehicle's own (forward is forward, down is down) and the world
    level if the vehicle was level at start.

    (The BlueOS extension, and TrackShot ≤ 0.1.1, applied only q' = R ⊗ q, which cancels out of
    the body-frame deltas: a down-looking camera then sent its forward motion as vertical and
    its vertical as forward, unless VISO_ORIENT rotated it back on the autopilot.)
    """
    x, y, z = (float(v) for v in position)
    qw0, qx0, qy0, qz0 = (float(v) for v in quaternion)
    if rotation is None:
        return (x, y, z), (qw0, qx0, qy0, qz0)
    R = rotation
    translated = (
        R[0][0] * x + R[0][1] * y + R[0][2] * z,
        R[1][0] * x + R[1][1] * y + R[1][2] * z,
        R[2][0] * x + R[2][1] * y + R[2][2] * z,
    )
    # Rotation matrix → quaternion (trace form; the trace is ≥ 1 for any pitch in ±90°)
    cw = math.sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2
    cx = (R[2][1] - R[1][2]) / (4 * cw)
    cy = (R[0][2] - R[2][0]) / (4 * cw)
    cz = (R[1][0] - R[0][1]) / (4 * cw)
    mount = (cw, cx, cy, cz)
    mount_inv = (cw, -cx, -cy, -cz)
    qw, qx, qy, qz = quaternion_multiply(quaternion_multiply(mount, (qw0, qx0, qy0, qz0)), mount_inv)
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return translated, (qw / norm, qx / norm, qy / norm, qz / norm)


def quaternion_to_rotation_matrix(q: Sequence[float]) -> list[list[float]]:
    """Body→world rotation matrix of a (qw, qx, qy, qz) quaternion."""
    qw, qx, qy, qz = q
    return [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ]


def world_to_body_frame(world_vector: Sequence[float], q: Sequence[float]) -> Vec3:
    """v_body = Rᵀ·v_world, R being body→world, so Rᵀ takes world vectors into the body frame."""
    R = quaternion_to_rotation_matrix(q)
    return (
        R[0][0] * world_vector[0] + R[1][0] * world_vector[1] + R[2][0] * world_vector[2],
        R[0][1] * world_vector[0] + R[1][1] * world_vector[1] + R[2][1] * world_vector[2],
        R[0][2] * world_vector[0] + R[1][2] * world_vector[1] + R[2][2] * world_vector[2],
    )


def flu_to_frd_frame(v: Sequence[float]) -> Vec3:
    """FLU → FRD: forward stays, left becomes right (−y), up becomes down (−z).

    diag(1, −1, −1) is a proper rotation (180° about x), so it maps rotation vectors the same way
    as positions: a left (counter-clockwise from above) turn is +z in FLU and −z in FRD.
    """
    return (v[0], -v[1], -v[2])


def quaternion_yaw(q: Sequence[float]) -> float:
    """Yaw (rotation about world z, counter-clockwise positive in FLU) of a body→world (qw, qx, qy, qz)."""
    qw, qx, qy, qz = q
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quaternion_multiply(a: Sequence[float], b: Sequence[float]) -> Quat:
    """Hamilton product a ⊗ b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def relative_rotation_vector(q_prev: Sequence[float], q_cur: Sequence[float]) -> Vec3:
    """Rotation from the previous to the current orientation, in the previous body frame (FLU).

    With R body→world, R_cur = R_prev·R_rel, so q_rel = q_prev⁻¹ ⊗ q_cur. Returned as a rotation
    vector (axis × angle, rad) on the short way round.
    """
    pw, px, py, pz = q_prev
    norm_sq = pw * pw + px * px + py * py + pz * pz
    q_prev_inv = (pw / norm_sq, -px / norm_sq, -py / norm_sq, -pz / norm_sq)
    w, x, y, z = quaternion_multiply(q_prev_inv, q_cur)
    if w < 0.0:   # q and −q are the same rotation; pick the one with angle ≤ π
        w, x, y, z = -w, -x, -y, -z
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(sin_half, w)
    scale = angle / sin_half
    return (x * scale, y * scale, z * scale)


def viso_delay_estimate_ms(latency_ms: Optional[float], span_ms: Optional[float]) -> Optional[float]:
    """How long before its arrival at the autopilot a VISION_POSITION_DELTA's velocity was true.

    ArduPilot ignores VISION_POSITION_DELTA.time_usec: it stamps each delta with its own clock on
    receipt minus VISO_DELAY_MS (AP_VisualOdom_Backend::handle_vision_position_delta_msg), and the
    EKF fuses delPos/delTime as the velocity at that instant (writeBodyFrameOdom, which, unlike
    optical flow, applies no half-interval correction). That velocity is the average over the span
    between the two poses' captures, so it belongs to the span's midpoint: half a span before the
    newer capture. The delay to compensate is therefore capture→host latency + half the span +
    the host→autopilot transit.
    """
    if latency_ms is None:
        return None
    return latency_ms + 0.5 * (span_ms or 0.0) + HOST_TO_AUTOPILOT_MS


def suggest_viso_delay_ms(delay_ms: Optional[float]) -> Optional[int]:
    """A delay (see `viso_delay_estimate_ms`) rounded to the nearest 10 ms, within VISO_DELAY_MS's range."""
    if delay_ms is None:
        return None
    rounded = int(math.floor(delay_ms / 10.0 + 0.5)) * 10
    return max(VISO_DELAY_MIN_MS, min(VISO_DELAY_MAX_MS, rounded))


# --- Processor ------------------------------------------------------------------------------------

class OdometryProcessor:
    """Turns successive poses into body-frame deltas, rejecting VIO jumps and bad time spans."""

    def __init__(self, camera_angle_deg: float = 0.0, max_speed_mps: float = 3.0,
                 max_span_s: float = 2.0, clock: Callable[[], float] = time.time,
                 acquire_poses: Optional[int] = None, acquire_min_s: Optional[float] = None,
                 frozen_after_s: Optional[float] = None, frozen_min_repeats: Optional[int] = None):
        self.camera_angle_deg = float(camera_angle_deg)
        # None: the module's constant, read now (so a test can patch the module)
        self.acquire_poses = ACQUIRE_POSES if acquire_poses is None else int(acquire_poses)
        self.acquire_min_s = ACQUIRE_MIN_S if acquire_min_s is None else float(acquire_min_s)
        self.frozen_after_s = FROZEN_AFTER_S if frozen_after_s is None else float(frozen_after_s)
        self.frozen_min_repeats = FROZEN_MIN_REPEATS if frozen_min_repeats is None else int(frozen_min_repeats)
        self.max_speed_mps = float(max_speed_mps)
        self.max_span_us = int(max_span_s * 1e6)
        self._clock = clock
        self._camera_rotation = (create_camera_rotation_matrix(self.camera_angle_deg)
                                 if self.camera_angle_deg != 0.0 else None)
        self._lock = threading.Lock()
        self._reset_locked()

    def reset(self) -> None:
        with self._lock:
            self._reset_locked()

    def _reset_locked(self) -> None:
        # Differencing chain: the previous pose, its timestamp and which clock that came from
        self._prev_position: Optional[Vec3] = None
        self._prev_quaternion: Optional[Quat] = None
        self._prev_us: Optional[int] = None
        self._prev_clock_kind: Optional[str] = None
        # Health
        self._latency_s: Optional[float] = None
        self._span_s: Optional[float] = None
        self._poses = 0
        self._deltas = 0
        self._jumps = 0
        self._rejected_spans = 0
        self._last_pose: Optional[tuple] = None
        self._last_pose_time: Optional[float] = None
        self._last_tracking: Optional[bool] = None
        self._first_pose_time: Optional[float] = None
        self._pose_times: deque[float] = deque()
        self._jump_times: deque[float] = deque()
        self._last_jump_log = -math.inf
        # Tracking: established (sending), lost (frozen), and the streaks that decide them
        self._last_raw: Optional[tuple] = None
        self._bad_since: Optional[float] = None      # start of the current run of repeated/invalid poses
        self._bad_count = 0
        self._good_since: Optional[float] = None     # start of the current run of distinct poses
        self._good_count = 0
        self._acquired = False
        self._lost = False
        self._lost_since: Optional[float] = None
        self._losses = 0
        self._ever_acquired = False
        self._last_tracking_log = -math.inf
        self._unlogged_losses = 0
        self._lost_logged = False

    # ----------------------------------------------------------------------------------------
    def process(self, sample: PoseSample) -> Optional[DeltaMessage]:
        """Feeds one pose. Returns the delta since the previous pose, or None if there is none to send."""
        now = self._clock()
        with self._lock:
            raw = (*sample.position, *sample.quaternion)
            if not all(math.isfinite(v) for v in raw) or not any(raw[3:]):
                # A broken VIO output is a jump like any other: never difference across it
                self._count_jump(now, "invalid pose")
                self._restart_chain(None, None, None, None)
                self._unusable(sample.t_received, "invalid poses")
                return None
            position, quaternion = apply_camera_rotation(sample.position, sample.quaternion,
                                                         self._camera_rotation)
            self._record_pose(now, sample, position, quaternion)
            repeated = raw == self._last_raw
            self._last_raw = raw
            if repeated:
                # Not a measurement (see FROZEN_AFTER_S). The chain stays on the last distinct
                # pose, so a short repeat costs nothing; a long one is a loss.
                self._unusable(sample.t_received, "pose frozen")
                return None
            self._bad_since, self._bad_count = None, 0

            # The delta's span must come from capture times: arrival times jitter with host load
            # and the EKF divides the delta by this span. A sample without a capture time falls
            # back to its arrival time, and a chain never spans the two clocks.
            if sample.t_capture_us is not None:
                stamp_us, clock_kind = int(sample.t_capture_us), "capture"
            else:
                stamp_us, clock_kind = int(sample.t_received * 1e6), "received"

            if not self._acquired:
                # Not sending yet (start-up, or after a loss): count distinct poses, never
                # difference across the gap
                self._restart_chain(position, quaternion, stamp_us, clock_kind)
                self._acquiring(sample.t_received)
                return None

            if self._prev_us is None or self._prev_clock_kind != clock_kind:
                self._restart_chain(position, quaternion, stamp_us, clock_kind)
                return None

            dt_us = stamp_us - self._prev_us
            if dt_us < 0 or dt_us > self.max_span_us or dt_us < MIN_DELTA_SPAN_US:
                # Out of order, a stall, or too short to divide by: start again from this pose
                self._rejected_spans += 1
                self._restart_chain(position, quaternion, stamp_us, clock_kind)
                return None

            prev = self._prev_position
            world_delta = (position[0] - prev[0], position[1] - prev[1], position[2] - prev[2])
            speed = math.sqrt(sum(d * d for d in world_delta)) / (dt_us / 1e6)
            # The VIO lost track or reset: RTAB-Map snaps back to the origin, and either backend
            # can jump. Sent as a delta, that jump would look to the EKF like a sudden move and kick
            # position hold, so skip it and carry on from the new pose.
            prev_moved = any(v != 0.0 for v in prev)
            snapped_to_origin = prev_moved and position[0] == position[1] == position[2] == 0.0
            if speed > self.max_speed_mps or snapped_to_origin:
                self._count_jump(now, "snapped back to the origin" if snapped_to_origin
                                 else f"implied {speed:.1f} m/s")
                self._restart_chain(position, quaternion, stamp_us, clock_kind)
                return None

            position_delta = flu_to_frd_frame(world_to_body_frame(world_delta, quaternion))
            angle_delta = flu_to_frd_frame(relative_rotation_vector(self._prev_quaternion, quaternion))
            self._restart_chain(position, quaternion, stamp_us, clock_kind)
            self._deltas += 1
            if clock_kind == "capture":
                span_s = dt_us / 1e6
                self._span_s = (span_s if self._span_s is None else
                                (1 - LATENCY_EMA_ALPHA) * self._span_s + LATENCY_EMA_ALPHA * span_s)
            return DeltaMessage(
                world_delta=world_delta,
                vio_yaw=quaternion_yaw(quaternion),
                stamp_us=stamp_us,
                # Wall clock at send time, as the extension does. ArduPilot doesn't read it (it
                # stamps the delta on receipt minus VISO_DELAY_MS); it is for logs and other readers
                time_usec=int(now * 1e6),
                time_delta_usec=dt_us,
                position_delta=position_delta,
                angle_delta=angle_delta,
                confidence=0.0 if sample.tracking is False else 100.0,
            )

    # The streaks are timed by the poses' own arrival times, not the processing clock: the same in
    # real use, and right for a simulator running faster than real time.
    def _unusable(self, now: float, reason: str) -> None:
        """A repeated or invalid pose: progress towards "lost", and away from "established"."""
        if self._bad_since is None:
            self._bad_since, self._bad_count = now, 0
        self._bad_count += 1
        self._good_since, self._good_count = None, 0
        if (not self._lost and self._bad_count >= self.frozen_min_repeats
                and now - self._bad_since >= self.frozen_after_s):
            self._lost, self._lost_since, self._acquired = True, self._bad_since, False
            self._losses += 1
            self._log_lost(now, f"VIO tracking lost ({reason} for {now - self._bad_since:.1f} s): "
                                "not sending odometry until it recovers")

    def _acquiring(self, now: float) -> None:
        if self._good_since is None:
            self._good_since, self._good_count = now, 0
        self._good_count += 1
        if self._good_count >= self.acquire_poses and now - self._good_since >= self.acquire_min_s:
            self._acquired = True
            if self._lost and self._lost_logged:
                # Always paired with its "lost" message, so the log never leaves a loss open
                logger.info(f"VIO tracking regained after {now - self._lost_since:.1f} s: sending again")
            elif not self._ever_acquired:
                logger.info("VIO tracking established: sending odometry")
            self._ever_acquired = True
            self._lost, self._lost_since = False, None

    def _log_lost(self, now: float, message: str) -> None:
        """At most one "lost" message every TRACKING_LOG_INTERVAL_S: a VIO flickering in and out
        of tracking must not flood the log. The losses in between are counted into the next one."""
        self._lost_logged = now - self._last_tracking_log >= TRACKING_LOG_INTERVAL_S
        if not self._lost_logged:
            self._unlogged_losses += 1
            return
        if self._unlogged_losses:
            message += f" (lost {self._unlogged_losses} more time(s) since the last message)"
            self._unlogged_losses = 0
        self._last_tracking_log = now
        logger.warning(message)

    def _restart_chain(self, position: Optional[Vec3], quaternion: Optional[Quat],
                       stamp_us: Optional[int], clock_kind: Optional[str]) -> None:
        self._prev_position = position
        self._prev_quaternion = quaternion
        self._prev_us = stamp_us
        self._prev_clock_kind = clock_kind

    def _record_pose(self, now: float, sample: PoseSample, position: Vec3, quaternion: Quat) -> None:
        if sample.t_capture_us is not None:
            latency_s = sample.t_received - sample.t_capture_us / 1e6
            if 0.0 <= latency_s < MAX_PLAUSIBLE_LATENCY_S:
                # Capture-to-host delay, smoothed; one part of the suggested VISO_DELAY_MS
                self._latency_s = (latency_s if self._latency_s is None else
                                   (1 - LATENCY_EMA_ALPHA) * self._latency_s + LATENCY_EMA_ALPHA * latency_s)
        self._last_pose = (*position, *quaternion)
        self._last_pose_time = now
        self._last_tracking = sample.tracking
        if self._first_pose_time is None:
            self._first_pose_time = now
        self._poses += 1
        self._pose_times.append(now)
        self._prune(now)

    def _count_jump(self, now: float, reason: str) -> None:
        self._jumps += 1
        self._jump_times.append(now)
        if now - self._last_jump_log > JUMP_LOG_INTERVAL_S:
            self._last_jump_log = now
            logger.warning(f"Skipped a VIO pose jump ({reason}); {self._jumps} so far")

    def _prune(self, now: float) -> None:
        while self._pose_times and now - self._pose_times[0] > FPS_WINDOW_S:
            self._pose_times.popleft()
        while self._jump_times and now - self._jump_times[0] > DEGRADED_WINDOW_S:
            self._jump_times.popleft()

    # ----------------------------------------------------------------------------------------
    def tracking_state(self) -> str:
        """Cheap per-pose view for the run log: "lost", "acquiring" or "tracking"."""
        with self._lock:
            return "lost" if self._lost else "tracking" if self._acquired else "acquiring"

    def is_sending(self) -> bool:
        with self._lock:
            return self._acquired and not self._lost

    def health(self) -> OdometryHealth:
        now = self._clock()
        with self._lock:
            self._prune(now)
            fps = 0.0
            if self._first_pose_time is not None:
                # Until the window has filled, divide by the time we've actually been receiving
                window = min(FPS_WINDOW_S, now - self._first_pose_time)
                if window > 0:
                    fps = len(self._pose_times) / window
            age = None if self._last_pose_time is None else now - self._last_pose_time
            stalled = age is not None and age > STALL_AFTER_S
            if self._poses == 0 and not self._lost:
                tracking = "unknown"
            elif stalled or self._lost or self._last_tracking is False:
                tracking = "lost"
            elif not self._acquired:
                tracking = "acquiring"
            elif len(self._jump_times) >= DEGRADED_JUMPS:
                tracking = "degraded"
            else:
                tracking = "good"
            latency_ms = None if self._latency_s is None else self._latency_s * 1000.0
            span_ms = None if self._span_s is None else self._span_s * 1000.0
            return OdometryHealth(
                fps=fps,
                latency_ms=latency_ms,
                suggested_viso_delay_ms=suggest_viso_delay_ms(viso_delay_estimate_ms(latency_ms, span_ms)),
                poses=self._poses,
                deltas=self._deltas,
                jumps=self._jumps,
                rejected_spans=self._rejected_spans,
                last_pose=self._last_pose,
                last_pose_age_s=age,
                stalled=stalled,
                tracking=tracking,
                sending=self._acquired and not self._lost and not stalled,
                lost_for_s=None if self._lost_since is None else now - self._lost_since,
                losses=self._losses,
            )
