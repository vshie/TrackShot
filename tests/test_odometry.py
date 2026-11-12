"""Golden tests for the pose → VISION_POSITION_DELTA math (trackshot.core.odometry).

Expected values are derived by hand in the comments. Frames: VIO world FLU (x fwd, y left, z up),
quaternions (qw, qx, qy, qz) body→world, output body FRD (x fwd, y right, z down).
"""

import math
import threading
from types import SimpleNamespace

import pytest

import trackshot.core.odometry as odometry_module
from trackshot.core.odometry import (
    OdometryProcessor,
    HOST_TO_AUTOPILOT_MS,
    relative_rotation_vector,
    suggest_viso_delay_ms,
    viso_delay_estimate_ms,
)
from trackshot.vio.source import PoseSample

@pytest.fixture(autouse=True)
def send_from_the_first_delta(monkeypatch):
    """The golden tests check the math from the second pose on; tracking acquisition (start-up
    gating) has its own tests in test_tracking.py."""
    monkeypatch.setattr(odometry_module, "ACQUIRE_POSES", 0)
    monkeypatch.setattr(odometry_module, "ACQUIRE_MIN_S", 0.0)


IDENTITY = (1.0, 0.0, 0.0, 0.0)
C45 = math.cos(math.radians(45))
YAW_LEFT_90 = (C45, 0.0, 0.0, C45)   # +90° about world z (up): the nose turns left
T0_US = 1_700_000_000_000_000


class FakeClock:
    def __init__(self, t: float = 1_700_000_000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


def sample(t_us, position, quaternion=IDENTITY, tracking=None, latency_s=0.05):
    """A pose captured at `t_us` that reached the host `latency_s` later (None: no capture time)."""
    return PoseSample(
        t_capture_us=t_us,
        t_received=(t_us if t_us is not None else T0_US) / 1e6 + latency_s,
        position=position,
        quaternion=quaternion,
        tracking=tracking,
    )


def delta_between(p0, p1, q=IDENTITY, dt_us=100_000, **kwargs):
    proc = OdometryProcessor(**kwargs)
    assert proc.process(sample(T0_US, p0, q)) is None   # first pose only starts the chain
    return proc.process(sample(T0_US + dt_us, p1, q))


# --- Position delta golden values ---------------------------------------------------------------

def test_identity_forward_is_frd_plus_x():
    # Body = world. 0.1 m along world x (forward) → body forward 0.1, FRD (0.1, 0, 0)
    d = delta_between((0, 0, 0), (0.1, 0, 0))
    assert d.position_delta == pytest.approx((0.1, 0.0, 0.0))
    assert d.time_delta_usec == 100_000
    assert d.confidence == 100.0


def test_world_left_is_frd_minus_y():
    # +y in FLU is left; FRD's y points right, so 0.1 m left is y = −0.1
    d = delta_between((0, 0, 0), (0, 0.1, 0))
    assert d.position_delta == pytest.approx((0.0, -0.1, 0.0))


def test_world_up_is_frd_minus_z():
    # +z in FLU is up; FRD's z points down, so rising 0.1 m is z = −0.1
    d = delta_between((0, 0, 0), (0, 0, 0.1))
    assert d.position_delta == pytest.approx((0.0, 0.0, -0.1))


def test_yawed_left_90_moving_world_left_is_forward():
    # q = (cos45, 0, 0, sin45) turns the body +90° about up: body x = R·(1,0,0) = world +y.
    # Moving 0.1 m along world +y is therefore straight ahead: FRD (0.1, 0, 0)
    d = delta_between((1, 1, 0), (1, 1.1, 0), q=YAW_LEFT_90)
    assert d.position_delta == pytest.approx((0.1, 0.0, 0.0), abs=1e-12)


def test_yawed_left_90_moving_world_forward_is_right():
    # Same heading (nose at world +y), moving along world +x: that's to the body's right
    # (body y = world −x in FLU), so FRD y = +0.1
    d = delta_between((1, 1, 0), (1.1, 1, 0), q=YAW_LEFT_90)
    assert d.position_delta == pytest.approx((0.0, 0.1, 0.0), abs=1e-12)


# --- Camera angle, compared with a verbatim copy of the extension's functions -------------------

class _ReferenceMath:
    """Straight copy of app/mavlink_node.py's create_camera_rotation_matrix,
    apply_camera_rotation, quaternion_to_rotation_matrix, world_to_body_frame, flu_to_frd_frame."""

    def __init__(self, camera_angle):
        self.camera_angle = camera_angle
        self.camera_rotation_matrix = self.create_camera_rotation_matrix() if camera_angle != 0.0 else None

    def create_camera_rotation_matrix(self):
        angle_rad = math.radians(self.camera_angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        return [[cos_a, 0, sin_a], [0, 1, 0], [-sin_a, 0, cos_a]]

    def apply_camera_rotation(self, translation, quaternion):
        if self.camera_rotation_matrix is None:
            return translation, quaternion
        R = self.camera_rotation_matrix
        transformed_translation = type(translation)(
            x=R[0][0] * translation.x + R[0][1] * translation.y + R[0][2] * translation.z,
            y=R[1][0] * translation.x + R[1][1] * translation.y + R[1][2] * translation.z,
            z=R[2][0] * translation.x + R[2][1] * translation.y + R[2][2] * translation.z,
        )
        camera_qw = math.sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2
        camera_qx = (R[2][1] - R[1][2]) / (4 * camera_qw)
        camera_qy = (R[0][2] - R[2][0]) / (4 * camera_qw)
        camera_qz = (R[1][0] - R[0][1]) / (4 * camera_qw)
        qw = camera_qw * quaternion.qw - camera_qx * quaternion.qx - camera_qy * quaternion.qy - camera_qz * quaternion.qz
        qx = camera_qw * quaternion.qx + camera_qx * quaternion.qw + camera_qy * quaternion.qz - camera_qz * quaternion.qy
        qy = camera_qw * quaternion.qy - camera_qx * quaternion.qz + camera_qy * quaternion.qw + camera_qz * quaternion.qx
        qz = camera_qw * quaternion.qz + camera_qx * quaternion.qy - camera_qy * quaternion.qx + camera_qz * quaternion.qw
        norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        return transformed_translation, type(quaternion)(qw=qw / norm, qx=qx / norm, qy=qy / norm, qz=qz / norm)

    def quaternion_to_rotation_matrix(self, q):
        qw, qx, qy, qz = q.qw, q.qx, q.qy, q.qz
        return [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
        ]

    def world_to_body_frame(self, world_vector, quaternion):
        R = self.quaternion_to_rotation_matrix(quaternion)
        R_T = [[R[j][i] for j in range(3)] for i in range(3)]
        return [
            R_T[0][0] * world_vector[0] + R_T[0][1] * world_vector[1] + R_T[0][2] * world_vector[2],
            R_T[1][0] * world_vector[0] + R_T[1][1] * world_vector[1] + R_T[1][2] * world_vector[2],
            R_T[2][0] * world_vector[0] + R_T[2][1] * world_vector[1] + R_T[2][2] * world_vector[2],
        ]

    def flu_to_frd_frame(self, flu_vector):
        return [flu_vector[0], -flu_vector[1], -flu_vector[2]]


def _vec(p):
    return SimpleNamespace(x=p[0], y=p[1], z=p[2])


def _quat(q):
    return SimpleNamespace(qw=q[0], qx=q[1], qy=q[2], qz=q[3])


def _normalised(q):
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


@pytest.mark.parametrize("angle", [0.0, 30.0, 45.0, 90.0, -20.0])
def test_camera_angle_matches_reference(angle):
    q0 = _normalised((0.9, 0.1, -0.2, 0.3))
    q1 = _normalised((0.88, 0.12, -0.21, 0.33))
    p0, p1 = (0.4, -0.3, 0.2), (0.45, -0.28, 0.17)
    ref = _ReferenceMath(angle)
    t0, r0 = ref.apply_camera_rotation(_vec(p0), _quat(q0))
    t1, r1 = ref.apply_camera_rotation(_vec(p1), _quat(q1))
    world_delta = [t1.x - t0.x, t1.y - t0.y, t1.z - t0.z]
    expected = ref.flu_to_frd_frame(ref.world_to_body_frame(world_delta, r1))

    proc = OdometryProcessor(camera_angle_deg=angle)
    proc.process(sample(T0_US, p0, q0))
    d = proc.process(sample(T0_US + 100_000, p1, q1))
    assert d.position_delta == pytest.approx(tuple(expected), abs=1e-12)
    assert proc.health().last_pose == pytest.approx((t1.x, t1.y, t1.z, r1.qw, r1.qx, r1.qy, r1.qz), abs=1e-12)


def test_camera_angle_cancels_out_of_the_delta():
    # The extension rotates position and orientation by the same R_cam, so
    # R'ᵀ·Δp' = (R_cam·R)ᵀ·R_cam·Δp = Rᵀ·Δp: the angle changes the reported pose, not the delta.
    q0 = _normalised((0.9, 0.1, -0.2, 0.3))
    q1 = _normalised((0.88, 0.12, -0.21, 0.33))
    deltas = []
    for angle in (0.0, 45.0, 90.0):
        proc = OdometryProcessor(camera_angle_deg=angle)
        proc.process(sample(T0_US, (0.4, -0.3, 0.2), q0))
        deltas.append(proc.process(sample(T0_US + 100_000, (0.45, -0.28, 0.17), q1)))
    for d in deltas[1:]:
        assert d.position_delta == pytest.approx(deltas[0].position_delta, abs=1e-12)
        assert d.angle_delta == pytest.approx(deltas[0].angle_delta, abs=1e-12)


def test_camera_down_90_rotates_reported_pose():
    # R_cam(90°) = [[0,0,1],[0,1,0],[-1,0,0]]: VIO (1, 0, 0) → (0, 0, −1)
    proc = OdometryProcessor(camera_angle_deg=90.0)
    proc.process(sample(T0_US, (1.0, 0.0, 0.0)))
    assert proc.health().last_pose[:3] == pytest.approx((0.0, 0.0, -1.0), abs=1e-12)


# --- Angle delta --------------------------------------------------------------------------------

def test_left_turn_is_negative_frd_yaw():
    # A 10° left turn (counter-clockwise seen from above) is +10° about FLU z (up).
    # FRD's z points down, and seen from below that same turn is clockwise, so it's −10° about
    # FRD z: angle_delta = (0, 0, −0.1745). (ArduPilot: positive yaw = turning right.)
    half = math.radians(10) / 2
    proc = OdometryProcessor()
    proc.process(sample(T0_US, (0, 0, 0), IDENTITY))
    d = proc.process(sample(T0_US + 100_000, (0, 0, 0), (math.cos(half), 0, 0, math.sin(half))))
    assert d.angle_delta == pytest.approx((0.0, 0.0, -math.radians(10)), abs=1e-12)
    assert d.position_delta == pytest.approx((0.0, 0.0, 0.0))


def test_angle_delta_is_in_the_body_frame():
    # Heading world +y (yawed left 90°), then roll 10° about the BODY x axis (q1 = q0 ⊗ q_roll):
    # the rotation must come out about body x (not world x/y). FLU +x roll lifts the left side,
    # i.e. right side down, which is also +x roll in FRD: (+0.1745, 0, 0).
    half = math.radians(10) / 2
    roll = (math.cos(half), math.sin(half), 0.0, 0.0)
    w0, x0, y0, z0 = YAW_LEFT_90
    w1, x1, y1, z1 = roll
    q1 = (w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1, w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
          w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1, w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1)
    proc = OdometryProcessor()
    proc.process(sample(T0_US, (0, 0, 0), YAW_LEFT_90))
    d = proc.process(sample(T0_US + 100_000, (0, 0, 0), q1))
    assert d.angle_delta == pytest.approx((math.radians(10), 0.0, 0.0), abs=1e-12)


def test_rotation_vector_takes_the_short_way():
    # −q is the same orientation; a 10° turn written with qw < 0 must still be 10°, not 350°
    half = math.radians(10) / 2
    rv = relative_rotation_vector(IDENTITY, (-math.cos(half), 0, 0, -math.sin(half)))
    assert rv == pytest.approx((0.0, 0.0, math.radians(10)), abs=1e-12)


# --- Rejections ---------------------------------------------------------------------------------

def test_speed_jump_is_rejected_and_chain_restarts_from_it():
    proc = OdometryProcessor()
    proc.process(sample(T0_US, (0, 0, 0)))
    # 0.5 m in 0.1 s = 5 m/s > 3 m/s: a VIO jump
    assert proc.process(sample(T0_US + 100_000, (0.5, 0, 0))) is None
    assert proc.health().jumps == 1
    # The next delta is measured from the jumped pose, not from the origin
    d = proc.process(sample(T0_US + 200_000, (0.6, 0, 0)))
    assert d.position_delta == pytest.approx((0.1, 0.0, 0.0))
    # 0.29 m in 0.1 s = 2.9 m/s is still believable
    assert proc.process(sample(T0_US + 300_000, (0.89, 0, 0))) is not None


def test_snap_to_origin_is_rejected():
    proc = OdometryProcessor()
    proc.process(sample(T0_US, (0.1, 0, 0)))
    # 0.1 m in 0.1 s is a plausible 1 m/s, but landing exactly on (0, 0, 0) after having moved
    # is RTAB-Map resetting
    assert proc.process(sample(T0_US + 100_000, (0.0, 0.0, 0.0))) is None
    assert proc.health().jumps == 1


def test_starting_at_origin_is_not_a_snap():
    proc = OdometryProcessor()
    proc.process(sample(T0_US, (0.0, 0.0, 0.0)))
    # An exact repeat isn't a measurement (see test_tracking.py), so it is withheld, but not as a jump
    assert proc.process(sample(T0_US + 100_000, (0.0, 0.0, 0.0))) is None
    d = proc.process(sample(T0_US + 200_000, (0.01, 0.0, 0.0)))
    assert d is not None and d.position_delta == pytest.approx((0.01, 0.0, 0.0))
    assert proc.health().jumps == 0


def test_invalid_pose_is_a_jump():
    proc = OdometryProcessor()
    proc.process(sample(T0_US, (0.0, 0.0, 0.0)))
    assert proc.process(sample(T0_US + 100_000, (math.nan, 0.0, 0.0))) is None
    assert proc.process(sample(T0_US + 200_000, (0.0, 0.0, 0.0), (0, 0, 0, 0))) is None
    assert proc.health().jumps == 2
    # The chain restarts: the next good pose only starts it again
    assert proc.process(sample(T0_US + 300_000, (0.001, 0.0, 0.0))) is None
    assert proc.process(sample(T0_US + 400_000, (0.002, 0.0, 0.0))) is not None


@pytest.mark.parametrize("dt_us", [2_000_001, 2_500_000, -100_000, 999, 0])
def test_bad_spans_are_rejected_and_restart(dt_us):
    proc = OdometryProcessor()
    proc.process(sample(T0_US, (0, 0, 0)))
    assert proc.process(sample(T0_US + dt_us, (0.01, 0, 0))) is None
    assert proc.health().rejected_spans == 1
    # Restarted from the rejected pose: the next delta spans 0.1 s from it
    d = proc.process(sample(T0_US + dt_us + 100_000, (0.02, 0, 0)))
    assert d.time_delta_usec == 100_000
    assert d.position_delta == pytest.approx((0.01, 0.0, 0.0))


def test_two_second_span_is_still_allowed():
    # The cap is "> 2 s" as in the extension; 1 m over exactly 2 s is 0.5 m/s
    d = delta_between((0, 0, 0), (1.0, 0, 0), dt_us=2_000_000)
    assert d is not None and d.time_delta_usec == 2_000_000


def test_missing_capture_time_restarts_chain():
    proc = OdometryProcessor()
    assert proc.process(sample(T0_US, (0, 0, 0))) is None
    assert proc.process(sample(T0_US + 100_000, (0.01, 0, 0))) is not None
    # No capture time: never difference an arrival time against a capture time
    assert proc.process(sample(None, (0.02, 0, 0))) is None
    # Back on capture times: again only a restart
    assert proc.process(sample(T0_US + 300_000, (0.03, 0, 0))) is None
    d = proc.process(sample(T0_US + 400_000, (0.04, 0, 0)))
    assert d.time_delta_usec == 100_000
    assert d.position_delta == pytest.approx((0.01, 0.0, 0.0))


def test_arrival_times_are_used_when_no_capture_times():
    # A source that never has capture times still works, on arrival times (as the extension did)
    proc = OdometryProcessor()
    proc.process(PoseSample(None, 100.0, (0, 0, 0), IDENTITY))
    d = proc.process(PoseSample(None, 100.1, (0.01, 0, 0), IDENTITY))
    assert d.time_delta_usec == pytest.approx(100_000, abs=1)


# --- Timing, latency, health --------------------------------------------------------------------

def test_time_usec_is_the_processing_clock():
    clock = FakeClock(1_800_000_000.25)
    proc = OdometryProcessor(clock=clock)
    proc.process(sample(T0_US, (0, 0, 0)))
    d = proc.process(sample(T0_US + 100_000, (0.01, 0, 0)))
    assert d.time_usec == 1_800_000_000_250_000


def test_latency_ema_and_suggestion():
    proc = OdometryProcessor()
    proc.process(sample(T0_US, (0, 0, 0), latency_s=0.050))
    assert proc.health().latency_ms == pytest.approx(50.0)
    proc.process(sample(T0_US + 100_000, (0.001, 0, 0), latency_s=0.200))
    # 0.9 × 50 + 0.1 × 200 = 65 ms
    h = proc.health()
    assert h.latency_ms == pytest.approx(65.0)
    # 65 ms latency + half the 100 ms span + 5 ms transit = 120
    assert h.suggested_viso_delay_ms == 120
    # Nonsense latencies (≥ 5 s or negative) are ignored
    proc.process(sample(T0_US + 200_000, (0.002, 0, 0), latency_s=7.0))
    proc.process(sample(T0_US + 300_000, (0, 0, 0), latency_s=-0.5))
    assert proc.health().latency_ms == pytest.approx(65.0)


def test_viso_delay_covers_half_the_span_and_the_transit():
    # ArduPilot stamps a delta at receipt − VISO_DELAY_MS and the delta's velocity is the average
    # over its span, true at the span's midpoint: latency + span/2 + transit
    assert viso_delay_estimate_ms(None, 100.0) is None
    assert viso_delay_estimate_ms(60.0, 100.0) == pytest.approx(60.0 + 50.0 + HOST_TO_AUTOPILOT_MS)
    assert viso_delay_estimate_ms(60.0, None) == pytest.approx(60.0 + HOST_TO_AUTOPILOT_MS)


def test_suggested_delay_removes_the_velocity_bias_under_acceleration():
    """A vehicle accelerating at 0.5 m/s² filmed at 10 fps with 60 ms capture→host latency: with
    the suggested VISO_DELAY_MS, each delta's velocity matches the truth at the time ArduPilot
    assigns it (its receipt time, 5 ms after the host, minus VISO_DELAY_MS)."""
    accel, fps, latency, transit = 0.5, 10.0, 0.060, HOST_TO_AUTOPILOT_MS / 1000.0
    clock = FakeClock(0.0)
    proc = OdometryProcessor(clock=clock)
    deltas = []
    for k in range(40):
        t_capture = 100.0 + k / fps
        clock.t = t_capture + latency
        d = proc.process(PoseSample(t_capture_us=int(round(t_capture * 1e6)), t_received=clock.t,
                                    position=(0.5 * accel * (t_capture - 100.0) ** 2, 0.0, 0.0),
                                    quaternion=IDENTITY))
        if d is not None:
            deltas.append((d.position_delta[0] / (d.time_delta_usec / 1e6), clock.t + transit))
    delay_s = proc.health().suggested_viso_delay_ms / 1000.0
    errors = [v - accel * (received - delay_s - 100.0) for v, received in deltas]
    # Within the 10 ms rounding of the suggestion: ≤ 0.5 m/s² × 5 ms = 2.5 mm/s
    assert abs(sum(errors) / len(errors)) <= 0.0026


def test_suggested_viso_delay_rounding_and_clamp():
    assert suggest_viso_delay_ms(None) is None
    assert suggest_viso_delay_ms(64.9) == 60
    assert suggest_viso_delay_ms(65.0) == 70
    assert suggest_viso_delay_ms(4.0) == 0
    assert suggest_viso_delay_ms(-30.0) == 0
    assert suggest_viso_delay_ms(400.0) == 250


def test_confidence_zero_when_not_tracking():
    proc = OdometryProcessor()
    proc.process(sample(T0_US, (0, 0, 0)))
    d = proc.process(sample(T0_US + 100_000, (0.01, 0, 0), tracking=False))
    assert d.confidence == 0.0
    assert proc.health().tracking == "lost"


def test_health_fps_stall_and_tracking_states():
    clock = FakeClock()
    proc = OdometryProcessor(clock=clock)
    h = proc.health()
    assert (h.tracking, h.poses, h.fps, h.stalled, h.last_pose) == ("unknown", 0, 0.0, False, None)

    t_us = T0_US
    for i in range(30):   # 3 s at 10 Hz, holding station (real VIO output is never bit-identical)
        proc.process(sample(t_us, (i * 1e-4, 0, 0)))
        clock.t += 0.1
        t_us += 100_000
    h = proc.health()
    assert h.fps == pytest.approx(10.0, rel=0.06)
    assert (h.tracking, h.poses, h.deltas, h.stalled) == ("good", 30, 29, False)
    assert h.last_pose_age_s == pytest.approx(0.1)

    # Three jumps inside 10 s → degraded
    for _ in range(3):
        proc.process(sample(t_us, (10.0, 0, 0)))
        proc.process(sample(t_us + 100_000, (0.0, 0, 0)))   # both 100 m/s
        clock.t += 0.2
        t_us += 200_000
    assert proc.health().tracking == "degraded"

    # No pose for > 1 s → stalled and lost, and the rate falls to 0
    clock.t += 2.5
    h = proc.health()
    assert h.stalled and h.tracking == "lost" and h.fps == 0.0

    # The jumps age out of the 10 s window
    clock.t += 10.0
    proc.process(sample(t_us + 12_000_000, (0.5, 0, 0)))
    assert proc.health().tracking == "good"

    proc.reset()
    h = proc.health()
    assert (h.tracking, h.poses, h.jumps, h.latency_ms) == ("unknown", 0, 0, None)


def test_process_is_thread_safe():
    proc = OdometryProcessor()
    errors = []

    def feed(offset):
        try:
            for i in range(500):
                proc.process(sample(T0_US + offset + i * 10_000, (i * 0.001, 0, 0)))
                proc.health()
        except Exception as e:   # pragma: no cover - only on failure
            errors.append(e)

    threads = [threading.Thread(target=feed, args=(k * 5_000_000,)) for k in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not errors
    assert proc.health().poses == 2000
