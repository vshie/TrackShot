"""trackshot.core.ned: vehicle-frame VIO motion → NED position and velocity, rotated by the compass."""

import math

import pytest

from trackshot.core import ned
from trackshot.core.ned import NedTracker, body_to_ned, wrap_pi
from trackshot.core.odometry import DeltaMessage

T0_US = 1_700_000_000_000_000
DT_US = 100_000


def delta(i, frd=(0.03, 0.0, 0.0), vio_yaw=0.0):
    """The i-th 10 Hz delta: `frd` is the vehicle-frame motion, `vio_yaw` the FLU (CCW+) VIO yaw."""
    return DeltaMessage(time_usec=T0_US + i * DT_US, time_delta_usec=DT_US, position_delta=frd,
                        angle_delta=(0, 0, 0), confidence=100.0, world_delta=(0, 0, 0), vio_yaw=vio_yaw,
                        stamp_us=T0_US + i * DT_US)


def t(i):
    return (T0_US + i * DT_US) / 1e6


def with_attitude(heading_deg, roll_deg=0.0, pitch_deg=0.0, at=0):
    tr = NedTracker()
    tr.observe_attitude(math.radians(roll_deg), math.radians(pitch_deg), math.radians(heading_deg), t(at))
    return tr


def test_nothing_without_the_autopilot_attitude():
    tr = NedTracker()
    assert tr.update(delta(0)) is None
    tr.observe_attitude(None, 0.0, 0.0, t(0))
    assert tr.update(delta(1)) is None
    tr.observe_attitude(0.0, 0.0, 0.0, t(1))
    assert tr.update(delta(2)) is not None


def test_a_stale_attitude_stops_the_output():
    tr = with_attitude(0.0, at=0)
    assert tr.update(delta(1)) is not None
    late = int(ned.ATTITUDE_MAX_AGE_S * 10) + 5
    assert tr.update(delta(late)) is None


@pytest.mark.parametrize("heading_deg, north, east", [
    (0.0, 0.03, 0.0), (90.0, 0.0, 0.03), (180.0, -0.03, 0.0), (270.0, 0.0, -0.03), (45.0, 0.0212, 0.0212)])
def test_forward_goes_along_the_compass_heading(heading_deg, north, east):
    tr = with_attitude(heading_deg)
    e = tr.update(delta(1))
    assert e.velocity[0] * 0.1 == pytest.approx(north, abs=1e-4)
    assert e.velocity[1] * 0.1 == pytest.approx(east, abs=1e-4)
    assert e.velocity[2] == 0.0 and e.position[2] == 0.0
    assert abs(wrap_pi(e.yaw - math.radians(heading_deg))) < 1e-9


def test_right_is_east_when_heading_north():
    e = with_attitude(0.0).update(delta(1, frd=(0.0, 0.05, 0.0)))
    assert e.velocity[:2] == pytest.approx((0.0, 0.5))


def test_vertical_motion_never_comes_out():
    e = with_attitude(30.0).update(delta(1, frd=(0.0, 0.0, 0.4)))
    assert e.velocity == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert e.position[2] == 0.0


def test_a_pitched_vehicle_moving_forward_is_level_motion_scaled():
    # Nose 10° down, moving along the body x: horizontally cos(10°) of it, the rest is vertical
    e = with_attitude(0.0, pitch_deg=-10.0).update(delta(1, frd=(0.1, 0.0, 0.0)))
    assert e.velocity[0] == pytest.approx(math.cos(math.radians(10)))
    assert e.velocity[1] == pytest.approx(0.0, abs=1e-12)


def test_no_learned_offset_the_vio_yaw_itself_doesnt_matter():
    # The VIO's own world yaw is arbitrary; only the compass sets the direction
    a = with_attitude(90.0).update(delta(1, vio_yaw=0.0))
    b = with_attitude(90.0).update(delta(1, vio_yaw=math.radians(123.0)))
    assert a.velocity == pytest.approx(b.velocity)


def test_heading_is_carried_forward_by_the_vio_turn_between_samples():
    tr = NedTracker()
    tr.update(delta(0, vio_yaw=0.0))
    tr.observe_attitude(0.0, 0.0, math.radians(90.0), t(0))        # compass east at t0
    # 0.4 s later the VIO has turned 8° right (FLU yaw −8°): heading 98° until the next sample
    e = tr.update(delta(4, vio_yaw=-math.radians(8.0)))
    assert math.degrees(e.yaw) == pytest.approx(98.0)
    # a fresh compass sample takes over
    tr.observe_attitude(0.0, 0.0, math.radians(100.0), t(4))
    e = tr.update(delta(5, vio_yaw=-math.radians(8.0)))
    assert math.degrees(e.yaw) == pytest.approx(100.0)


def test_without_vio_history_at_the_sample_the_compass_is_used_as_is():
    tr = with_attitude(45.0, at=0)                                  # no VIO yaw near t0
    e = tr.update(delta(20, vio_yaw=math.radians(30.0)))
    assert math.degrees(e.yaw) == pytest.approx(45.0)


def test_position_integrates_and_reanchors_on_the_ekf_after_a_gap():
    tr = with_attitude(90.0)
    e = tr.update(delta(1), ekf_north_east=(10.0, 20.0))
    assert e.anchored and e.position[:2] == pytest.approx((10.0, 20.03))
    e = tr.update(delta(2), ekf_north_east=(99.0, 99.0))
    assert not e.anchored and e.position[:2] == pytest.approx((10.0, 20.06))
    tr.observe_attitude(0.0, 0.0, math.radians(90.0), t(15))
    e = tr.update(delta(16), ekf_north_east=(-5.0, 7.0))           # > 1 s since the last delta
    assert e.anchored and e.position[:2] == pytest.approx((-5.0, 7.03))


def test_body_to_ned_matches_a_rotation_matrix():
    # yaw 90°: body x → east; body y → south; body z → down
    assert body_to_ned((1, 0, 0), 0, 0, math.pi / 2) == pytest.approx((0, 1, 0), abs=1e-12)
    assert body_to_ned((0, 1, 0), 0, 0, math.pi / 2) == pytest.approx((-1, 0, 0), abs=1e-12)
    assert body_to_ned((0, 0, 1), 0, 0, math.pi / 2) == pytest.approx((0, 0, 1), abs=1e-12)
    # rolled 90° right (right side down), heading north: the belly faces west, so body z → −east
    assert body_to_ned((0, 0, 1), math.pi / 2, 0, 0) == pytest.approx((0, -1, 0), abs=1e-12)
