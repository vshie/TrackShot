"""Tracking: what TrackShot sends when the VIO starts, freezes and recovers (trackshot.core.odometry).

RTAB-Map, once lost, keeps reporting its last pose bit for bit. Sent as deltas, that is "not
moving" at full confidence, which ArduSub fuses while the vehicle drifts. So an exactly repeated
pose is never sent, a run of them lasting FROZEN_AFTER_S is "lost", and after a loss (and at
start-up) nothing is sent until ACQUIRE_POSES distinct poses spanning ACQUIRE_MIN_S have come in.
"""

import pytest
from loguru import logger

from trackshot.core import odometry
from trackshot.core.odometry import OdometryProcessor
from trackshot.mavlink.messages import TRACKING_VALUES
from trackshot.vio.source import PoseSample

T0 = 1_700_000_000.0
IDENTITY = (1.0, 0.0, 0.0, 0.0)
DT = 0.1   # 10 Hz


class Feed:
    """Poses at 10 Hz on a shared clock; `still` repeats the last position exactly (frozen VIO)."""

    def __init__(self, **kwargs):
        self.t = T0
        self.x = 0.0
        self.proc = OdometryProcessor(clock=lambda: self.t, **kwargs)
        self.sent = []

    def pose(self, frozen=False, position=None):
        if not frozen:
            self.x += 0.01                         # 0.1 m/s along x
        p = position if position is not None else (self.x, 0.0, 0.0)
        s = PoseSample(t_capture_us=int((self.t - 0.05) * 1e6), t_received=self.t, position=p, quaternion=IDENTITY)
        d = self.proc.process(s)
        if d is not None:
            self.sent.append((self.t, d))
        self.t += DT
        return d

    def run(self, n, **kwargs):
        return [self.pose(**kwargs) for _ in range(n)]

    def health(self):
        return self.proc.health()


@pytest.fixture
def logs():
    lines = []
    sink = logger.add(lambda m: lines.append((m.record["level"].name, m.record["message"])), level="INFO")
    yield lines
    logger.remove(sink)


def test_nothing_is_sent_until_tracking_is_established():
    f = Feed()
    assert f.health().tracking == "unknown"
    out = f.run(odometry.ACQUIRE_POSES - 1)
    assert out == [None] * (odometry.ACQUIRE_POSES - 1)
    assert f.health().tracking == "acquiring" and not f.health().sending
    f.run(5)
    h = f.health()
    assert h.tracking == "good" and h.sending
    assert f.sent, "deltas flow once established"
    # The first delta is one frame's motion, not everything since the first pose
    assert f.sent[0][1].position_delta[0] == pytest.approx(0.01)


def test_acquisition_needs_both_enough_poses_and_enough_time():
    f = Feed(acquire_poses=3, acquire_min_s=1.0)
    f.run(5)                                    # 5 poses but only 0.4 s
    assert f.health().tracking == "acquiring"
    f.run(8)
    assert f.health().tracking == "good"


def test_a_short_repeat_is_withheld_but_not_a_loss():
    f = Feed()
    f.run(15)
    sent = len(f.sent)
    assert f.run(3, frozen=True) == [None] * 3   # 0.3 s of repeats: below FROZEN_AFTER_S
    assert f.health().tracking == "good"
    d = f.pose()
    # The chain stayed on the last distinct pose: this delta spans the repeats, covering 0.01 m
    # (the frozen frames didn't move x) over 0.4 s
    assert d is not None and d.position_delta[0] == pytest.approx(0.01)
    assert d.time_delta_usec == pytest.approx(400_000, abs=10)
    assert len(f.sent) == sent + 1


def test_a_frozen_vio_is_lost_and_sends_nothing_until_it_recovers(logs):
    f = Feed()
    f.run(15)
    sent = len(f.sent)
    f.run(25, frozen=True)                       # 2.5 s frozen
    h = f.health()
    assert h.tracking == "lost" and not h.sending and h.losses == 1
    assert h.lost_for_s == pytest.approx(2.5, abs=0.15)
    assert len(f.sent) == sent                   # not one zero delta went out
    assert any(level == "WARNING" and "VIO tracking lost (pose frozen" in text for level, text in logs)

    # Moving again: re-acquired before anything is sent, and nothing spans the frozen gap
    f.run(odometry.ACQUIRE_POSES - 1)
    assert f.health().tracking == "lost" or f.health().tracking == "acquiring"
    assert len(f.sent) == sent
    f.run(6)
    h = f.health()
    assert h.tracking == "good" and h.sending and h.lost_for_s is None
    first_after = f.sent[sent][1]
    assert first_after.position_delta[0] == pytest.approx(0.01)
    assert first_after.time_delta_usec == pytest.approx(100_000, abs=10)
    assert any("VIO tracking regained" in text for _, text in logs)


def test_lost_needs_both_the_time_and_the_repeats():
    f = Feed(frozen_after_s=1.0, frozen_min_repeats=50)
    f.run(15)
    f.run(20, frozen=True)                       # 2 s but only 20 repeats
    assert f.health().tracking == "good"
    f.run(40, frozen=True)
    assert f.health().tracking == "lost"


def test_a_run_of_invalid_poses_is_lost_too():
    f = Feed()
    f.run(15)
    f.run(15, position=(float("nan"), 0.0, 0.0))
    assert f.health().tracking == "lost" and not f.health().sending


def test_lost_and_regained_messages_are_rate_limited(logs):
    f = Feed()
    f.run(15)
    for _ in range(3):                           # lose and regain three times inside 10 s
        f.run(12, frozen=True)
        f.run(12)
    lost = [text for level, text in logs if "VIO tracking lost" in text]
    regained = [text for level, text in logs if "VIO tracking regained" in text]
    # One lost/regained pair; the two later losses inside 10 s are counted, not logged
    assert (len(lost), len(regained)) == (1, 1)
    assert f.health().losses == 3
    f.t += odometry.TRACKING_LOG_INTERVAL_S
    f.run(12, frozen=True)
    lost = [text for level, text in logs if "VIO tracking lost" in text]
    assert len(lost) == 2 and "more time(s) since the last message" in lost[-1]


def test_cockpit_tracking_value_marks_not_sending():
    assert TRACKING_VALUES["acquiring"] < 0.5 and TRACKING_VALUES["lost"] < 0.5
    assert TRACKING_VALUES["good"] == 1.0
