"""trackshot.core.altitude: altitude above the seabed from the stereo depth."""

import numpy as np
import pytest

from trackshot.core.altitude import range_from_depth


def depth(h=200, w=320, mm=2500, valid=1.0, seed=0):
    rng = np.random.default_rng(seed)
    d = np.full((h, w), mm, dtype=np.uint16)
    d[rng.random((h, w)) > valid] = 0          # stereo holes
    return d


def test_median_of_the_middle_of_the_image():
    d = depth(mm=2500)
    d[:, :40] = 800                            # something near the edge doesn't count
    r = range_from_depth(d, t=5.0)
    assert r.distance_m == pytest.approx(2.5) and r.quality == 100 and r.t == 5.0


def test_a_fish_in_the_middle_is_outvoted():
    d = depth(mm=3000)
    d[95:105, 155:165] = 600                   # small, near: the median ignores it
    assert range_from_depth(d, 0).distance_m == pytest.approx(3.0)


def test_quality_follows_the_valid_fraction():
    assert range_from_depth(depth(valid=0.9), 0).quality == 100
    q = range_from_depth(depth(valid=0.3), 0).quality
    assert 50 < q < 70                          # 0.3 / 0.5 → ~60: below RNGFND_SQ_MIN 90
    r = range_from_depth(depth(valid=0.05), 0)
    assert r.distance_m is None and r.quality == 1


def test_out_of_range_depth_is_ignored():
    assert range_from_depth(depth(mm=100), 0).distance_m is None      # 0.1 m: too close
    assert range_from_depth(depth(mm=15000), 0).distance_m is None    # 15 m: beyond 10 m
