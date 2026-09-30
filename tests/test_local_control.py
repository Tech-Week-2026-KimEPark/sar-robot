import math

import pytest

from sar.control.local_control import pure_pursuit, safety_filter
from sar.geometry import Pose


def test_pure_pursuit_no_path_is_reached():
    v, w, reached = pure_pursuit(Pose(), [])
    assert (v, w, reached) == (0.0, 0.0, True)


def test_pure_pursuit_within_goal_tolerance_stops():
    v, w, reached = pure_pursuit(Pose(x=1.0, y=1.0), [(1.05, 1.0)])
    assert (v, w, reached) == (0.0, 0.0, True)


def test_pure_pursuit_drives_straight_toward_point_ahead():
    v, w, reached = pure_pursuit(Pose(), [(1.0, 0.0)], lookahead=0.35)
    assert not reached
    assert v > 0.0
    assert w == pytest.approx(0.0, abs=1e-9)


def test_pure_pursuit_pivots_when_target_behind():
    v, w, reached = pure_pursuit(Pose(), [(-1.0, 0.0)], lookahead=0.35)
    assert not reached
    assert v == 0.0
    assert w != 0.0


def test_pure_pursuit_turns_left_for_point_on_left():
    v, w, reached = pure_pursuit(Pose(), [(1.0, 1.0)], lookahead=0.35)
    assert not reached
    assert w > 0.0


def test_safety_filter_passes_when_clear():
    ranges = [3.0] * 360
    v, w, blocked = safety_filter(0.18, 0.0, ranges)
    assert (v, w, blocked) == (0.18, 0.0, False)


def test_safety_filter_stops_when_front_blocked():
    ranges = [3.0] * 360
    ranges[180] = 0.1
    v, w, blocked = safety_filter(0.18, 0.0, ranges)
    assert v == 0.0
    assert blocked is True


def test_safety_filter_ignores_side_obstacle():
    ranges = [3.0] * 360
    ranges[90] = 0.05  # 왼쪽, 정면 아님
    v, w, blocked = safety_filter(0.18, 0.0, ranges)
    assert (v, blocked) == (0.18, False)


def test_safety_filter_no_ranges_passes_through():
    v, w, blocked = safety_filter(0.18, 0.5, None)
    assert (v, w, blocked) == (0.18, 0.5, False)


def test_safety_filter_reversing_ignores_front():
    ranges = [3.0] * 360
    ranges[180] = 0.1
    v, w, blocked = safety_filter(-0.1, 0.0, ranges)
    assert (v, blocked) == (-0.1, False)


def test_lookahead_point_math_matches_pivot_threshold():
    # 정면에서 60도 벗어난 목표는 회전만, 45도는 전진 허용
    _, w60, reached60 = pure_pursuit(
        Pose(), [(math.cos(math.radians(60)), math.sin(math.radians(60)))]
    )
    v45, _, reached45 = pure_pursuit(
        Pose(), [(math.cos(math.radians(45)), math.sin(math.radians(45)))]
    )
    assert not reached60 and not reached45
    assert v45 > 0.0
