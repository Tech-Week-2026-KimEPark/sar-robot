import math

import pytest

from sar import config
from sar.local_control import _FRONT_HALF_ANGLE, pure_pursuit, safety_filter


def test_pure_pursuit_no_path_is_reached():
    assert pure_pursuit((0.0, 0.0, 0.0), []) == (0.0, 0.0, True)


def test_pure_pursuit_within_goal_tolerance_stops():
    assert pure_pursuit((1.0, 1.0, 0.0), [(1.05, 1.0)]) == (0.0, 0.0, True)


def test_pure_pursuit_drives_straight_toward_point_ahead():
    v, w, reached = pure_pursuit((0.0, 0.0, 0.0), [(1.0, 0.0)])
    assert not reached
    assert v == pytest.approx(config.V_MAX)
    assert w == pytest.approx(0.0, abs=1e-9)


def test_pure_pursuit_pivots_when_target_behind():
    v, w, reached = pure_pursuit((0.0, 0.0, 0.0), [(-1.0, 0.0)])
    assert not reached
    assert v == 0.0
    assert w == pytest.approx(config.W_MAX)


def test_pure_pursuit_turns_left_for_point_on_left():
    v, w, reached = pure_pursuit((0.0, 0.0, 0.0), [(1.0, 1.0)])
    assert not reached
    assert w > 0.0


def test_pure_pursuit_uses_robot_heading():
    # 로봇이 서쪽(pi)을 보고 있으면 동쪽의 목표는 정면이 아니라 뒤쪽이 됨
    v, w, reached = pure_pursuit((0.0, 0.0, math.pi), [(1.0, 0.0)])
    assert not reached
    assert v == 0.0
    assert w != 0.0


def test_pure_pursuit_ignores_passed_points_behind_robot():
    # sar-robot-병합-분석.md 6.2절 재현: 0.5m 진행한 상태에서 지나온 시작점을
    # 목표로 선택하면 제자리 회전만 하고 전진하지 못함
    path = [(0.0, 0.0), (0.25, 0.0), (0.5, 0.0), (0.75, 0.0), (1.0, 0.0), (2.0, 0.0)]
    pose = (0.5, 0.0, 0.0)
    v, w, reached = pure_pursuit(pose, path)
    assert not reached
    assert v > 0.0
    assert w == pytest.approx(0.0, abs=1e-9)


def test_safety_filter_passes_when_clear():
    ranges = [3.0] * 360
    assert safety_filter(config.V_MAX, 0.0, ranges) == (config.V_MAX, 0.0, False)


def test_safety_filter_stops_when_front_blocked():
    ranges = [3.0] * 360
    ranges[180] = 0.1
    v, w, blocked = safety_filter(config.V_MAX, 0.0, ranges)
    assert v == 0.0
    assert blocked is True


def test_safety_filter_ignores_side_obstacle():
    ranges = [3.0] * 360
    ranges[90] = 0.05  # 왼쪽, 정면 아님
    v, w, blocked = safety_filter(config.V_MAX, 0.0, ranges)
    assert (v, blocked) == (config.V_MAX, False)


def test_safety_filter_no_ranges_passes_through():
    assert safety_filter(config.V_MAX, 0.5, None) == (config.V_MAX, 0.5, False)


def test_safety_filter_reversing_ignores_front():
    ranges = [3.0] * 360
    ranges[180] = 0.1
    assert safety_filter(-0.1, 0.0, ranges) == (-0.1, 0.0, False)


def test_safety_filter_reversing_checks_rear():
    ranges = [3.0] * 360
    ranges[0] = 0.1  # 후면 장애물
    v, w, blocked = safety_filter(-0.1, 0.0, ranges)
    assert v == 0.0
    assert blocked is True


def test_safety_filter_zero_velocity_skips_check():
    ranges = [3.0] * 360
    ranges[180] = 0.05
    assert safety_filter(0.0, 0.5, ranges) == (0.0, 0.5, False)


def test_safety_filter_cone_covers_robot_width_at_stop_dist():
    # 로봇 반지름(0.105m) + SAFETY_MARGIN(0.06m) = 0.165m를 STOP_DIST(0.20m)에서
    # 덮어야 함 (sar-robot-병합-분석.md 6.3절). 반각 25도(반폭 0.093m)는 부족했음.
    half_width = config.STOP_DIST * math.tan(_FRONT_HALF_ANGLE)
    assert half_width >= config.ROBOT_RADIUS + config.SAFETY_MARGIN - 1e-9
