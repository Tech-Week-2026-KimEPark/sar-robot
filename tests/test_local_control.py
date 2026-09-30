import math

import pytest

from sar import config
from sar.local_control import (
    _FRONT_HALF_ANGLE,
    _SLOW_DIST,
    _YIELD_DIST,
    predict_conflict,
    pure_pursuit,
    safety_filter,
    yield_command,
)


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


def test_safety_filter_slows_down_before_stop_dist():
    # 대피 인원 이동 대응(과제와 구현 기준 12장): 정지 거리 도달 전부터 서서히 감속
    ranges = [3.0] * 360
    mid = (config.STOP_DIST + _SLOW_DIST) / 2
    ranges[180] = mid
    v, w, blocked = safety_filter(config.V_MAX, 0.0, ranges)
    assert not blocked
    assert 0.0 < v < config.V_MAX


def test_safety_filter_no_slowdown_beyond_slow_dist():
    ranges = [3.0] * 360
    ranges[180] = _SLOW_DIST + 0.01
    assert safety_filter(config.V_MAX, 0.0, ranges) == (config.V_MAX, 0.0, False)


def test_safety_filter_full_stop_scales_to_zero():
    ranges = [3.0] * 360
    ranges[180] = config.STOP_DIST + 1e-6
    v, w, blocked = safety_filter(config.V_MAX, 0.0, ranges)
    assert not blocked
    assert v == pytest.approx(0.0, abs=1e-3)


def test_safety_filter_yields_toward_open_side():
    # 오른쪽(270 방향)만 막혀 있으면 왼쪽으로 살짝 틀어 통로를 양보함(w 증가).
    # 180은 좌우 콘의 경계라 왼쪽에도 포함되므로 건드리지 않음
    ranges = [3.0] * 360
    ranges[200] = (config.STOP_DIST + _SLOW_DIST) / 2  # 오른쪽 방향만 좁음
    v, w, blocked = safety_filter(config.V_MAX, 0.0, ranges)
    assert not blocked
    assert w > 0.0


def test_safety_filter_yield_clips_to_w_max():
    ranges = [3.0] * 360
    ranges[200] = config.STOP_DIST + 0.01  # 오른쪽만 좁아서 양보 편향이 생김
    v, w, blocked = safety_filter(config.V_MAX, config.W_MAX, ranges)
    assert abs(w) <= config.W_MAX + 1e-9


# --- predict_conflict: 사람-회피-설계.md 3.3절 ---


def test_predict_conflict_head_on_approach():
    # 정면으로 마주 옴 (넓은 방), 예측 시간 안에 만남: 최근접 거리가 0에 가까움
    person = {"x": 1.0, "y": 0.0, "vx": -0.3, "vy": 0.0}
    t_star, d_min = predict_conflict((0.0, 0.0, 0.0), config.V_MAX, person)
    assert t_star > 0.0
    assert d_min < 0.05


def test_predict_conflict_moving_away_clips_to_now():
    person = {"x": 1.0, "y": 0.0, "vx": 0.3, "vy": 0.0}
    t_star, d_min = predict_conflict((0.0, 0.0, 0.0), 0.0, person)
    assert t_star == 0.0
    assert d_min == pytest.approx(1.0)


def test_predict_conflict_crossing_sideways():
    # 옆에서 가로지름: 예측 거리가 현재 거리보다 가까워짐
    person = {"x": 0.0, "y": 2.0, "vx": 0.0, "vy": -0.3}
    t_star, d_min = predict_conflict((0.0, 0.0, 0.0), config.V_MAX, person)
    assert t_star > 0.0
    assert d_min < 2.0


def test_predict_conflict_approaching_from_behind():
    # 뒤에서 다가옴: 상대 위치가 로봇 뒤(-x)라도 예측이 동작하고 거리가 좁혀짐
    person = {"x": -1.0, "y": 0.0, "vx": 0.3, "vy": 0.0}
    t_star, d_min = predict_conflict((0.0, 0.0, 0.0), 0.0, person)
    assert t_star > 0.0
    assert d_min < 1.0


def test_predict_conflict_zero_relative_speed_keeps_current_distance():
    person = {"x": 1.0, "y": 0.0, "vx": config.V_MAX, "vy": 0.0}
    t_star, d_min = predict_conflict((0.0, 0.0, 0.0), config.V_MAX, person)
    assert t_star == 0.0
    assert d_min == pytest.approx(1.0)


# --- yield_command: 사람-회피-설계.md 3.4절 ---


def test_yield_command_moving_away_is_done():
    person = {"x": 1.0, "y": 0.0, "vx": 0.3, "vy": 0.0}
    v, w, done = yield_command((0.0, 0.0, 0.0), [3.0] * 360, person)
    assert done
    assert (v, w) == (0.0, 0.0)


def test_yield_command_stationary_person_is_done():
    person = {"x": 1.0, "y": 0.0, "vx": 0.0, "vy": 0.0}
    v, w, done = yield_command((0.0, 0.0, 0.0), [3.0] * 360, person)
    assert done


def test_yield_command_open_space_steps_aside():
    person = {"x": 1.0, "y": 0.0, "vx": -0.3, "vy": 0.0}
    v, w, done = yield_command((0.0, 0.0, 0.0), [3.0] * 360, person)
    assert not done
    assert v > 0.0 or w != 0.0


def test_yield_command_threshold_switches_between_back_and_step_aside():
    threshold = _YIELD_DIST + config.ROBOT_RADIUS + config.SAFETY_MARGIN
    person = {"x": 1.0, "y": 0.0, "vx": -0.3, "vy": 0.0}

    narrow = [threshold - 0.01] * 360
    v_narrow, _, _ = yield_command((0.0, 0.0, 0.0), narrow, person)
    assert v_narrow < 0.0

    open_ranges = [threshold + 0.01] * 360
    v_open, _, _ = yield_command((0.0, 0.0, 0.0), open_ranges, person)
    assert v_open >= 0.0


def test_yield_command_narrow_corridor_backs_away():
    # 좌우 모두 STOP_DIST 근처로 막혀 있으면(복도) 사람 쪽으로 후진
    ranges = [config.STOP_DIST] * 360
    person = {"x": 1.0, "y": 0.0, "vx": -0.3, "vy": 0.0}
    v, w, done = yield_command((0.0, 0.0, 0.0), ranges, person)
    assert not done
    assert v < 0.0


def test_yield_command_picks_more_open_side():
    # 사람이 +y 방향으로 이동(왼쪽에서 오른쪽) -> 좌우 비키는 방향은 진행 방향에 수직
    # 오른쪽(로봇 -y 방향)만 넓게 열어두면 그쪽으로 비켜서야 함
    ranges = [0.0] * 360  # 우선 전부 막힘으로 채운 뒤 한쪽만 열어둠
    for i in range(len(ranges)):
        ranges[i] = config.STOP_DIST
    ranges[90] = 3.0  # 로봇 기준 왼쪽(90) 방향만 열림
    person = {"x": 1.0, "y": 0.0, "vx": -0.3, "vy": 0.0}
    v, w, done = yield_command((0.0, 0.0, 0.0), ranges, person)
    assert not done
    # 완전히 막히지 않았으므로 후진(v<0)이 아니라 비켜서는 동작(v>0 또는 회전)이어야 함
    assert not (v < 0.0)
