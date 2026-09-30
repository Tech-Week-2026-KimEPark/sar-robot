"""planner 공개 함수 테스트.

경로 안전성과 최단 비용은 tests/planner_checks.py의 독립 검사기로 확인한다.
planner 내부 함수는 호출하지 않는다.
"""

import math
from itertools import pairwise

import numpy as np
import pytest
from planner_checks import (
    cells_cost,
    free_points,
    make_map,
    path_length,
    path_violations,
    random_map,
    rect,
    reference_cost,
    reference_costs,
    two_rooms,
)

from sar import config, planner
from sar.grid_map import GridMap, simulate_scan

# ---------------------------------------------------------------- 기본 동작


def test_plan_goes_through_door():
    gm = two_rooms()
    path = planner.plan(gm, (-2.0, 1.5), (2.0, 1.5))
    assert path is not None
    assert path[0] == (-2.0, 1.5) and path[-1] == (2.0, 1.5)
    assert any(abs(x) < 0.1 and abs(y) < 0.4 for x, y in path)
    assert path_violations(gm, path) == []
    assert max(math.dist(a, b) for a, b in pairwise(path)) <= config.PATH_STEP + 1e-9


def test_plan_returns_none_when_unreachable():
    gm = two_rooms(door=False)
    assert planner.plan(gm, (-2.0, 1.5), (2.0, 1.5)) is None


@pytest.mark.parametrize(
    "start, goal",
    [
        (None, (1.0, 1.0)),
        ((0.0, 0.0), None),
        ((math.nan, 0.0), (1.0, 1.0)),
        ((-2.0, 1.5), (math.inf, 0.0)),
        ((-2.0, 1.5), (50.0, 0.0)),  # 지도 밖
        ((-50.0, 1.5), (2.0, 1.5)),
    ],
)
def test_plan_invalid_inputs_return_none(start, goal):
    assert planner.plan(two_rooms(), start, goal) is None


def test_plan_none_grid_and_empty_map():
    assert planner.plan(None, (0.0, 0.0), (1.0, 1.0)) is None
    assert planner.plan(GridMap(0.0, 0.0, size=4.0), (0.0, 0.0), (1.0, 1.0)) is None


def test_same_cell_start_and_goal():
    gm = two_rooms()
    start, goal = (-2.01, 1.01), (-2.03, 1.04)
    assert gm.to_cell(*start) == gm.to_cell(*goal)
    path = planner.plan(gm, start, goal)
    assert path[0] == start and path[-1] == goal
    assert path_length(path) == pytest.approx(math.dist(start, goal))


def test_same_point_start_and_goal():
    path = planner.plan(two_rooms(), (-2.0, 1.0), (-2.0, 1.0))
    assert path is not None and path[0] == (-2.0, 1.0) and path[-1] == (-2.0, 1.0)


def test_map_border_cells():
    # 지도 가장자리 칸까지 빈칸인 지도: 경계 칸을 시작·목표로 계획
    gm = make_map(4.0)
    lo, hi = gm.to_world(0, 0), gm.to_world(gm.n - 1, gm.n - 1)
    path = planner.plan(gm, lo, hi)
    assert path is not None and path[0] == lo and path[-1] == hi
    assert path_violations(gm, path) == []


def test_unknown_cells_need_allow_unknown():
    gm = two_rooms()
    known = np.ones((gm.n, gm.n), bool)
    known[:, gm.to_cell(0.1, 0.0)[1] :] = False  # 오른쪽 방은 모름
    gm = make_map(8.0, gm.logodds > config.OCC_THRESHOLD, known)
    assert planner.plan(gm, (-2.0, 1.5), (2.0, 1.5)) is None
    path = planner.plan(gm, (-2.0, 1.5), (2.0, 1.5), allow_unknown=True)
    assert path is not None and path[-1] == (2.0, 1.5)
    assert path_violations(gm, path, allow_unknown=True) == []
    assert path_violations(gm, path, allow_unknown=False) != []


def test_path_length_helper():
    assert planner.path_length(None) is None
    assert planner.path_length([(0.0, 0.0), (3.0, 4.0), (3.0, 5.0)]) == pytest.approx(6.0)


# ---------------------------------------------------------------- 최종 경로 안전성


@pytest.mark.parametrize("smooth", [True, False])
def test_random_maps_paths_are_safe(monkeypatch, smooth):
    # 다듬기와 시작·끝 좌표 교체 뒤에도 선분이 지나는 모든 칸이 통과 가능하고 모서리 통과 없음
    monkeypatch.setattr(config, "PATH_SMOOTH", smooth)
    checked = 0
    for seed in range(40):
        gm = random_map(seed)
        pts = free_points(gm, seed, 12)
        for a, b in zip(pts[::2], pts[1::2], strict=True):
            path = planner.plan(gm, a, b)
            if path is None:
                continue
            checked += 1
            assert path[0] == a and path[-1] == b
            assert path_violations(gm, path) == [], f"seed {seed} {a} -> {b}"
    assert checked > 100


def test_smoothing_never_lengthens_and_stays_off_walls(monkeypatch):
    gm = two_rooms()
    a, b = (-2.5, -1.5), (-0.5, 1.5)
    smooth = planner.plan(gm, a, b)
    monkeypatch.setattr(config, "PATH_SMOOTH", False)
    raw = planner.plan(gm, a, b)
    assert path_length(smooth) <= path_length(raw) + 1e-9
    clear = gm.clearance()
    assert min(clear[gm.to_cell(x, y)] for x, y in smooth) >= config.INFLATE


def test_narrow_door_and_corridor():
    # 빈칸 7개(0.35 m) 문: 가운데 칸의 장애물 거리 0.20 m로 통과 가능 (통과 가능 칸 1줄)
    # 빈칸 6개(0.30 m) 문: 가장 먼 칸도 장애물 거리 0.15 m로 통과 불가
    for width, reachable in ((0.34, True), (0.29, False)):
        gm = GridMap(0.0, 0.0, size=6.0)
        wall = np.zeros((gm.n, gm.n), bool)
        wall[rect(gm, -2.5, -2.5, 2.5, -2.5)] = wall[rect(gm, -2.5, 2.5, 2.5, 2.5)] = True
        wall[rect(gm, -2.5, -2.5, -2.5, 2.5)] = wall[rect(gm, 2.5, -2.5, 2.5, 2.5)] = True
        wall[rect(gm, 0.0, -2.5, 0.0, 2.5)] = True
        wall[rect(gm, 0.0, 0.0, 0.0, width)] = False
        gm = make_map(6.0, wall)
        path = planner.plan(gm, (-1.5, -1.5), (1.5, 1.5))
        assert (path is not None) == reachable, f"문 폭 {width}"
        if path is not None:
            assert path_violations(gm, path) == []


def _wall_with_strip(strip_half: float) -> GridMap:
    """x = 0 세로 벽(문 없음)과 벽 왼쪽의 세로 장애물(y ±strip_half).

    벽과 장애물 사이 빈칸 6개(0.30 m)는 양쪽 팽창으로 모두 통과 불가.
    """
    gm = GridMap(0.0, 0.0, size=6.0)
    wall = np.zeros((gm.n, gm.n), bool)
    wall[rect(gm, -2.5, -2.5, 2.5, -2.5)] = wall[rect(gm, -2.5, 2.5, 2.5, 2.5)] = True
    wall[rect(gm, -2.5, -2.5, -2.5, 2.5)] = wall[rect(gm, 2.5, -2.5, 2.5, 2.5)] = True
    wall[rect(gm, 0.0, -2.5, 0.0, 2.5)] = True
    wall[rect(gm, -0.45, -strip_half, -0.31, strip_half)] = True
    return make_map(6.0, wall)


def test_start_snap_does_not_cross_wall():
    # 시작점은 벽 왼쪽 팽창 영역 안. 가장 가까운 통과 가능 칸은 벽 반대편(오른쪽)
    gm = _wall_with_strip(0.2)
    start = (-0.12, 0.0)
    _, blocked, _, _ = gm.layers()
    assert blocked[gm.to_cell(*start)]
    # 벽 반대편 목표: 실제로 도달 불가이므로 None
    assert planner.plan(gm, start, (1.5, 1.0)) is None
    # 같은 쪽 목표: 같은 쪽 빈칸으로 탈출. 탈출 구간도 장애물 칸을 지나지 않음
    path = planner.plan(gm, start, (-1.5, 1.0))
    assert path is not None and path[0] == start
    assert all(x < 0.0 for x, _ in path)
    assert path_violations(gm, path, escape_head=True) == []


def test_start_snap_fails_when_only_far_side_is_free():
    # 같은 쪽 빈칸이 SNAP_RADIUS 밖이면 벽 반대편 칸을 쓰지 않고 실패
    gm = _wall_with_strip(0.6)
    assert planner.plan(gm, (-0.12, 0.0), (1.5, 1.0)) is None
    assert planner.plan(gm, (-0.12, 0.0), (-1.5, 1.0)) is None


def test_goal_snap_does_not_cross_wall():
    gm = _wall_with_strip(0.2)
    goal = (-0.12, 0.0)  # 벽 왼쪽 팽창 영역 안
    # 로봇이 벽 오른쪽: 목표 쪽 빈칸에 도달할 수 없으므로 None (벽 반대편에서 끝나는 경로 금지)
    assert planner.plan(gm, (1.5, 1.0), goal) is None
    # 로봇이 벽 왼쪽: 목표와 같은 쪽의 대체 칸 중심에서 끝남
    path = planner.plan(gm, (-1.5, 1.0), goal)
    assert path is not None and path[-1] != goal
    assert path[-1][0] < 0.0 and math.dist(path[-1], goal) <= config.SNAP_RADIUS
    assert path_violations(gm, path) == []


def test_start_inside_inflation_escapes_without_touching_obstacle():
    gm = two_rooms()
    start = (-3.0 + 0.10, 0.0)  # 외벽에서 0.1 m (팽창 영역 안, 장애물 칸 아님)
    path = planner.plan(gm, start, (-1.0, 0.0))
    assert path is not None and path[0] == start
    assert path_violations(gm, path, escape_head=True) == []
    assert path_violations(gm, path) != []  # 탈출 구간 허용 없이는 위반으로 판정되는 경로


def test_start_on_obstacle_cell_escapes_away_from_wall():
    # 지도 오차로 로봇 위치가 장애물 칸으로 표시된 경우: 그 장애물 구간만 벗어나는 탈출 허용
    gm = two_rooms()
    start = (-3.0 + 0.04, 0.0)  # 외벽 칸 안. 방 안쪽 빈칸이 바깥쪽 빈칸보다 가까운 위치
    assert gm.logodds[gm.to_cell(*start)] > config.OCC_THRESHOLD
    path = planner.plan(gm, start, (-1.0, 0.0))
    assert path is not None
    assert all(x >= start[0] - 1e-9 for x, _ in path)  # 벽 바깥쪽으로 나가지 않음


def test_goal_inside_inflation_ends_at_nearest_free_cell():
    gm = two_rooms()
    goal = (-2.0, 2.0 - 0.05)  # 벽 바로 앞
    path = planner.plan(gm, (-2.0, 0.0), goal)
    assert path is not None and path[-1] != goal
    assert math.dist(path[-1], goal) <= config.SNAP_RADIUS
    assert path_violations(gm, path) == []


def test_distance_field_path_follows_same_safety_policy():
    gm = two_rooms()
    # 기준점이 팽창 영역 안: 끝점을 기준점 좌표로 바꾸지 않고 대체 칸 중심에서 끝남
    origin = (-2.0, 1.95)
    field = planner.DistanceField(gm, origin)
    path = field.path((-2.0, 0.0))
    assert path is not None and path[-1] != origin
    assert path_violations(gm, path) == []
    # 출발점이 팽창 영역 안: 탈출 구간 뒤 안전
    home = planner.DistanceField(gm, (2.0, 0.0))
    start = (-3.0 + 0.10, 0.0)
    path = home.path(start)
    assert path[0] == start and path[-1] == (2.0, 0.0)
    assert path_violations(gm, path, escape_head=True) == []
    # 벽 반대편 칸으로 대체하지 않음
    strip = _wall_with_strip(0.6)
    assert planner.DistanceField(strip, (1.5, 1.0)).path((-0.12, 0.0)) is None
    assert planner.DistanceField(strip, (1.5, 1.0)).distance((-0.12, 0.0)) is None


def test_distance_field_paths_safe_on_random_maps():
    for seed in range(12):
        gm = random_map(seed)
        pts = free_points(gm, seed + 100, 6)
        field = planner.DistanceField(gm, pts[0])
        for p in pts[1:]:
            path = field.path(p)
            if path is not None:
                assert path[0] == p and path[-1] == pts[0]
                assert path_violations(gm, path) == [], f"seed {seed}"


# ---------------------------------------------------------------- 최단 비용 (독립 기준과 비교)


def test_plan_cost_matches_independent_dijkstra(monkeypatch):
    monkeypatch.setattr(config, "PATH_SMOOTH", False)
    for seed in range(6):
        gm = random_map(seed, size=4.0, blocks=8)
        pts = free_points(gm, seed, 8)
        costs = reference_costs(gm, gm.to_cell(*pts[0]))
        for goal in pts[1:]:
            path = planner.plan(gm, pts[0], goal)
            want = costs.get(gm.to_cell(*goal), math.inf)
            if math.isinf(want):
                assert path is None
            else:
                assert cells_cost(gm, path) == pytest.approx(want, rel=1e-5)


def test_distance_field_matches_independent_dijkstra():
    gm = random_map(3, size=4.0, blocks=8)
    pts = free_points(gm, 3, 8)
    field = planner.DistanceField(gm, pts[0])
    costs = reference_costs(gm, gm.to_cell(*pts[0]), allow_unknown=True)
    for p in pts[1:]:
        want = costs.get(gm.to_cell(*p))
        got = field.distance(p)
        if want is None:
            assert got is None
        else:
            assert got == pytest.approx(want * gm.res, rel=1e-5)
    assert field.distance(pts[0]) == pytest.approx(0.0, abs=1e-9)


def test_distance_is_weighted_cost_not_length():
    # 벽 근처 비용 때문에 가중 비용은 실제 경로 길이 이상
    gm = two_rooms()
    field = planner.DistanceField(gm, (2.0, 0.0))
    path = field.path((-2.0, 0.0))
    assert field.distance((-2.0, 0.0)) >= planner.path_length(path) - gm.res


# ---------------------------------------------------------------- DistanceField 재사용 조건


def _ring_map() -> GridMap:
    """가운데 큰 장애물을 둘러싼 고리형 통로. 왼쪽 통로가 오른쪽보다 짧음."""
    gm = GridMap(0.0, 0.0, size=6.0)
    wall = np.zeros((gm.n, gm.n), bool)
    wall[rect(gm, -2.5, -2.5, 2.5, -2.5)] = wall[rect(gm, -2.5, 2.5, 2.5, 2.5)] = True
    wall[rect(gm, -2.5, -2.5, -2.5, 2.5)] = wall[rect(gm, 2.5, -2.5, 2.5, 2.5)] = True
    wall[rect(gm, -1.0, -1.5, 1.8, 1.5)] = True
    return make_map(6.0, wall)


def test_field_with_matching_goal_keeps_optimal_cost(monkeypatch):
    monkeypatch.setattr(config, "PATH_SMOOTH", False)
    gm = _ring_map()
    start, goal = (-1.4, -2.0), (-1.4, 2.0)
    want = reference_cost(gm, gm.to_cell(*start), gm.to_cell(*goal))
    field = planner.DistanceField(gm, goal)
    assert field.is_current()
    path = planner.plan(gm, start, goal, field=field)
    assert cells_cost(gm, path) == pytest.approx(want, rel=1e-5)


def test_field_for_another_goal_is_ignored(monkeypatch):
    # 다른 목표 기준 거리 지도는 휴리스틱으로 쓰지 않음. 결과는 독립 기준의 최단 비용과 같아야 함
    monkeypatch.setattr(config, "PATH_SMOOTH", False)
    gm = _ring_map()
    start, goal = (-1.4, -2.0), (-1.4, 2.0)
    want = reference_cost(gm, gm.to_cell(*start), gm.to_cell(*goal))
    for wrong_origin in ((2.2, 0.0), (2.2, -2.0), (-2.2, -2.2)):
        wrong = planner.DistanceField(gm, wrong_origin)
        path = planner.plan(gm, start, goal, field=wrong)
        assert path is not None and path[-1] == goal
        assert cells_cost(gm, path) == pytest.approx(want, rel=1e-5)


def test_stale_field_is_ignored_and_its_path_refused(monkeypatch):
    monkeypatch.setattr(config, "PATH_SMOOTH", False)
    gm = two_rooms()
    start, goal = (-2.0, 1.5), (2.0, 1.5)
    field = planner.DistanceField(gm, goal)
    assert field.path(start) is not None
    before = field.distance(start)
    # 문 아래쪽 절반을 닫음: 지도 객체는 같고 내용만 변경
    gm.logodds[rect(gm, 0.0, -0.4, 0.0, 0.0)] = config.L_MAX
    gm.invalidate()
    assert not field.is_current()
    assert field.path(start) is None  # 오래된 경로는 반환하지 않음
    assert field.distance(start) == before  # 비용은 생성 시점 값(근삿값)으로 유지
    path = planner.plan(gm, start, goal, field=field)
    want = reference_cost(gm, gm.to_cell(*start), gm.to_cell(*goal))
    assert cells_cost(gm, path) == pytest.approx(want, rel=1e-5)
    assert path_violations(gm, path) == []
    # 문을 완전히 닫으면 오래된 field가 있어도 경로 없음
    gm.logodds[rect(gm, 0.0, -0.4, 0.0, 0.4)] = config.L_MAX
    gm.invalidate()
    assert planner.plan(gm, start, goal, field=field) is None


def test_field_stays_current_when_only_log_odds_change():
    gm = two_rooms()
    walls = gm.logodds > config.OCC_THRESHOLD
    pose = (-1.5, 0.0, 0.0)
    scan = simulate_scan(walls, gm.to_cell, pose)
    gm.update(pose, scan)  # 첫 갱신: 분류가 바뀔 수 있음
    field = planner.DistanceField(gm, (-2.0, 1.0))
    version = gm.plan_version
    gm.update(pose, scan)  # 같은 스캔: log-odds만 변하고 분류는 유지
    assert gm.plan_version == version and field.is_current()
    assert field.path((-1.0, -1.0)) is not None


def test_field_with_smaller_area_is_ignored(monkeypatch):
    # 거리 지도 영역 밖에서 출발: 영역 밖 조회값을 도달 불가로 처리하면 탐색이 실패함
    monkeypatch.setattr(config, "PATH_SMOOTH", False)
    gm = GridMap(0.0, 0.0, size=12.0)
    known = np.zeros((gm.n, gm.n), bool)
    known[rect(gm, -1.0, -1.0, 1.0, 1.0)] = True
    gm = make_map(12.0, None, known)
    goal, start = (0.0, 0.0), (4.5, 4.5)  # 시작점은 확인 영역 + 여백 밖 (모르는 칸)
    field = planner.DistanceField(gm, goal)
    path = planner.plan(gm, start, goal, allow_unknown=True, field=field)
    assert path is not None and path[0] == start and path[-1] == goal
    want = reference_cost(gm, gm.to_cell(*start), gm.to_cell(*goal), allow_unknown=True)
    assert cells_cost(gm, path, allow_unknown=True) == pytest.approx(want, rel=1e-5)


def test_field_ignored_after_cost_config_change(monkeypatch):
    gm = two_rooms()
    field = planner.DistanceField(gm, (2.0, 1.5))
    monkeypatch.setattr(config, "WALL_COST", 5.0)
    assert not field.is_current()
    assert planner.plan(gm, (-2.0, 1.5), (2.0, 1.5), field=field) is not None


def test_distance_field_invalid_origin():
    gm = two_rooms(door=False)
    for origin in ((50.0, 0.0), (math.nan, 0.0)):
        field = planner.DistanceField(gm, origin)
        assert field.distance((0.5, 0.5)) is None and field.path((0.5, 0.5)) is None
    walled = planner.DistanceField(gm, (-2.0, 0.0))
    assert walled.distance((2.0, 0.0)) is None and walled.path((2.0, 0.0)) is None
    assert walled.distance((math.nan, 0.0)) is None


# ---------------------------------------------------------------- 프론티어 선택


def frontier_map(walls: bool = False) -> GridMap:
    """8 m 지도. 확인한 영역은 x -1~1, y -1~1. walls이면 오른쪽 변만 열린 방."""
    gm = GridMap(0.0, 0.0, size=8.0)
    known = np.zeros((gm.n, gm.n), bool)
    known[rect(gm, -1.0, -1.0, 1.0, 1.0)] = True
    wall = np.zeros((gm.n, gm.n), bool)
    if walls:
        wall[rect(gm, -1.0, -1.0, 1.0, -1.0)] = wall[rect(gm, -1.0, 1.0, 1.0, 1.0)] = True
        wall[rect(gm, -1.0, -1.0, -1.0, 1.0)] = True
    return make_map(8.0, wall, known)


def test_choose_frontier_targets_open_side():
    target = planner.choose_frontier(frontier_map(walls=True), (0.0, 0.0, 0.0), [])
    assert target is not None and target[0] == pytest.approx(1.0, abs=0.05)


def test_choose_frontier_respects_blacklist():
    gm = frontier_map()
    first = planner.choose_frontier(gm, (0.0, 0.0, 0.0), [])
    second = planner.choose_frontier(gm, (0.0, 0.0, 0.0), [first])
    assert second is not None
    assert math.dist(first, second) > config.BLACKLIST_RADIUS
    assert planner.choose_frontier(gm, (0.0, 0.0, 0.0), [first, (math.nan, 0.0)]) == second


def _best_frontier_score(gm: GridMap, pose) -> float:
    """독립 다익스트라 기준 최고 점수 (묶음 칸 수 / 경로 비용 [m])."""
    costs = reference_costs(gm, gm.to_cell(pose[0], pose[1]))
    best = 0.0
    for size, cells in gm.frontiers():
        for cell in cells:
            d = costs.get(cell, math.inf) * gm.res
            if config.FRONTIER_MIN_DIST <= d < math.inf:
                best = max(best, size / d)
    return best


@pytest.mark.parametrize("seed", range(6))
def test_choose_frontier_early_stop_matches_exhaustive_search(monkeypatch, seed):
    # 조기 종료 결과의 점수가 모든 프론티어 칸을 확인한 최고 점수와 같음. 경로 비용도 최단
    monkeypatch.setattr(config, "PATH_SMOOTH", False)
    full = random_map(seed, size=5.0, blocks=10)
    known = np.zeros((full.n, full.n), bool)
    known[rect(full, -2.5, -2.5, 0.4 * (seed % 3), 2.5)] = True
    gm = make_map(5.0, full.logodds > config.OCC_THRESHOLD, known)
    pose = free_points(gm, seed, 1)[0] + (0.0,)
    found = planner.choose_frontier_path(gm, pose, [])
    best = _best_frontier_score(gm, pose)
    if found is None:
        assert best == 0.0
        return
    target, path = found
    sizes = {cell: size for size, cells in gm.frontiers() for cell in cells}
    cost = reference_cost(gm, gm.to_cell(pose[0], pose[1]), gm.to_cell(*target))
    assert sizes[gm.to_cell(*target)] / (cost * gm.res) == pytest.approx(best, rel=1e-5)
    assert path[0] == pose[:2] and path[-1] == target
    assert cells_cost(gm, path) == pytest.approx(cost, rel=1e-5)
    assert path_violations(gm, path) == []
    assert planner.choose_frontier(gm, pose, []) == target


def test_choose_frontier_path_is_safe_with_smoothing():
    for seed in range(10):
        full = random_map(seed, size=5.0, blocks=10)
        known = np.zeros((full.n, full.n), bool)
        known[rect(full, -2.5, -2.5, 0.5, 2.5)] = True
        gm = make_map(5.0, full.logodds > config.OCC_THRESHOLD, known)
        pose = free_points(gm, seed, 1)[0] + (0.0,)
        found = planner.choose_frontier_path(gm, pose, [])
        if found is not None:
            assert path_violations(gm, found[1]) == [], f"seed {seed}"


def test_choose_frontier_skips_unreachable_clusters():
    # 문 없는 칸막이 너머의 프론티어는 크기가 커도 선택하지 않음
    gm = GridMap(0.0, 0.0, size=8.0)
    wall = np.zeros((gm.n, gm.n), bool)
    wall[rect(gm, 0.0, -3.5, 0.0, 3.5)] = True
    known = np.zeros((gm.n, gm.n), bool)
    known[rect(gm, -2.0, -0.5, 3.0, 0.5)] = True
    known[rect(gm, 0.0, -3.5, 3.0, 3.5)] = True  # 칸막이 오른쪽은 넓게 확인 (큰 프론티어)
    gm = make_map(8.0, wall, known)
    target = planner.choose_frontier(gm, (-1.0, 0.0, 0.0), [])
    assert target is not None and target[0] < 0.0


def test_choose_frontier_none_cases():
    gm = two_rooms()  # 전부 확인, 프론티어 없음
    assert planner.choose_frontier(gm, (0.5, 1.0, 0.0), []) is None
    assert planner.choose_frontier_path(gm, (0.5, 1.0, 0.0), []) is None
    assert planner.choose_frontier(None, (0.0, 0.0, 0.0), []) is None
    assert planner.choose_frontier(frontier_map(), None, []) is None
    assert planner.choose_frontier(frontier_map(), (math.nan, 0.0, 0.0), []) is None
    assert planner.choose_frontier(frontier_map(), (50.0, 0.0, 0.0), []) is None
