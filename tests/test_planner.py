import math
from itertools import pairwise

import numpy as np
import pytest

from sar import config, planner
from sar.grid_map import GridMap


def two_rooms(door: bool = True) -> GridMap:
    """8 m 지도, x -3~3 / y -2~2 외벽, x = 0 칸막이, 칸막이 가운데 0.8 m 문."""
    gm = GridMap(0.0, 0.0, size=8.0)
    gm.seen[:] = True
    gm.logodds[:] = config.L_MIN
    wall = np.zeros_like(gm.seen)
    r0, c0 = gm.to_cell(-3.0, -2.0)
    r1, c1 = gm.to_cell(3.0, 2.0)
    wall[r0, c0 : c1 + 1] = wall[r1, c0 : c1 + 1] = True
    wall[r0 : r1 + 1, c0] = wall[r0 : r1 + 1, c1] = True
    mid = gm.to_cell(0.0, 0.0)[1]
    wall[r0 : r1 + 1, mid] = True
    if door:
        wall[gm.to_cell(0.0, -0.4)[0] : gm.to_cell(0.0, 0.4)[0] + 1, mid] = False
    gm.logodds[wall] = config.L_MAX
    gm._cache.clear()
    return gm


def path_cells_passable(gm: GridMap, path) -> bool:
    _, blocked, _, _ = gm.layers()
    return all(not blocked[gm.to_cell(x, y)] for x, y in path[1:-1])


def path_length(path) -> float:
    return sum(math.dist(a, b) for a, b in pairwise(path))


def test_plan_goes_through_door():
    gm = two_rooms()
    path = planner.plan(gm, (-2.0, 1.5), (2.0, 1.5))
    assert path is not None
    assert path[0] == (-2.0, 1.5) and path[-1] == (2.0, 1.5)
    assert any(abs(x) < 0.1 and abs(y) < 0.4 for x, y in path)
    assert path_cells_passable(gm, path)
    steps = [math.dist(a, b) for a, b in pairwise(path)]
    assert max(steps) <= config.PATH_STEP + 1e-9


def test_plan_returns_none_when_unreachable():
    gm = two_rooms(door=False)
    assert planner.plan(gm, (-2.0, 1.5), (2.0, 1.5)) is None


def test_plan_handles_none_and_outside_inputs():
    gm = two_rooms()
    assert planner.plan(None, (0, 0), (1, 1)) is None
    assert planner.plan(gm, None, (1, 1)) is None
    assert planner.plan(gm, (0, 0), None) is None
    assert planner.plan(gm, (-2.0, 1.5), (50.0, 0.0)) is None


def test_unknown_cells_need_allow_unknown():
    gm = two_rooms()
    # 오른쪽 방을 모름으로 변경
    gm.seen[:, gm.to_cell(0.1, 0.0)[1] :] = False
    gm._cache.clear()
    assert planner.plan(gm, (-2.0, 1.5), (2.0, 1.5)) is None
    path = planner.plan(gm, (-2.0, 1.5), (2.0, 1.5), allow_unknown=True)
    assert path is not None and path[-1] == (2.0, 1.5)


def test_start_inside_inflation_is_snapped():
    gm = two_rooms()
    start = (-3.0 + config.INFLATE - 0.05, 0.0)  # 외벽 팽창 영역 안
    path = planner.plan(gm, start, (-1.0, 0.0))
    assert path is not None and path[0] == start


def test_goal_inside_inflation_ends_at_nearest_free_cell():
    gm = two_rooms()
    goal = (-2.0, 2.0 - 0.05)  # 벽 바로 앞 사과
    path = planner.plan(gm, (-2.0, 0.0), goal)
    assert path is not None
    assert math.dist(path[-1], goal) <= config.SNAP_RADIUS
    assert path_cells_passable(gm, path + [path[-1]])


def test_astar_cost_matches_dijkstra():
    gm = two_rooms()
    s_cell, g_cell = gm.to_cell(-2.5, -1.5), gm.to_cell(2.5, 1.5)
    win = planner._Window(gm, [s_cell, g_cell], allow_unknown=False)
    s, g = win.index(*s_cell), win.index(*g_cell)
    astar, _ = planner._search(win, s, goal=g, heuristic=planner._octile(win, g))
    dijkstra, _ = planner._search(win, s)
    assert astar[g] == pytest.approx(dijkstra[g])


def test_field_heuristic_keeps_optimal_cost():
    gm = two_rooms()
    goal = (2.5, 1.5)
    field = planner.DistanceField(gm, goal)
    plain = planner.plan(gm, (-2.5, -1.5), goal)
    guided = planner.plan(gm, (-2.5, -1.5), goal, field=field)
    assert guided is not None
    assert path_length(guided) == pytest.approx(path_length(plain), rel=0.02)


def test_wall_cost_keeps_path_off_walls():
    gm = two_rooms()
    # 외벽을 따라가는 직선 대신 통로 가운데 쪽으로 이동
    path = planner.plan(gm, (-2.5, -1.6), (-0.5, -1.6))
    clear = gm.clearance()
    worst = min(clear[gm.to_cell(x, y)] for x, y in path)
    assert worst >= config.INFLATE


def test_smoothing_reduces_points_and_can_be_disabled(monkeypatch):
    gm = two_rooms()
    smooth = planner.plan(gm, (-2.5, -1.5), (-0.5, 1.5))
    monkeypatch.setattr(config, "PATH_SMOOTH", False)
    raw = planner.plan(gm, (-2.5, -1.5), (-0.5, 1.5))
    assert path_length(smooth) <= path_length(raw) + 1e-9


def test_distance_field_distance_and_path():
    gm = two_rooms()
    home = (-2.0, 0.0)
    field = planner.DistanceField(gm, home)
    assert field.distance(home) == pytest.approx(0.0, abs=gm.res)
    d = field.distance((2.0, 0.0))
    assert d is not None and d >= 4.0
    path = field.path((2.0, 0.0))
    assert path[0] == (2.0, 0.0) and path[-1] == home
    assert path_cells_passable(gm, path)
    walled = planner.DistanceField(two_rooms(door=False), home)
    assert walled.distance((2.0, 0.0)) is None
    assert walled.path((2.0, 0.0)) is None


def frontier_map() -> GridMap:
    """8 m 지도. 확인한 영역은 x -1~1, y -1~1 빈칸. 바깥은 모름."""
    gm = GridMap(0.0, 0.0, size=8.0)
    r0, c0 = gm.to_cell(-1.0, -1.0)
    r1, c1 = gm.to_cell(1.0, 1.0)
    gm.seen[r0 : r1 + 1, c0 : c1 + 1] = True
    gm.logodds[r0 : r1 + 1, c0 : c1 + 1] = config.L_MIN
    gm._cache.clear()
    return gm


def test_choose_frontier_prefers_large_near_cluster():
    gm = frontier_map()
    # 오른쪽 경계만 남기고 나머지 3면은 벽 (프론티어 1개 묶음)
    r0, c0 = gm.to_cell(-1.0, -1.0)
    r1, c1 = gm.to_cell(1.0, 1.0)
    gm.logodds[r0, c0 : c1 + 1] = gm.logodds[r1, c0 : c1 + 1] = config.L_MAX
    gm.logodds[r0 : r1 + 1, c0] = config.L_MAX
    gm._cache.clear()
    target = planner.choose_frontier(gm, (0.0, 0.0, 0.0), [])
    assert target is not None
    assert target[0] == pytest.approx(1.0, abs=gm.res)


def test_choose_frontier_respects_blacklist():
    gm = frontier_map()
    first = planner.choose_frontier(gm, (0.0, 0.0, 0.0), [])
    second = planner.choose_frontier(gm, (0.0, 0.0, 0.0), [first])
    assert second is not None
    assert math.dist(first, second) > config.BLACKLIST_RADIUS


def test_choose_frontier_score_matches_exhaustive_search():
    gm = frontier_map()
    # 조기 종료 결과가 전체 다익스트라 기준 최고 점수와 같은지 확인
    pose = (0.3, -0.2, 0.0)
    target = planner.choose_frontier(gm, pose, [])
    win = planner._Window(gm, [gm.to_cell(*pose[:2])], allow_unknown=False)
    dist, _ = planner._search(win, win.index(*gm.to_cell(*pose[:2])))
    best = 0.0
    for size, cells in gm.frontiers():
        for cell in cells:
            idx = win.index(*cell)
            d = dist[idx] if idx is not None else math.inf
            if d * gm.res >= config.FRONTIER_MIN_DIST and not math.isinf(d):
                best = max(best, size / (d * gm.res))
    idx = win.index(*gm.to_cell(*target))
    assert gm.frontiers()[0][0] / (dist[idx] * gm.res) == pytest.approx(best)


def test_choose_frontier_none_cases():
    gm = two_rooms()  # 전부 확인, 프론티어 없음
    assert planner.choose_frontier(gm, (0.0, 1.0, 0.0), []) is None
    assert planner.choose_frontier(None, (0.0, 0.0, 0.0), []) is None
    assert planner.choose_frontier(frontier_map(), None, []) is None
