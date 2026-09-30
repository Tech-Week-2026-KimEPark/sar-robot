import math

import numpy as np
import pytest
from planner_checks import random_map, reference_layers

from sar import config
from sar.grid_map import FREE, OCCUPIED, UNKNOWN, GridMap, simulate_scan


def room_walls(gm: GridMap, half: float = 1.0) -> np.ndarray:
    """(0, 0) 중심 한 변 2*half [m] 정사각형 방의 벽 bool 격자."""
    walls = np.zeros((gm.n, gm.n), bool)
    r0, c0 = gm.to_cell(-half, -half)
    r1, c1 = gm.to_cell(half, half)
    walls[r0, c0 : c1 + 1] = walls[r1, c0 : c1 + 1] = True
    walls[r0 : r1 + 1, c0] = walls[r0 : r1 + 1, c1] = True
    return walls


def test_cell_world_round_trip_and_axes():
    gm = GridMap(0.0, 0.0, size=4.0)
    assert gm.n == 80
    assert gm.to_cell(0.0, 0.0) == (40, 40)
    # row ↔ y, col ↔ x
    assert gm.to_cell(1.0, 0.0) == (40, 60)
    assert gm.to_cell(0.0, 1.0) == (60, 40)
    x, y = gm.to_world(*gm.to_cell(0.52, -0.73))
    assert abs(x - 0.52) <= gm.res / 2 and abs(y + 0.73) <= gm.res / 2


def test_default_map_is_centered_on_start():
    gm = GridMap()
    assert gm.n == 640
    assert gm.to_cell(config.START_X, config.START_Y) == (320, 320)


def test_room_scan_marks_walls_free_and_unknown():
    gm = GridMap(0.0, 0.0, size=6.0)
    scan = simulate_scan(room_walls(gm), gm.to_cell, (0.0, 0.0, 0.0))
    for _ in range(3):
        gm.update((0.0, 0.0, 0.0), scan)
    grid = gm.public()
    assert grid[gm.to_cell(0.0, 0.0)] == FREE
    assert grid[gm.to_cell(0.5, 0.5)] == FREE
    assert grid[gm.to_cell(1.0, 0.0)] == OCCUPIED
    assert grid[gm.to_cell(0.0, -1.0)] == OCCUPIED
    assert grid[gm.to_cell(2.0, 0.0)] == UNKNOWN


def test_lidar_index_direction():
    # 인덱스 90(왼쪽 = +y, theta 0)만 1 m에서 반사
    gm = GridMap(0.0, 0.0, size=6.0)
    ranges = [math.inf] * 360
    ranges[90] = 1.0
    gm.update((0.0, 0.0, 0.0), ranges)
    assert gm.logodds[gm.to_cell(0.0, 1.0)] > 0
    assert gm.logodds[gm.to_cell(0.0, 0.5)] < 0


def test_heading_rotates_scan():
    # theta = pi/2이면 정면(인덱스 180)은 +y 방향
    gm = GridMap(0.0, 0.0, size=6.0)
    ranges = [math.inf] * 360
    ranges[180] = 1.0
    gm.update((0.0, 0.0, math.pi / 2), ranges)
    assert gm.logodds[gm.to_cell(0.0, 1.0)] > 0


def test_no_return_clears_up_to_max_range_without_hits():
    gm = GridMap(0.0, 0.0, size=10.0)
    gm.update((0.0, 0.0, 0.0), [math.inf] * 360)
    assert not gm.occupied().any()
    assert gm.seen[gm.to_cell(config.LIDAR_MAX - 0.1, 0.0)]
    assert not gm.seen[gm.to_cell(config.LIDAR_MAX + 0.2, 0.0)]


def test_none_inputs_are_ignored():
    gm = GridMap(0.0, 0.0, size=4.0)
    gm.update(None, [1.0] * 360)
    gm.update((0.0, 0.0, 0.0), None)
    gm.update((0.0, 0.0, 0.0), [])
    assert not gm.seen.any()


def test_log_odds_clipped_and_moving_person_cleared():
    gm = GridMap(0.0, 0.0, size=6.0)
    person = [math.inf] * 360
    person[180] = 1.0
    for _ in range(20):
        gm.update((0.0, 0.0, 0.0), person)
    cell = gm.to_cell(1.0, 0.0)
    assert gm.logodds[cell] == pytest.approx(config.L_MAX)
    # 사람이 떠난 뒤 (L_MAX - OCC_THRESHOLD) / |L_FREE| = 8회 이내에 빈칸
    empty = [math.inf] * 360
    for _ in range(8):
        gm.update((0.0, 0.0, 0.0), empty)
    assert gm.public()[cell] == FREE


def test_layers_inflate_around_walls():
    gm = GridMap(0.0, 0.0, size=6.0)
    gm.seen[:] = True
    gm.logodds[room_walls(gm)] = config.L_MAX
    gm.invalidate()
    occ, blocked, soft, unknown = gm.layers()
    assert occ[gm.to_cell(1.0, 0.0)]
    assert blocked[gm.to_cell(1.0 - config.INFLATE + 0.03, 0.0)]
    assert not blocked[gm.to_cell(0.0, 0.0)]
    assert soft[gm.to_cell(1.0 - config.INFLATE - 0.05, 0.0)]
    assert not soft[gm.to_cell(0.0, 0.0)]
    assert not unknown.any()


def test_frontiers_groups_and_filters_small_clusters():
    gm = GridMap(0.0, 0.0, size=4.0)
    # 왼쪽 절반 확인 (빈칸): 경계 열 1개가 프론티어 묶음 1개
    gm.seen[:, : gm.n // 2] = True
    gm.logodds[:, : gm.n // 2] = config.L_MIN
    # 오른쪽 모르는 영역 안 3칸 빈칸 섬: MIN_FRONTIER_CELLS 미만이므로 제외
    gm.seen[10, 60:63] = True
    gm.invalidate()
    frontiers = gm.frontiers()
    assert len(frontiers) == 1
    size, cells = frontiers[0]
    assert size == gm.n
    assert all(col == gm.n // 2 - 1 for _, col in cells)


def test_frontiers_empty_when_fully_known():
    gm = GridMap(0.0, 0.0, size=2.0)
    gm.seen[:] = True
    assert gm.frontiers() == []
    assert GridMap(0.0, 0.0, size=2.0).frontiers() == []


def test_layers_match_independent_distance_calculation():
    # 팽창 영역이 모든 장애물 칸까지의 유클리드 거리를 직접 계산한 결과와 같음
    for seed in range(3):
        gm = random_map(seed, size=4.0, blocks=8)
        passable, _ = reference_layers(gm, allow_unknown=True)
        _, blocked, _, _ = gm.layers()
        assert np.array_equal(blocked, ~passable)


def test_plan_version_changes_only_when_classification_changes():
    gm = GridMap(0.0, 0.0, size=6.0)
    scan = simulate_scan(room_walls(gm), gm.to_cell, (0.0, 0.0, 0.0))
    gm.update((0.0, 0.0, 0.0), scan)
    assert gm.version == 1 and gm.plan_version == 1
    layers = gm.layers()
    frontiers = gm.frontiers()
    before = gm.logodds.copy()
    gm.update((0.0, 0.0, 0.0), scan)  # 같은 스캔: log-odds는 변하고 분류는 유지
    assert gm.version == 2 and gm.plan_version == 1
    assert not np.array_equal(before, gm.logodds)
    assert gm.layers() is layers  # 캐시 유지
    assert gm.frontiers() == frontiers
    # 새 장애물(1 m 앞 반사): 분류 변경으로 plan_version 증가와 캐시 무효화
    person = list(scan)
    person[180] = 0.5
    gm.update((0.0, 0.0, 0.0), person)
    gm.update((0.0, 0.0, 0.0), person)
    assert gm.plan_version > 1
    assert gm.layers() is not layers
    assert gm.layers()[0][gm.to_cell(0.5, 0.0)]


def test_new_cells_change_plan_version():
    gm = GridMap(0.0, 0.0, size=10.0)
    empty = [math.inf] * 360
    gm.update((0.0, 0.0, 0.0), empty)
    gm.update((0.0, 0.0, 0.0), empty)
    version = gm.plan_version
    gm.update((0.5, 0.0, 0.0), empty)  # 이동: 새로 확인한 칸 발생
    assert gm.plan_version == version + 1


def test_invalidate_after_direct_edit():
    gm = GridMap(0.0, 0.0, size=4.0)
    gm.seen[:] = True
    gm.invalidate()
    assert not gm.layers()[0].any()
    version = gm.plan_version
    gm.logodds[gm.to_cell(1.0, 0.0)] = config.L_MAX
    gm.invalidate()
    assert gm.plan_version == version + 1
    assert gm.layers()[0][gm.to_cell(1.0, 0.0)]


def test_update_ignores_non_finite_pose():
    gm = GridMap(0.0, 0.0, size=4.0)
    gm.update((math.nan, 0.0, 0.0), [1.0] * 360)
    gm.update((0.0, 0.0, math.inf), [1.0] * 360)
    assert not gm.seen.any() and gm.version == 0


def test_frontier_sizes_match_frontiers():
    gm = GridMap(0.0, 0.0, size=4.0)
    gm.seen[:, : gm.n // 2] = True
    gm.logodds[:, : gm.n // 2] = config.L_MIN
    gm.seen[10, 60:63] = True  # MIN_FRONTIER_CELLS 미만 묶음
    gm.invalidate()
    sizes = gm.frontier_sizes()
    assert sizes.shape == (gm.n, gm.n)
    for size, cells in gm.frontiers():
        assert all(sizes[cell] == size for cell in cells)
    assert sizes[10, 61] == 0
    assert np.count_nonzero(sizes) == sum(size for size, _ in gm.frontiers())
    assert not GridMap(0.0, 0.0, size=2.0).frontier_sizes().any()


def test_frontiers_returns_copy():
    gm = GridMap(0.0, 0.0, size=4.0)
    gm.seen[:, : gm.n // 2] = True
    gm.invalidate()
    first = gm.frontiers()
    first.clear()
    assert len(gm.frontiers()) == 1


def test_simulate_scan_distances():
    gm = GridMap(0.0, 0.0, size=6.0)
    scan = simulate_scan(room_walls(gm), gm.to_cell, (0.0, 0.0, 0.0))
    assert len(scan) == 360
    # 정면(180)·왼쪽(90)·뒤(0)·오른쪽(270) 벽은 1 m 근처
    for index in (0, 90, 180, 270):
        assert scan[index] == pytest.approx(1.0, abs=gm.res)
    open_scan = simulate_scan(np.zeros((gm.n, gm.n), bool), gm.to_cell, (0.0, 0.0, 0.0))
    assert all(math.isinf(d) for d in open_scan)
