"""계획 모듈 테스트용 독립 검사기와 합성 지도.

planner 내부 함수를 호출하지 않는다. 경로 안전성은 선분을 2 mm 간격으로 표본화해 확인하고,
최단 비용은 별도 다익스트라로 계산한다. 구현과 같은 오류를 공유하지 않기 위한 구성이다.
"""

import heapq
import math
from itertools import pairwise

import numpy as np

from sar import config
from sar.grid_map import GridMap

SAMPLE_STEP = 0.002  # m, 선분 표본 간격


def make_map(size: float, walls: np.ndarray | None = None, known: np.ndarray | None = None):
    """(0, 0) 중심 지도. walls는 장애물 bool 격자, known은 확인한 칸 bool 격자(기본: 전체)."""
    gm = GridMap(0.0, 0.0, size=size)
    gm.seen[:] = True if known is None else known
    gm.logodds[:] = config.L_MIN
    if walls is not None:
        gm.logodds[walls] = config.L_MAX
    gm.logodds[~gm.seen] = 0.0
    gm.invalidate() if hasattr(gm, "invalidate") else gm._cache.clear()
    return gm


def rect(gm: GridMap, x0: float, y0: float, x1: float, y1: float) -> tuple[slice, slice]:
    """월드 좌표 사각형 (양 끝 포함)의 격자 슬라이스."""
    ra, ca = gm.to_cell(min(x0, x1), min(y0, y1))
    rb, cb = gm.to_cell(max(x0, x1), max(y0, y1))
    return slice(ra, rb + 1), slice(ca, cb + 1)


def two_rooms(door: bool = True, size: float = 8.0) -> GridMap:
    """x -3~3 / y -2~2 외벽, x = 0 칸막이, 칸막이 가운데 0.8 m 문."""
    gm = GridMap(0.0, 0.0, size=size)
    wall = np.zeros((gm.n, gm.n), bool)
    wall[rect(gm, -3.0, -2.0, 3.0, -2.0)] = True
    wall[rect(gm, -3.0, 2.0, 3.0, 2.0)] = True
    wall[rect(gm, -3.0, -2.0, -3.0, 2.0)] = True
    wall[rect(gm, 3.0, -2.0, 3.0, 2.0)] = True
    wall[rect(gm, 0.0, -2.0, 0.0, 2.0)] = True
    if door:
        wall[rect(gm, 0.0, -0.4, 0.0, 0.4)] = False
    return make_map(size, wall)


def random_map(seed: int, size: float = 6.0, blocks: int = 14) -> GridMap:
    """외벽과 무작위 사각형 장애물. 시드가 같으면 같은 지도."""
    rng = np.random.default_rng(seed)
    gm = GridMap(0.0, 0.0, size=size)
    half = size / 2 - 0.5
    wall = np.zeros((gm.n, gm.n), bool)
    wall[rect(gm, -half, -half, half, -half)] = True
    wall[rect(gm, -half, half, half, half)] = True
    wall[rect(gm, -half, -half, -half, half)] = True
    wall[rect(gm, half, -half, half, half)] = True
    for _ in range(blocks):
        x, y = rng.uniform(-half + 0.3, half - 0.5, 2)
        w, h = rng.uniform(0.05, 0.5, 2)
        wall[rect(gm, x, y, x + w, y + h)] = True
    return make_map(size, wall)


def free_points(gm: GridMap, seed: int, count: int) -> list[tuple[float, float]]:
    """통과 가능 칸 안의 무작위 좌표 (칸 중심이 아닌 임의 위치)."""
    rng = np.random.default_rng(seed)
    _, blocked, _, unknown = gm.layers()
    rows, cols = np.nonzero(~blocked & ~unknown)
    picks = rng.integers(0, len(rows), count)
    points = []
    for i in picks:
        x, y = gm.to_world(int(rows[i]), int(cols[i]))
        dx, dy = rng.uniform(-0.49, 0.49, 2) * gm.res
        points.append((x + dx, y + dy))
    return points


def path_violations(
    gm: GridMap, path, allow_unknown: bool = False, escape_head: bool = False
) -> list[str]:
    """경로의 안전 규칙 위반 목록. 빈 목록이면 안전.

    - 모든 표본점의 칸은 팽창 영역(blocked) 밖. 모르는 칸은 allow_unknown일 때만 허용
    - 연속한 표본점이 대각 이웃 칸으로 넘어가면 양옆 직교 칸도 통과 가능해야 함 (모서리 통과 금지)
    - escape_head: 시작점이 팽창 영역 안이면 처음 통과 가능 칸에 도달할 때까지 blocked 허용.
      이 구간에서도 장애물 칸(occ)은 금지
    """
    occ, blocked, _, unknown = gm.layers()

    def ok(cell) -> bool:
        if not (0 <= cell[0] < gm.n and 0 <= cell[1] < gm.n):
            return False
        return not blocked[cell] and (allow_unknown or not unknown[cell])

    problems = []
    escaping = escape_head
    prev = None
    for (x0, y0), (x1, y1) in pairwise(path):
        n = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / SAMPLE_STEP)))
        for i in range(n + 1):
            t = i / n
            p = (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
            cell = gm.to_cell(*p)
            if escaping:
                if ok(cell):
                    escaping = False
                elif not (0 <= cell[0] < gm.n and 0 <= cell[1] < gm.n) or occ[cell]:
                    problems.append(f"탈출 구간이 장애물 칸 통과 {cell} at {p}")
                prev = cell
                continue
            if not ok(cell):
                problems.append(f"통과 불가 칸 {cell} at ({p[0]:.3f}, {p[1]:.3f})")
            elif prev is not None and prev[0] != cell[0] and prev[1] != cell[1]:
                for side in ((prev[0], cell[1]), (cell[0], prev[1])):
                    if not ok(side):
                        problems.append(f"모서리 통과 {prev}->{cell}, 통과 불가 옆 칸 {side}")
            prev = cell
    return problems


def reference_layers(gm: GridMap, allow_unknown: bool = False):
    """독립 계산한 (통과 가능 bool 격자, 칸 비용 배수 격자).

    장애물 거리는 모든 장애물 칸까지의 유클리드 거리를 직접 계산한다 (OpenCV 미사용).
    """
    occ = gm.logodds > config.OCC_THRESHOLD
    n = gm.n
    rr, cc = np.mgrid[0:n, 0:n]
    d2 = np.full((n, n), np.inf)
    for r, c in zip(*(a.tolist() for a in np.nonzero(occ)), strict=True):
        np.minimum(d2, (rr - r) ** 2 + (cc - c) ** 2, out=d2)
    clear = np.sqrt(d2) * gm.res
    unknown = ~gm.seen
    passable = clear >= config.INFLATE
    mult = 1.0 + config.WALL_COST * np.clip(
        (config.INFLATE + config.WALL_BAND - clear) / config.WALL_BAND, 0.0, 1.0
    )
    if allow_unknown:
        mult = np.where(unknown, mult * config.UNKNOWN_COST, mult)
    else:
        passable &= ~unknown
    return passable, mult


def reference_costs(gm: GridMap, start_cell, allow_unknown: bool = False) -> dict:
    """독립 다익스트라로 start_cell에서 모든 칸까지의 최단 비용 [칸 단위].

    비용 규칙 (planner 기능 문서): 8방향, 대각은 양옆 직교 칸이 통과 가능할 때만 허용,
    간선 비용 = 이동 거리 x 두 칸 비용 배수의 평균.
    """
    passable, mult = reference_layers(gm, allow_unknown)
    n = gm.n

    def free(r, c):
        return 0 <= r < n and 0 <= c < n and passable[r, c]

    best = {start_cell: 0.0} if free(*start_cell) else {}
    heap = [(0.0, start_cell)] if best else []
    while heap:
        d, (r, c) = heapq.heappop(heap)
        if d > best.get((r, c), math.inf):
            continue
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if (dr, dc) == (0, 0) or not free(r + dr, c + dc):
                    continue
                if dr and dc and not (free(r + dr, c) and free(r, c + dc)):
                    continue
                step = math.hypot(dr, dc) * (mult[r, c] + mult[r + dr, c + dc]) / 2
                if d + step < best.get((r + dr, c + dc), math.inf):
                    best[(r + dr, c + dc)] = d + step
                    heapq.heappush(heap, (d + step, (r + dr, c + dc)))
    return best


def reference_cost(gm: GridMap, start_cell, goal_cell, allow_unknown: bool = False) -> float:
    """독립 다익스트라의 start_cell → goal_cell 최단 비용 [칸 단위]. 도달 불가면 inf."""
    return reference_costs(gm, start_cell, allow_unknown).get(goal_cell, math.inf)


def cells_cost(gm: GridMap, path, allow_unknown: bool = False) -> float:
    """다듬지 않은 경로(PATH_SMOOTH = False)의 칸 단위 비용을 독립 계산.

    경로점이 속한 칸의 순서를 구하고 인접 칸 사이 간선 비용을 더한다.
    같은 칸 안의 이동(실제 시작·끝 좌표 ↔ 칸 중심)은 비용 0으로 본다.
    """
    _, mult = reference_layers(gm, allow_unknown)
    cells = [gm.to_cell(x, y) for x, y in path]
    total = 0.0
    for a, b in pairwise(cells):
        step = math.hypot(a[0] - b[0], a[1] - b[1])
        assert step < 1.5, f"이웃이 아닌 칸 이동 {a} -> {b}"
        total += step * (mult[a] + mult[b]) / 2
    return total


def path_length(path) -> float:
    return sum(math.dist(a, b) for a, b in pairwise(path))
