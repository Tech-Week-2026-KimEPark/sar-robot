"""격자 경로 계획: 목표 1개는 A*, 목표 여러 개는 다익스트라.

| 용도 | 방법 | 이유 |
|---|---|---|
| plan() 두 지점 경로 | A*, 8방향, 옥타일 휴리스틱 | 목표가 1개이면 확장 칸 수가 가장 적음 |
| choose_frontier() | 로봇 기준 다익스트라, 조기 종료 | 모든 프론티어 경로 거리를 한 번에 계산 |
| DistanceField | 기준점 다익스트라 1회 | 복귀 거리 즉시 조회, 복귀 경로, A* 휴리스틱 |

- 칸 비용 = 이동 거리 x 칸 비용 배수. 배수는 1 이상이므로 옥타일 거리는 과대평가하지 않음
- 벽 근처(팽창 영역 바깥 WALL_BAND 폭)는 벽에 가까울수록 배수가 1 + WALL_COST까지 증가
- 대각 이동은 양옆 직교 칸이 모두 통과 가능할 때만 허용 (모서리 통과 방지)
- 계산은 확인한 영역 + PLAN_MARGIN 범위만 수행. 테두리 1칸을 통과 불가로 채워 경계 검사 생략
"""

import heapq
import math
from collections.abc import Callable, Iterable, Sequence
from itertools import pairwise

import numpy as np

from sar import config
from sar.grid_map import GridMap

Point = tuple[float, float]
_SQRT2 = math.sqrt(2)


class _Window:
    """계획 영역의 통과 가능 여부와 칸 비용 배수. 1차원 칸 번호로 접근."""

    def __init__(
        self, grid: GridMap, must_include: Iterable[tuple[int, int]], allow_unknown: bool
    ) -> None:
        _, blocked, _, unknown = grid.layers()
        rows, cols = np.nonzero(grid.seen.any(axis=1))[0], np.nonzero(grid.seen.any(axis=0))[0]
        pts = list(must_include)
        rs = [r for r, _ in pts] + ([rows[0], rows[-1]] if len(rows) else [])
        cs = [c for _, c in pts] + ([cols[0], cols[-1]] if len(cols) else [])
        margin = int(math.ceil(config.PLAN_MARGIN / grid.res))
        self.r0 = max(0, min(rs) - margin)
        self.c0 = max(0, min(cs) - margin)
        r1 = min(grid.n - 1, max(rs) + margin)
        c1 = min(grid.n - 1, max(cs) + margin)
        win = (slice(self.r0, r1 + 1), slice(self.c0, c1 + 1))

        clear = grid.clearance()[win]
        cost = 1.0 + config.WALL_COST * np.clip(
            (config.INFLATE + config.WALL_BAND - clear) / config.WALL_BAND, 0.0, 1.0
        )
        unk = unknown[win]
        passable = ~blocked[win]
        if allow_unknown:
            cost = np.where(unk, cost * config.UNKNOWN_COST, cost)
        else:
            passable &= ~unk
        # 테두리 1칸 추가: 이웃 칸 번호가 항상 배열 안에 있음
        self.passable = np.pad(passable, 1, constant_values=False)
        self.cost_arr = np.pad(cost, 1, constant_values=np.inf)
        self.h, self.w = self.passable.shape
        self.res = grid.res
        self.grid = grid
        w = self.w
        # (칸 번호 차이, 이동 거리 [칸], 대각 이동 시 확인할 직교 칸 2개)
        self.moves = [
            (-w, 1.0, 0, 0),
            (w, 1.0, 0, 0),
            (-1, 1.0, 0, 0),
            (1, 1.0, 0, 0),
            (-w - 1, _SQRT2, -w, -1),
            (-w + 1, _SQRT2, -w, 1),
            (w - 1, _SQRT2, w, -1),
            (w + 1, _SQRT2, w, 1),
        ]

    def index(self, row: int, col: int) -> int | None:
        """지도 칸 (row, col)의 계획 영역 칸 번호. 영역 밖이면 None."""
        r, c = row - self.r0 + 1, col - self.c0 + 1
        if 1 <= r < self.h - 1 and 1 <= c < self.w - 1:
            return r * self.w + c
        return None

    def cell(self, idx: int) -> tuple[int, int]:
        """계획 영역 칸 번호의 지도 칸 (row, col)."""
        r, c = divmod(idx, self.w)
        return r - 1 + self.r0, c - 1 + self.c0

    def world(self, idx: int) -> Point:
        return self.grid.to_world(*self.cell(idx))

    def snap(self, idx: int | None) -> int | None:
        """칸이 통과 불가이면 SNAP_RADIUS 안에서 가장 가까운 통과 가능 칸. 없으면 None."""
        if idx is None:
            return None
        if self.passable.flat[idx]:
            return idx
        k = int(math.ceil(config.SNAP_RADIUS / self.res))
        r, c = divmod(idx, self.w)
        ra, rb = max(0, r - k), min(self.h, r + k + 1)
        ca, cb = max(0, c - k), min(self.w, c + k + 1)
        rr, cc = np.nonzero(self.passable[ra:rb, ca:cb])
        if len(rr) == 0:
            return None
        d2 = (rr + ra - r) ** 2 + (cc + ca - c) ** 2
        best = int(np.argmin(d2))
        if d2[best] > k * k:
            return None
        return int((rr[best] + ra) * self.w + cc[best] + ca)


def _search(
    win: _Window,
    source: int,
    goal: int | None = None,
    heuristic: Callable[[int], float] | None = None,
    on_pop: Callable[[int, float], bool] | None = None,
) -> tuple[list[float], list[int]]:
    """A* (heuristic 지정) 또는 다익스트라. 거리는 칸 단위 비용.

    goal 칸을 꺼내거나 on_pop(칸, 거리)가 True를 반환하면 종료.
    각 칸은 한 번만 확정되므로 반복 횟수는 계획 영역 칸 수 x 8 이하.
    """
    n = win.h * win.w
    passable = win.passable.ravel().tolist()
    cost = win.cost_arr.ravel().tolist()
    moves = win.moves
    dist = [math.inf] * n
    parent = [-1] * n
    closed = bytearray(n)
    dist[source] = 0.0
    heap = [(heuristic(source) if heuristic else 0.0, 0.0, source)]
    while heap:
        _, d, u = heapq.heappop(heap)
        if closed[u]:
            continue
        closed[u] = 1
        if u == goal or (on_pop is not None and on_pop(u, d)):
            break
        cu = cost[u]
        for off, step, a, b in moves:
            v = u + off
            if not passable[v] or closed[v]:
                continue
            if a and not (passable[u + a] and passable[u + b]):
                continue
            nd = d + step * (cu + cost[v]) * 0.5
            if nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(heap, (nd + heuristic(v) if heuristic else nd, nd, v))
    return dist, parent


def _octile(win: _Window, goal: int) -> Callable[[int], float]:
    gr, gc = divmod(goal, win.w)

    def h(idx: int) -> float:
        r, c = divmod(idx, win.w)
        dr, dc = abs(r - gr), abs(c - gc)
        return max(dr, dc) + (_SQRT2 - 1) * min(dr, dc)

    return h


def _unwind(parent: list[int], end: int) -> list[int]:
    """parent를 따라 end에서 시작 칸까지 역추적한 칸 번호 목록 (시작 → end 순서)."""
    path = [end]
    for _ in range(len(parent)):
        p = parent[path[-1]]
        if p < 0:
            break
        path.append(p)
    path.reverse()
    return path


def _line_ok(win: _Window, a: int, b: int, max_cost: float) -> bool:
    """칸 a·b 중심을 잇는 선분 위 모든 칸이 통과 가능하고 비용 배수가 max_cost 이하인지 여부."""
    ar, ac = divmod(a, win.w)
    br, bc = divmod(b, win.w)
    steps = 2 * max(abs(br - ar), abs(bc - ac))
    for k in range(1, steps):
        t = k / steps
        r = int(round(ar + (br - ar) * t))
        c = int(round(ac + (bc - ac) * t))
        if not win.passable[r, c] or win.cost_arr[r, c] > max_cost + 1e-9:
            return False
    return True


def _smooth(win: _Window, cells: list[int]) -> list[int]:
    """시야선 기반 경로 다듬기 (string pulling).

    원래 경로 구간의 최대 비용 배수를 넘는 칸을 지나지 않으므로 벽에 더 가까워지지 않음.
    """
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:  # i는 매 반복 1 이상 증가
        j = i + 1
        seg_max = max(win.cost_arr.flat[cells[i]], win.cost_arr.flat[cells[j]])
        while j + 1 < len(cells):
            cand_max = max(seg_max, win.cost_arr.flat[cells[j + 1]])
            if not _line_ok(win, cells[i], cells[j + 1], cand_max):
                break
            j += 1
            seg_max = cand_max
        out.append(cells[j])
        i = j
    return out


def _resample(points: Sequence[Point], step: float) -> list[Point]:
    """꺾은선을 step [m] 간격 점으로 변환. 시작점과 끝점 포함."""
    out = [points[0]]
    for (x0, y0), (x1, y1) in pairwise(points):
        seg = math.hypot(x1 - x0, y1 - y0)
        k = max(1, int(math.ceil(seg / step)))
        out.extend((x0 + (x1 - x0) * i / k, y0 + (y1 - y0) * i / k) for i in range(1, k + 1))
    return out


def _to_path(win: _Window, cells: list[int], head: Point | None, tail: Point | None) -> list:
    """칸 경로를 월드 좌표 경로로 변환. head·tail은 칸 중심 대신 사용할 실제 시작·끝 좌표."""
    if config.PATH_SMOOTH:
        cells = _smooth(win, cells)
    points = [win.world(i) for i in cells]
    if head is not None:
        points[0] = head
    if tail is not None:
        if len(points) > 1:
            points[-1] = tail
        else:
            points.append(tail)
    return _resample(points, config.PATH_STEP)


class DistanceField:
    """기준점까지의 경로 거리 지도. 기준점에서 다익스트라 1회 실행.

    - distance(xy): 복귀 거리 [m] 즉시 조회 (RETURN_RESERVE 판단)
    - path(xy): xy에서 기준점까지 최적 경로. 추가 탐색 없이 역추적만 수행
    - plan(..., field=...)의 A* 휴리스틱. 같은 지도에서 과대평가하지 않음
    모르는 칸은 UNKNOWN_COST 배수로 통과를 허용.
    """

    def __init__(self, grid: GridMap, origin_xy: Point) -> None:
        self.grid = grid
        self.origin = origin_xy
        cell = grid.to_cell(*origin_xy)
        self.win = _Window(grid, [cell], allow_unknown=True)
        src = self.win.snap(self.win.index(*cell))
        if src is None:
            self._dist, self._parent = [], []
            return
        self._dist, self._parent = _search(self.win, src)

    def _lookup(self, row: int, col: int) -> float:
        """지도 칸의 거리 [칸 단위 비용]. 계산 범위 밖이거나 도달 불가면 inf."""
        idx = self.win.index(row, col)
        if idx is None or not self._dist:
            return math.inf
        return self._dist[idx]

    def distance(self, xy: Point) -> float | None:
        """xy에서 기준점까지 경로 비용 [m]. 도달 불가면 None."""
        idx = self.win.snap(self.win.index(*self.grid.to_cell(*xy)))
        if idx is None or not self._dist or math.isinf(self._dist[idx]):
            return None
        return self._dist[idx] * self.grid.res

    def path(self, xy: Point) -> list[Point] | None:
        """xy에서 기준점까지 경로 [(x, y), ...]. 도달 불가면 None."""
        idx = self.win.snap(self.win.index(*self.grid.to_cell(*xy)))
        if idx is None or not self._dist or math.isinf(self._dist[idx]):
            return None
        cells = _unwind(self._parent, idx)
        cells.reverse()  # 역추적 결과는 기준점 → xy 순서
        return _to_path(self.win, cells, head=xy, tail=self.origin)


def plan(
    grid: GridMap | None,
    start_xy: Point | None,
    goal_xy: Point | None,
    allow_unknown: bool = False,
    field: DistanceField | None = None,
) -> list[Point] | None:
    """start_xy에서 goal_xy까지 A* 경로 [(x, y), ...]. 경로가 없으면 None.

    - 시작·목표가 통과 불가 칸이면 SNAP_RADIUS 안의 가장 가까운 통과 가능 칸을 경유
    - 목표를 대체 칸으로 옮긴 경우 경로 끝점은 대체 칸 중심
    - field가 goal_xy 기준 DistanceField이면 휴리스틱으로 사용해 확장 칸 수를 줄임
    """
    if grid is None or start_xy is None or goal_xy is None:
        return None
    s_cell, g_cell = grid.to_cell(*start_xy), grid.to_cell(*goal_xy)
    if not (grid.inside(*s_cell) and grid.inside(*g_cell)):
        return None
    win = _Window(grid, [s_cell, g_cell], allow_unknown)
    g_raw = win.index(*g_cell)
    s, g = win.snap(win.index(*s_cell)), win.snap(g_raw)
    if s is None or g is None:
        return None
    heuristic = _octile(win, g)
    if field is not None and field.grid is grid:
        octile = heuristic

        def heuristic(idx: int) -> float:
            return max(octile(idx), field._lookup(*win.cell(idx)))

    dist, parent = _search(win, s, goal=g, heuristic=heuristic)
    if math.isinf(dist[g]):
        return None
    return _to_path(win, _unwind(parent, g), head=start_xy, tail=goal_xy if g == g_raw else None)


def choose_frontier(
    grid: GridMap | None,
    pose: Sequence[float] | None,
    blacklist: Iterable[Point] = (),
) -> Point | None:
    """점수 (프론티어 묶음 칸 수 / 경로 거리)가 가장 큰 프론티어 칸의 (x, y). 없으면 None.

    로봇에서 다익스트라를 실행해 각 묶음의 가장 가까운 도달 가능 칸을 찾음. 칸은 거리
    오름차순으로 확정되므로 남은 칸의 점수 상한은 (최대 묶음 크기 / 현재 거리)이며, 현재
    최고 점수가 상한 이상이면 탐색을 종료함. 블랙리스트 좌표 BLACKLIST_RADIUS 이내 칸은 제외.
    """
    if grid is None or pose is None:
        return None
    clusters = grid.frontiers()
    if not clusters:
        return None
    r_cell = grid.to_cell(pose[0], pose[1])
    if not grid.inside(*r_cell):
        return None
    win = _Window(grid, [r_cell], allow_unknown=False)
    source = win.snap(win.index(*r_cell))
    if source is None:
        return None

    size_at = np.zeros(win.h * win.w, np.int64)
    for size, cells in clusters:
        for row, col in cells:
            idx = win.index(row, col)
            if idx is not None:
                size_at[idx] = size
    size_grid = size_at.reshape(win.h, win.w)
    ys, xs = np.mgrid[0 : win.h, 0 : win.w]
    for bx, by in blacklist:
        br, bc = grid.to_cell(bx, by)
        near = (ys - (br - win.r0 + 1)) ** 2 + (xs - (bc - win.c0 + 1)) ** 2
        size_grid[near * grid.res**2 <= config.BLACKLIST_RADIUS**2] = 0
    if not size_at.any():
        return None
    max_size = int(size_at.max())
    sizes = size_at.tolist()
    best = [0.0, -1]  # [점수, 칸 번호]
    min_cells = config.FRONTIER_MIN_DIST / grid.res

    def on_pop(idx: int, d: float) -> bool:
        if sizes[idx] and d >= min_cells:
            score = sizes[idx] / (d * grid.res)
            if score > best[0]:
                best[0], best[1] = score, idx
        d_m = max(d, min_cells) * grid.res
        return best[1] >= 0 and best[0] >= max_size / d_m

    _search(win, source, on_pop=on_pop)
    return win.world(best[1]) if best[1] >= 0 else None


if __name__ == "__main__":
    # Webots 없이 실행: controllers/sar_main에서 python -m sar.planner
    # 벽으로 나뉜 방 2개, 벽 한가운데 문. 전체를 확인한 지도에서 경로와 복귀 거리 확인
    gm = GridMap(0.0, 0.0, size=8.0)
    gm.seen[:] = True
    gm.logodds[:] = config.L_MIN
    r0, c0 = gm.to_cell(-3.0, -2.0)
    r1, c1 = gm.to_cell(3.0, 2.0)
    wall = np.zeros_like(gm.seen)
    wall[r0, c0 : c1 + 1] = wall[r1, c0 : c1 + 1] = True
    wall[r0 : r1 + 1, c0] = wall[r0 : r1 + 1, c1] = True
    mid_c = gm.to_cell(0.0, 0.0)[1]
    door_a, door_b = gm.to_cell(0.0, -0.4)[0], gm.to_cell(0.0, 0.4)[0]
    wall[r0 : r1 + 1, mid_c] = True
    wall[door_a : door_b + 1, mid_c] = False
    gm.logodds[wall] = config.L_MAX
    gm._cache.clear()

    path = plan(gm, (-2.0, 1.5), (2.0, 1.5))
    assert path is not None and path[0] == (-2.0, 1.5) and path[-1] == (2.0, 1.5)
    assert any(abs(x) < 0.1 and abs(y) < 0.4 for x, y in path), "문을 지나야 함"
    field = DistanceField(gm, (2.0, 1.5))
    d = field.distance((-2.0, 1.5))
    assert d is not None and d > 4.0
    back = field.path((-2.0, 1.5))
    assert back is not None and back[-1] == (2.0, 1.5)
    print(f"planner self-test ok, path {len(path)} points, return distance {d:.2f} m")
