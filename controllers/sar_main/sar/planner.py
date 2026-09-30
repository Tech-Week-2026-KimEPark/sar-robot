"""격자 경로 계획: 목표 1개는 A*, 목표 여러 개는 다익스트라.

| 용도 | 방법 | 이유 |
|---|---|---|
| plan() 두 지점 경로 | A*, 8방향, 옥타일 휴리스틱 | 목표가 1개이면 확장 칸 수가 가장 적음 |
| choose_frontier_path() | 로봇 기준 다익스트라, 조기 종료 | 프론티어 선택과 경로를 한 번에 계산 |
| DistanceField | 기준점 다익스트라 1회 | 복귀 비용 조회, 복귀 경로, A* 휴리스틱 |

비용
- 칸 비용 = 이동 거리 x 칸 비용 배수. 배수는 1 이상이므로 옥타일 거리는 과대평가하지 않음
- 벽 근처(팽창 영역 바깥 WALL_BAND 폭)는 벽에 가까울수록 배수가 1 + WALL_COST까지 증가

최종 경로의 안전 규칙 (탐색, 경로 다듬기, 시작·끝 좌표 교체에 같은 규칙 적용)
- 경로 선분이 지나가거나 닿는 모든 칸은 통과 가능해야 함. 통과 가능 = 팽창 영역 밖이고
  allow_unknown이 거짓이면 확인한 칸
- 선분이 격자 모서리를 지나면 모서리에 닿는 칸 4개를 모두 검사 (A*의 대각 이동 규칙과 같음)
- 시작·목표가 통과 불가 칸이면 SNAP_RADIUS 안의 통과 가능 칸으로 대체. 실제 좌표와 대체 칸을
  잇는 선분이 장애물 칸을 지나면 그 칸은 선택하지 않음 (벽 반대편 칸 제외)
- 시작점이 팽창 영역 안이면 대체 칸까지의 탈출 구간만 팽창 영역 통과를 허용. 탈출 구간은 장애물
  칸을 지나지 않고, 시작 위치보다 장애물에 더 가까워지지 않음

계산은 확인한 영역 + PLAN_MARGIN 범위만 수행. 테두리 1칸을 통과 불가로 채워 경계 검사 생략.
계획 영역은 GridMap.plan_version이 같은 동안 재사용.
"""

import heapq
import math
from collections.abc import Callable, Iterable, Sequence
from itertools import pairwise

import cv2
import numpy as np

from sar import config
from sar.grid_map import GridMap

Point = tuple[float, float]
_SQRT2 = math.sqrt(2)
_EPS = 1e-6  # 칸 단위. 선분이 격자선·모서리에 닿는지 판정하는 여유


def _cost_signature() -> tuple[float, float, float, float]:
    """칸 비용과 통과 가능 여부를 정하는 설정값. 값이 바뀌면 이전 계산 결과는 재사용 불가."""
    return (config.INFLATE, config.WALL_BAND, config.WALL_COST, config.UNKNOWN_COST)


def _finite(xy: Sequence[float] | None) -> bool:
    return xy is not None and len(xy) >= 2 and math.isfinite(xy[0]) and math.isfinite(xy[1])


class _Window:
    """계획 영역의 통과 가능 여부와 칸 비용 배수. 1차원 칸 번호 또는 (행, 열) 실수 좌표로 접근."""

    def __init__(self, grid: GridMap, bounds: tuple[int, int, int, int], allow_unknown: bool):
        self.r0, self.c0, self.r1, self.c1 = bounds
        self.grid = grid
        self.res = grid.res
        self.allow_unknown = allow_unknown
        self.plan_version = grid.plan_version
        self.signature = _cost_signature()
        occ, blocked, _, unknown = grid.layers()
        win = (slice(self.r0, self.r1 + 1), slice(self.c0, self.c1 + 1))

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
        # 테두리 1칸 추가: 이웃 칸 번호가 항상 배열 안에 있음. 테두리는 통과 불가, 장애물 취급
        self.passable = np.pad(passable, 1, constant_values=False)
        self.cost_arr = np.pad(cost, 1, constant_values=np.inf)
        self.occ = np.pad(occ[win], 1, constant_values=True)
        self.known = np.pad(~unk, 1, constant_values=False)
        self.clear = np.pad(clear, 1, constant_values=0.0)
        self.h, self.w = self.passable.shape
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
        self._lists: tuple[list, list] | None = None
        self._labels: np.ndarray | None = None

    def lists(self) -> tuple[list, list]:
        """탐색 반복문용 (통과 가능 여부, 비용 배수 / 2) 목록."""
        if self._lists is None:
            self._lists = (self.passable.ravel().tolist(), (self.cost_arr * 0.5).ravel().tolist())
        return self._lists

    def labels(self) -> np.ndarray:
        """통과 가능 칸의 연결 영역 번호 (0은 통과 불가).

        대각 이동은 양옆 직교 칸이 통과 가능할 때만 허용하므로 도달 가능 여부는 4방향 연결과 같음.
        """
        if self._labels is None:
            _, self._labels = cv2.connectedComponents(
                self.passable.astype(np.uint8), connectivity=4
            )
        return self._labels

    def contains(self, other: "_Window") -> bool:
        return (
            self.r0 <= other.r0
            and self.c0 <= other.c0
            and self.r1 >= other.r1
            and self.c1 >= other.c1
        )

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

    def local(self, xy: Sequence[float]) -> tuple[float, float]:
        """월드 좌표의 계획 영역 (행, 열) 실수 좌표. 칸 (r, c)는 [r, r+1) x [c, c+1)."""
        return (
            (xy[1] - self.grid.y0) / self.res - self.r0 + 1,
            (xy[0] - self.grid.x0) / self.res - self.c0 + 1,
        )

    def center(self, idx: int) -> tuple[float, float]:
        r, c = divmod(idx, self.w)
        return r + 0.5, c + 0.5


def _window(grid: GridMap, must_include: Iterable[tuple[int, int]], allow_unknown: bool) -> _Window:
    """확인한 영역과 must_include 칸을 포함하는 계획 영역. 같은 지도 상태에서는 재사용."""
    cache = grid._cache
    if "seen_box" not in cache:
        rows = np.flatnonzero(grid.seen.any(axis=1))
        cols = np.flatnonzero(grid.seen.any(axis=0))
        cache["seen_box"] = (
            (int(rows[0]), int(rows[-1]), int(cols[0]), int(cols[-1])) if len(rows) else None
        )
    box = cache["seen_box"]
    pts = list(must_include)
    rs = [r for r, _ in pts] + (list(box[:2]) if box else [])
    cs = [c for _, c in pts] + (list(box[2:]) if box else [])
    margin = int(math.ceil(config.PLAN_MARGIN / grid.res))
    bounds = (
        max(0, min(rs) - margin),
        max(0, min(cs) - margin),
        min(grid.n - 1, max(rs) + margin),
        min(grid.n - 1, max(cs) + margin),
    )
    key = ("window", bounds, allow_unknown, _cost_signature())
    if key not in cache:
        cache[key] = _Window(grid, bounds, allow_unknown)
    return cache[key]


def _trace(p0: tuple[float, float], p1: tuple[float, float]) -> tuple[np.ndarray, ...]:
    """선분 p0 → p1 (행, 열 실수 좌표)이 지나는 칸과 닿는 칸.

    반환: (지나는 칸 행, 열, 닿는 칸 행, 열). 지나는 칸은 p0에서 p1 순서.
    닿는 칸은 선분이 격자선을 넘는 지점 주변 칸. 모서리를 지나면 모서리의 칸 4개를 모두 포함.
    """
    (r0, c0), (r1, c1) = p0, p1
    crossings = []
    for a0, a1 in ((r0, r1), (c0, c1)):
        lo, hi = (a0, a1) if a0 <= a1 else (a1, a0)
        lines = np.arange(math.floor(lo) + 1, math.ceil(hi))
        if len(lines):
            crossings.append((lines - a0) / (a1 - a0))
    if crossings:
        t = np.sort(np.concatenate(crossings))
        edges = np.concatenate(([0.0], t, [1.0]))
        keep = np.diff(edges) > 1e-9  # 모서리에서 겹친 교차점 사이의 길이 0 구간 제외
        mid = ((edges[:-1] + edges[1:]) * 0.5)[keep]
    else:
        t = np.empty(0)
        mid = np.array([0.5])
    inner_r = np.floor(r0 + (r1 - r0) * mid).astype(np.intp)
    inner_c = np.floor(c0 + (c1 - c0) * mid).astype(np.intp)
    pr, pc = r0 + (r1 - r0) * t, c0 + (c1 - c0) * t
    lo_r, hi_r = np.floor(pr - _EPS), np.floor(pr + _EPS)
    lo_c, hi_c = np.floor(pc - _EPS), np.floor(pc + _EPS)
    touch_r = np.concatenate([lo_r, lo_r, hi_r, hi_r]).astype(np.intp)
    touch_c = np.concatenate([lo_c, hi_c, lo_c, hi_c]).astype(np.intp)
    return inner_r, inner_c, touch_r, touch_c


def _segment_ok(win: _Window, p0, p1, max_cost: float = math.inf) -> bool:
    """선분이 지나가거나 닿는 칸이 모두 통과 가능하고, 지나는 칸의 비용 배수가 max_cost 이하인지."""
    inner_r, inner_c, touch_r, touch_c = _trace(p0, p1)
    if not (win.passable[inner_r, inner_c].all() and win.passable[touch_r, touch_c].all()):
        return False
    return math.isinf(max_cost) or bool((win.cost_arr[inner_r, inner_c] <= max_cost + 1e-9).all())


def _escape_ok(win: _Window, p_from, p_to, mover: bool) -> bool:
    """통과 불가 칸 안의 실제 좌표 p_from과 대체 칸 중심 p_to를 잇는 선분의 유효성.

    - 장애물 칸은 p_from이 장애물 칸 안일 때 그 연속 구간만 허용. 구간을 벗어난 뒤 다시 장애물
      칸을 지나거나 닿으면 무효 (벽 반대편)
    - mover(로봇이 실제로 이동하는 시작 쪽): allow_unknown이 거짓이면 첫 칸 이후 모르는 칸 금지.
      시작 위치의 장애물 거리(로봇 반지름 한도)보다 1칸 넘게 가까워지는 칸 금지
    """
    inner_r, inner_c, touch_r, touch_c = _trace(p_from, p_to)
    occ_in = win.occ[inner_r, inner_c]
    lead = len(occ_in) if occ_in.all() else int(np.argmin(occ_in))
    if occ_in[lead:].any():
        return False
    touch_occ = win.occ[touch_r, touch_c]
    if touch_occ.any():
        lead_cells = set(zip(inner_r[:lead].tolist(), inner_c[:lead].tolist(), strict=True))
        touched = zip(touch_r[touch_occ].tolist(), touch_c[touch_occ].tolist(), strict=True)
        if any(cell not in lead_cells for cell in touched):
            return False
    if mover:
        if not win.allow_unknown and not win.known[inner_r[1:], inner_c[1:]].all():
            return False
        floor = min(float(win.clear[inner_r[0], inner_c[0]]), config.ROBOT_RADIUS) - win.res
        if (win.clear[inner_r[lead:], inner_c[lead:]] < floor).any():
            return False
    return True


def _snap(win: _Window, xy: Sequence[float], mover: bool) -> tuple[int, bool] | None:
    """좌표의 계획 칸 번호와 대체 여부. 통과 불가 칸이면 가장 가까운 유효한 통과 가능 칸.

    후보는 실제 좌표에서 SNAP_RADIUS 이내 칸을 가까운 순서로 확인. 유효한 칸이 없으면 None.
    """
    idx = win.index(*win.grid.to_cell(xy[0], xy[1]))
    if idx is None:
        return None
    if win.passable.flat[idx]:
        return idx, False
    p = win.local(xy)
    k = int(math.ceil(config.SNAP_RADIUS / win.res))
    r, c = divmod(idx, win.w)
    ra, rb = max(0, r - k), min(win.h, r + k + 1)
    ca, cb = max(0, c - k), min(win.w, c + k + 1)
    rr, cc = np.nonzero(win.passable[ra:rb, ca:cb])
    if len(rr) == 0:
        return None
    rr, cc = rr + ra, cc + ca
    d2 = (rr + 0.5 - p[0]) ** 2 + (cc + 0.5 - p[1]) ** 2
    order = np.argsort(d2, kind="stable")
    limit = (config.SNAP_RADIUS / win.res) ** 2
    for i in order.tolist():  # 후보 수는 (2k + 1)^2 이하
        if d2[i] > limit:
            break
        cand = (int(rr[i]), int(cc[i]))
        if _escape_ok(win, p, (cand[0] + 0.5, cand[1] + 0.5), mover):
            return cand[0] * win.w + cand[1], True
    return None


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
    휴리스틱은 단조(consistent)여야 함. 옥타일 거리와 DistanceField 값은 단조.
    """
    n = win.h * win.w
    passable, half = win.lists()
    moves = win.moves
    dist = [math.inf] * n
    parent = [-1] * n
    closed = bytearray(n)
    dist[source] = 0.0
    push, pop = heapq.heappush, heapq.heappop
    heap = [(heuristic(source) if heuristic else 0.0, 0.0, source)]
    while heap:
        _, d, u = pop(heap)
        if closed[u]:
            continue
        closed[u] = 1
        if u == goal or (on_pop is not None and on_pop(u, d)):
            break
        hu = half[u]
        for off, step, a, b in moves:
            v = u + off
            if not passable[v] or closed[v]:
                continue
            if a and not (passable[u + a] and passable[u + b]):
                continue
            nd = d + step * (hu + half[v])
            if nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                push(heap, (nd + heuristic(v) if heuristic else nd, nd, v))
    return dist, parent


def _octile(win: _Window, goal: int) -> Callable[[int], float]:
    gr, gc = divmod(goal, win.w)
    w = win.w

    def h(idx: int) -> float:
        r, c = divmod(idx, w)
        dr, dc = abs(r - gr), abs(c - gc)
        return (dr + 0.41421356237309515 * dc) if dr > dc else (dc + 0.41421356237309515 * dr)

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


def _resample(points: Sequence[Point], step: float) -> list[Point]:
    """꺾은선을 step [m] 간격 점으로 변환. 시작점과 끝점 포함."""
    out = [points[0]]
    for (x0, y0), (x1, y1) in pairwise(points):
        seg = math.hypot(x1 - x0, y1 - y0)
        k = max(1, int(math.ceil(seg / step)))
        out.extend((x0 + (x1 - x0) * i / k, y0 + (y1 - y0) * i / k) for i in range(1, k))
        out.append((x1, y1))  # 구간 끝점은 계산 오차 없이 원래 좌표 사용
    return out


def _to_path(
    win: _Window, cells: list[int], head: Point, head_snapped: bool, tail: Point | None
) -> list[Point]:
    """칸 경로를 월드 좌표 경로로 변환.

    head: 실제 시작 좌표. head_snapped이면 head → cells[0] 중심은 탈출 구간으로 고정
    tail: 실제 끝 좌표. 목표를 대체 칸으로 옮겼으면 None (끝점은 cells[-1] 중심)

    다듬기 전 꺾은선은 [head, 칸 중심들, tail]이며 모든 선분이 안전 규칙을 만족함.
    다듬기는 _segment_ok를 통과한 선분으로만 중간 점을 생략하므로 결과도 같은 규칙을 만족함.
    생략한 구간의 최대 비용 배수를 넘는 칸을 지나지 않아 벽에 더 가까워지지 않음.
    """
    local = [win.local(head)] + [win.center(i) for i in cells]
    world = [head] + [win.world(i) for i in cells]
    cost = win.cost_arr.ravel()
    costs = [float(cost[cells[0]])] + [float(cost[i]) for i in cells]
    if tail is not None:
        local.append(win.local(tail))
        world.append(tail)
        costs.append(costs[-1])
    keep = list(range(len(local)))
    if config.PATH_SMOOTH and len(local) > 2:
        first = 1 if head_snapped else 0
        keep = list(range(first + 1))
        i = first
        while i < len(local) - 1:  # i는 매 반복 1 이상 증가
            j = i + 1
            seg_max = max(costs[i], costs[j])
            while j + 1 < len(local):
                cand = max(seg_max, costs[j + 1])
                if not _segment_ok(win, local[i], local[j + 1], cand):
                    break
                j += 1
                seg_max = cand
            keep.append(j)
            i = j
    points = [world[keep[0]]]
    for k in keep[1:]:
        if world[k] != points[-1]:
            points.append(world[k])
    return _resample(points, config.PATH_STEP)


def path_length(path: Sequence[Point] | None) -> float | None:
    """경로의 실제 길이 [m]. 경로가 None이면 None. 복귀 시간 추정은 이 값을 속도로 나눠 계산."""
    if path is None:
        return None
    return sum(math.dist(a, b) for a, b in pairwise(path))


class DistanceField:
    """기준점까지의 가중 경로 비용 지도. 기준점에서 다익스트라 1회 실행.

    - distance(xy): 기준점까지 가중 비용 [m]. 벽 근처·모르는 칸은 배수가 붙으므로 실제 길이 이상.
      지도가 바뀐 뒤에도 생성 시점의 값을 반환 (근삿값). 최신 여부는 is_current()로 확인
    - path(xy): xy에서 기준점까지 경로. 추가 탐색 없이 역추적. 지도가 바뀌었으면 None
    - plan(..., field=...): 목표·지도 상태·영역이 맞을 때만 A* 휴리스틱으로 사용
    모르는 칸은 UNKNOWN_COST 배수로 통과를 허용. 기준점이 통과 불가 칸이면 대체 칸 기준.
    """

    def __init__(self, grid: GridMap, origin_xy: Point) -> None:
        self.grid = grid
        self.origin = origin_xy
        self.plan_version = grid.plan_version
        self.win: _Window | None = None
        self._dist: list[float] = []
        self._parent: list[int] = []
        self._src: int | None = None
        self._src_snapped = False
        if not _finite(origin_xy) or not grid.inside(*grid.to_cell(*origin_xy)):
            return
        self.win = _window(grid, [grid.to_cell(*origin_xy)], allow_unknown=True)
        snapped = _snap(self.win, origin_xy, mover=False)
        if snapped is None:
            return
        self._src, self._src_snapped = snapped
        self._dist, self._parent = _search(self.win, self._src)

    def is_current(self) -> bool:
        """생성 이후 계획용 지도 상태와 비용 설정이 바뀌지 않았는지 여부."""
        return (
            self.win is not None
            and self.plan_version == self.grid.plan_version
            and self.win.signature == _cost_signature()
        )

    def _locate(self, xy: Sequence[float] | None) -> tuple[int, bool] | None:
        if self.win is None or self._src is None or not _finite(xy):
            return None
        snapped = _snap(self.win, xy, mover=True)
        if snapped is None or math.isinf(self._dist[snapped[0]]):
            return None
        return snapped

    def distance(self, xy: Point) -> float | None:
        """xy에서 기준점까지 가중 경로 비용 [m]. 도달 불가면 None. 길이는 path_length() 사용."""
        found = self._locate(xy)
        return None if found is None else self._dist[found[0]] * self.grid.res

    def path(self, xy: Point) -> list[Point] | None:
        """xy에서 기준점까지 경로 [(x, y), ...]. 도달 불가이거나 지도가 바뀌었으면 None."""
        if not self.is_current():
            return None
        found = self._locate(xy)
        if found is None:
            return None
        cells = _unwind(self._parent, found[0])
        cells.reverse()  # 역추적 결과는 기준점 → xy 순서
        tail = None if self._src_snapped else self.origin
        return _to_path(self.win, cells, head=(xy[0], xy[1]), head_snapped=found[1], tail=tail)


def _field_heuristic(
    field: DistanceField | None, win: _Window, goal: int
) -> Callable[[int], float] | None:
    """field를 A* 휴리스틱으로 쓸 수 있으면 조회 함수, 아니면 None.

    조건: 같은 지도 객체, 같은 계획용 지도 상태와 비용 설정, 기준 칸 = 계획 목표 칸,
    거리 지도 영역이 계획 영역을 포함. 거리 지도는 모르는 칸 통과를 허용한 그래프의 최단
    비용이고 계획 그래프는 그 부분 그래프이므로 과대평가하지 않으며 단조성을 유지함.
    """
    if field is None or field.grid is not win.grid or not field.is_current():
        return None
    fwin = field.win
    if field._src is None or fwin.cell(field._src) != win.cell(goal) or not fwin.contains(win):
        return None
    fdist, fw, w = field._dist, fwin.w, win.w
    dr, dc = win.r0 - fwin.r0, win.c0 - fwin.c0

    def lookup(idx: int) -> float:
        r, c = divmod(idx, w)
        return fdist[(r + dr) * fw + c + dc]

    return lookup


def plan(
    grid: GridMap | None,
    start_xy: Point | None,
    goal_xy: Point | None,
    allow_unknown: bool = False,
    field: DistanceField | None = None,
) -> list[Point] | None:
    """start_xy에서 goal_xy까지 A* 경로 [(x, y), ...]. 경로가 없으면 None.

    - 첫 점은 start_xy. 끝점은 goal_xy, 목표를 대체 칸으로 옮긴 경우 대체 칸 중심
    - 좌표가 유한하지 않거나 지도 밖이면 None
    - field: goal_xy 기준 DistanceField. 사용 조건이 맞지 않으면 무시하고 일반 A*로 계산
    """
    if grid is None or not _finite(start_xy) or not _finite(goal_xy):
        return None
    s_cell, g_cell = grid.to_cell(start_xy[0], start_xy[1]), grid.to_cell(goal_xy[0], goal_xy[1])
    if not (grid.inside(*s_cell) and grid.inside(*g_cell)):
        return None
    win = _window(grid, [s_cell, g_cell], allow_unknown)
    start, goal = _snap(win, start_xy, mover=True), _snap(win, goal_xy, mover=False)
    if start is None or goal is None:
        return None
    (s, s_snapped), (g, g_snapped) = start, goal
    labels = win.labels().ravel()
    if labels[s] != labels[g]:
        return None
    heuristic = _octile(win, g)
    lookup = _field_heuristic(field, win, g)
    if lookup is not None:
        octile = heuristic

        def heuristic(idx: int) -> float:
            return max(octile(idx), lookup(idx))

    dist, parent = _search(win, s, goal=g, heuristic=heuristic)
    if math.isinf(dist[g]):
        return None
    tail = None if g_snapped else (goal_xy[0], goal_xy[1])
    return _to_path(win, _unwind(parent, g), (start_xy[0], start_xy[1]), s_snapped, tail)


def choose_frontier_path(
    grid: GridMap | None,
    pose: Sequence[float] | None,
    blacklist: Iterable[Point] = (),
) -> tuple[Point, list[Point]] | None:
    """점수가 가장 큰 프론티어 칸의 (x, y)와 그 칸까지의 경로. 없으면 None.

    점수 = 프론티어 묶음 칸 수 / 경로 비용 [m]. 로봇에서 다익스트라를 실행하며 칸은 비용
    오름차순으로 확정되므로 남은 칸의 점수 상한은 (도달 가능한 최대 묶음 크기 / 현재 비용)임.
    현재 최고 점수가 상한 이상이면 종료. 경로는 같은 다익스트라의 역추적 결과이므로
    plan(grid, pose, target)과 비용이 같고 추가 탐색이 없음.
    블랙리스트 좌표 BLACKLIST_RADIUS 이내 칸과 FRONTIER_MIN_DIST보다 가까운 칸은 제외.
    """
    if grid is None or not _finite(pose):
        return None
    r_cell = grid.to_cell(pose[0], pose[1])
    if not grid.inside(*r_cell):
        return None
    sizes_full = grid.frontier_sizes()
    if not sizes_full.any():
        return None
    win = _window(grid, [r_cell], allow_unknown=False)
    start = _snap(win, pose, mover=True)
    if start is None:
        return None
    source, snapped = start
    labels = win.labels()
    size_grid = np.zeros((win.h, win.w), np.int32)
    size_grid[1:-1, 1:-1] = sizes_full[win.r0 : win.r1 + 1, win.c0 : win.c1 + 1]
    size_grid[labels != labels.flat[source]] = 0  # 도달 불가 영역의 프론티어 제외
    k = int(math.ceil(config.BLACKLIST_RADIUS / grid.res))
    for point in blacklist:
        if not _finite(point):
            continue
        br, bc = win.local(point)
        ra, rb = max(0, int(br) - k), min(win.h, int(br) + k + 2)
        ca, cb = max(0, int(bc) - k), min(win.w, int(bc) + k + 2)
        if ra >= rb or ca >= cb:
            continue
        rr, cc = np.ogrid[ra:rb, ca:cb]
        near = (rr + 0.5 - br) ** 2 + (cc + 0.5 - bc) ** 2 <= (
            config.BLACKLIST_RADIUS / grid.res
        ) ** 2
        size_grid[ra:rb, ca:cb][near] = 0
    max_size = int(size_grid.max())
    if max_size == 0:
        return None
    sizes = size_grid.ravel().tolist()
    res = grid.res
    min_cells = config.FRONTIER_MIN_DIST / res
    best = [0.0, -1]  # [점수, 칸 번호]

    def on_pop(idx: int, d: float) -> bool:
        if sizes[idx] and d >= min_cells:
            score = sizes[idx] / (d * res)
            if score > best[0]:
                best[0], best[1] = score, idx
        return best[1] >= 0 and best[0] * max(d, min_cells) * res >= max_size

    _, parent = _search(win, source, on_pop=on_pop)
    if best[1] < 0:
        return None
    path = _to_path(win, _unwind(parent, best[1]), (pose[0], pose[1]), snapped, None)
    return win.world(best[1]), path


def choose_frontier(
    grid: GridMap | None,
    pose: Sequence[float] | None,
    blacklist: Iterable[Point] = (),
) -> Point | None:
    """점수 (프론티어 묶음 칸 수 / 경로 비용)가 가장 큰 프론티어 칸의 (x, y). 없으면 None.

    경로도 필요하면 choose_frontier_path()를 사용. 같은 계산으로 경로를 함께 반환함.
    """
    found = choose_frontier_path(grid, pose, blacklist)
    return None if found is None else found[0]


if __name__ == "__main__":
    # Webots 없이 실행: controllers/sar_main에서 python -m sar.planner
    # 벽으로 나뉜 방 2개, 벽 한가운데 문. 전체를 확인한 지도에서 경로와 복귀 비용 확인
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
    gm.invalidate()

    path = plan(gm, (-2.0, 1.5), (2.0, 1.5))
    assert path is not None and path[0] == (-2.0, 1.5) and path[-1] == (2.0, 1.5)
    assert any(abs(x) < 0.1 and abs(y) < 0.4 for x, y in path), "문을 지나야 함"
    field = DistanceField(gm, (2.0, 1.5))
    d = field.distance((-2.0, 1.5))
    assert d is not None and d > 4.0
    back = field.path((-2.0, 1.5))
    assert back is not None and back[-1] == (2.0, 1.5)
    assert plan(gm, (-2.0, 1.5), (2.0, 1.5), field=field) is not None
    gm.logodds[door_a : door_b + 1, mid_c] = config.L_MAX  # 문을 닫으면 이전 거리 지도는 무효
    gm.invalidate()
    assert not field.is_current() and field.path((-2.0, 1.5)) is None
    assert plan(gm, (-2.0, 1.5), (2.0, 1.5), field=field) is None
    print(f"planner self-test ok, path {len(path)} points, {path_length(path):.2f} m, cost {d:.2f}")
