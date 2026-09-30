"""라이다 점유 격자 지도, 장애물 팽창, 프론티어 검출.

- 격자는 grid[row][col], row ↔ y, col ↔ x, 해상도 MAP_RES, 시작점 중심 MAP_SIZE 정사각형
- 칸 값은 log-odds l = ln(p / (1 - p)). 광선이 지나간 칸에 L_FREE, 맞은 칸에 L_OCC를 더하고
  [L_MIN, L_MAX]로 제한 (과제와 구현 기준 6장)
- 공개값은 -1 모름, 0 빈칸, 1 장애물
- 광선 추적은 numpy로 모든 빔을 한 번에 표본화. 스캔 1회에 같은 칸은 한 번만 갱신
- plan_version은 계획에 쓰는 분류(장애물 여부, 확인 여부)가 바뀔 때만 증가. log-odds 값만
  바뀐 갱신은 팽창·프론티어 캐시를 유지
"""

import math
from collections.abc import Sequence

import cv2
import numpy as np

from sar import config

UNKNOWN, FREE, OCCUPIED = -1, 0, 1

# 프론티어 검출용 4방향 이웃 커널
_CROSS = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], np.uint8)


class GridMap:
    def __init__(
        self,
        center_x: float = config.START_X,
        center_y: float = config.START_Y,
        size: float = config.MAP_SIZE,
        res: float = config.MAP_RES,
    ) -> None:
        self.res = res
        self.n = int(round(size / res))
        # 격자 (0, 0) 칸의 아래쪽 왼쪽 모서리 월드 좌표
        self.x0 = center_x - self.n * res / 2
        self.y0 = center_y - self.n * res / 2
        self.logodds = np.zeros((self.n, self.n), np.float32)
        self.seen = np.zeros((self.n, self.n), bool)
        self.version = 0  # update()로 값이 바뀔 때마다 증가
        self.plan_version = 0  # 장애물·확인 분류가 바뀔 때만 증가. 계획 결과의 유효 상태 식별
        self._cache: dict = {}

    # 좌표 변환

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        """월드 좌표 [m]가 속한 칸 (row, col). 지도 밖 좌표도 계산만 수행."""
        return int(math.floor((y - self.y0) / self.res)), int(math.floor((x - self.x0) / self.res))

    def to_world(self, row: int, col: int) -> tuple[float, float]:
        """칸 중심의 월드 좌표 (x, y) [m]."""
        return self.x0 + (col + 0.5) * self.res, self.y0 + (row + 0.5) * self.res

    def inside(self, row: int, col: int) -> bool:
        """칸이 지도 범위 안인지 여부."""
        return 0 <= row < self.n and 0 <= col < self.n

    # 갱신

    def update(self, pose: Sequence[float] | None, ranges: Sequence[float] | None) -> None:
        """로봇 pose (x, y, theta)와 라이다 거리 목록으로 log-odds 갱신. None이면 무시."""
        if pose is None or ranges is None or len(ranges) == 0:
            return
        x, y, theta = pose
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(theta)):
            return
        r = np.asarray(ranges, np.float64)
        count = len(r)
        # 인덱스 i의 로봇 좌표계 각도 = pi - i * 2pi / N (180 정면, 90 왼쪽)
        angles = theta + math.pi - np.arange(count) * (2 * math.pi / count)
        valid = ~np.isnan(r) & (r >= config.LIDAR_MIN)
        hit = valid & (r <= config.LIDAR_MAX)
        # 반사 없음(inf)과 최대 거리 초과는 LIDAR_MAX까지 빈칸으로만 처리
        free_len = np.where(hit, r - self.res / 2, config.LIDAR_MAX)
        free_len = np.where(valid, free_len, 0.0)

        t = np.arange(0.0, config.LIDAR_MAX, config.RAY_STEP)
        cos_a, sin_a = np.cos(angles), np.sin(angles)
        along = t[None, :] < free_len[:, None]
        fx = (x + t[None, :] * cos_a[:, None])[along]
        fy = (y + t[None, :] * sin_a[:, None])[along]
        free_idx = self._flat_cells(fx, fy)
        # 로봇 몸체 영역은 빈칸 (시작 칸이 모름으로 남아 계획이 실패하는 경우 방지)
        free_idx = np.concatenate([free_idx, self._footprint(x, y)])

        hx = x + r[hit] * cos_a[hit]
        hy = y + r[hit] * sin_a[hit]
        hit_idx = np.unique(self._flat_cells(hx, hy))
        free_idx = np.setdiff1d(np.unique(free_idx), hit_idx, assume_unique=True)

        flat = self.logodds.reshape(-1)
        seen = self.seen.reshape(-1)
        touched = np.concatenate([free_idx, hit_idx])
        occ_before = flat[touched] > config.OCC_THRESHOLD
        new_cells = not seen[touched].all()
        flat[free_idx] = np.maximum(flat[free_idx] + config.L_FREE, config.L_MIN)
        flat[hit_idx] = np.minimum(flat[hit_idx] + config.L_OCC, config.L_MAX)
        seen[touched] = True
        self.version += 1
        # 분류가 그대로이면 팽창·프론티어·계획 영역 캐시를 유지
        if new_cells or not np.array_equal(occ_before, flat[touched] > config.OCC_THRESHOLD):
            self.plan_version += 1
            self._cache.clear()

    def invalidate(self) -> None:
        """logodds·seen 배열을 직접 수정한 뒤 호출. 캐시를 지우고 버전을 증가."""
        self.version += 1
        self.plan_version += 1
        self._cache.clear()

    def _flat_cells(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """월드 좌표 배열의 1차원 칸 번호. 지도 밖 좌표는 제외."""
        rows = np.floor((ys - self.y0) / self.res).astype(np.int64)
        cols = np.floor((xs - self.x0) / self.res).astype(np.int64)
        ok = (rows >= 0) & (rows < self.n) & (cols >= 0) & (cols < self.n)
        return rows[ok] * self.n + cols[ok]

    def _footprint(self, x: float, y: float) -> np.ndarray:
        """로봇 반지름 안 칸의 1차원 칸 번호."""
        k = int(math.ceil(config.ROBOT_RADIUS / self.res))
        offsets = np.arange(-k, k + 1) * self.res
        ox, oy = np.meshgrid(offsets, offsets)
        inside = ox**2 + oy**2 <= config.ROBOT_RADIUS**2
        return self._flat_cells(x + ox[inside], y + oy[inside])

    # 조회

    def occupied(self) -> np.ndarray:
        """장애물 칸 bool 배열 (log-odds > OCC_THRESHOLD)."""
        if "occ" not in self._cache:
            self._cache["occ"] = self.logodds > config.OCC_THRESHOLD
        return self._cache["occ"]

    def clearance(self) -> np.ndarray:
        """각 칸 중심에서 가장 가까운 장애물 칸 중심까지 거리 [m]. 장애물이 없으면 큰 값."""
        if "clear" not in self._cache:
            occ = self.occupied()
            if not occ.any():
                self._cache["clear"] = np.full(occ.shape, np.inf, np.float32)
            else:
                free = (~occ).astype(np.uint8)
                # 정확한 유클리드 거리 (근사 마스크는 최대 2% 과대평가해 팽창이 부족해짐)
                dist = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
                self._cache["clear"] = dist * self.res
        return self._cache["clear"]

    def layers(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """(occ, blocked, soft, unknown) bool 배열.

        blocked: 장애물에서 INFLATE 이내 (로봇 중심 진입 불가)
        soft: blocked 바깥 WALL_BAND 폭 (벽 근처 추가 비용)
        """
        if "layers" not in self._cache:
            clear = self.clearance()
            blocked = clear < config.INFLATE
            soft = ~blocked & (clear < config.INFLATE + config.WALL_BAND)
            self._cache["layers"] = (self.occupied(), blocked, soft, ~self.seen)
        return self._cache["layers"]

    def public(self) -> np.ndarray:
        """공개값 격자 int8 배열 (-1 모름, 0 빈칸, 1 장애물). viz.render_map 입력."""
        grid = np.where(self.occupied(), OCCUPIED, FREE).astype(np.int8)
        grid[~self.seen] = UNKNOWN
        return grid

    def frontiers(self) -> list[tuple[int, list[tuple[int, int]]]]:
        """프론티어 묶음 [(칸 수, [(row, col), ...]), ...]. 칸 수 내림차순.

        프론티어 칸은 모르는 칸과 4방향으로 맞닿은 아는 빈칸. 8방향 연결로 묶고
        MIN_FRONTIER_CELLS 미만 묶음은 센서 잡음으로 제외.
        """
        if "frontiers" in self._cache:
            return list(self._cache["frontiers"])
        free = self.seen & ~self.occupied()
        unknown = (~self.seen).astype(np.uint8)
        near_unknown = cv2.dilate(unknown, _CROSS).astype(bool)
        mask = (free & near_unknown).astype(np.uint8)
        count, labels = cv2.connectedComponents(mask, connectivity=8)
        sizes = np.bincount(labels.ravel())[labels].astype(np.int32)
        sizes[(labels == 0) | (sizes < config.MIN_FRONTIER_CELLS)] = 0
        self._cache["frontier_sizes"] = sizes
        if count <= 1:
            self._cache["frontiers"] = []
            return []
        rows, cols = np.nonzero(labels)
        ids = labels[rows, cols]
        order = np.argsort(ids, kind="stable")
        rows, cols, ids = rows[order], cols[order], ids[order]
        splits = np.flatnonzero(np.diff(ids)) + 1
        result = []
        for rs, cs in zip(np.split(rows, splits), np.split(cols, splits), strict=True):
            if len(rs) >= config.MIN_FRONTIER_CELLS:
                result.append((len(rs), list(zip(rs.tolist(), cs.tolist(), strict=True))))
        result.sort(key=lambda item: -item[0])
        self._cache["frontiers"] = result
        return list(result)

    def frontier_sizes(self) -> np.ndarray:
        """칸마다 그 칸이 속한 프론티어 묶음의 칸 수 (int32). 프론티어가 아니면 0."""
        if "frontier_sizes" not in self._cache:
            self.frontiers()
        return self._cache["frontier_sizes"]


def simulate_scan(
    walls: np.ndarray, to_cell, pose: Sequence[float], count: int = 360
) -> list[float]:
    """벽 bool 격자에서 라이다 거리 목록 생성 (단독 테스트·pytest용). 반사 없으면 inf.

    to_cell은 GridMap.to_cell. 모든 빔을 RAY_STEP / 2 간격으로 표본화해 첫 벽 칸 거리를 반환.
    """
    x, y, theta = pose
    gm = to_cell.__self__
    step = config.RAY_STEP / 2
    t = np.arange(1, int(config.LIDAR_MAX / step) + 1) * step
    angles = theta + math.pi - np.arange(count) * (2 * math.pi / count)
    rows = np.floor((y + t[None, :] * np.sin(angles)[:, None] - gm.y0) / gm.res).astype(np.int64)
    cols = np.floor((x + t[None, :] * np.cos(angles)[:, None] - gm.x0) / gm.res).astype(np.int64)
    inside = (rows >= 0) & (rows < walls.shape[0]) & (cols >= 0) & (cols < walls.shape[1])
    hit = np.zeros(rows.shape, bool)
    hit[inside] = walls[rows[inside], cols[inside]]
    # 지도 밖으로 나간 뒤의 표본은 무시 (빔 종료)
    stop = hit | ~inside
    first = stop.argmax(axis=1)
    found = stop.any(axis=1) & hit[np.arange(count), first]
    return np.where(found, t[first], np.inf).tolist()


if __name__ == "__main__":
    # Webots 없이 실행: controllers/sar_main에서 python -m sar.grid_map
    # 시작점 중심 2 m x 2 m 방. 스캔 몇 회 후 벽은 장애물, 내부는 빈칸, 바깥은 모름
    gm = GridMap(0.0, 0.0, size=6.0)
    walls = np.zeros((gm.n, gm.n), bool)
    r0, c0 = gm.to_cell(-1.0, -1.0)
    r1, c1 = gm.to_cell(1.0, 1.0)
    walls[r0, c0 : c1 + 1] = walls[r1, c0 : c1 + 1] = True
    walls[r0 : r1 + 1, c0] = walls[r0 : r1 + 1, c1] = True
    scan = simulate_scan(walls, gm.to_cell, (0.0, 0.0, 0.0))
    for _ in range(3):
        gm.update((0.0, 0.0, 0.0), scan)
    grid = gm.public()
    assert grid[gm.to_cell(0.0, 0.0)] == FREE
    assert grid[gm.to_cell(1.0, 0.0)] == OCCUPIED
    assert grid[gm.to_cell(2.0, 0.0)] == UNKNOWN
    occ, blocked, soft, unknown = gm.layers()
    assert blocked[gm.to_cell(0.9, 0.0)] and not blocked[gm.to_cell(0.0, 0.0)]
    print("grid_map self-test ok, frontiers:", len(gm.frontiers()))
