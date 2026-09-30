"""지도·주행 궤적·계획 경로·구조 위치 그림 생성과 PNG 저장.

격자는 공개값 배열(-1 모름, 0 빈칸, 1 장애물)을 받는다. row 0이 y 최소이므로
그림에서는 상하를 뒤집어 +y가 위쪽이 되게 그린다.
"""

import os
from collections.abc import Callable, Sequence

import cv2
import numpy as np

from sar import config

ToCell = Callable[[float, float], tuple[int, int]]

# BGR 색
_UNKNOWN = (160, 160, 160)
_FREE = (255, 255, 255)
_OCCUPIED = (0, 0, 0)
_TRAJECTORY = (200, 120, 0)
_PATH = (0, 160, 0)
_RESCUED = (0, 0, 220)
_START = (200, 0, 200)
_ROBOT = (0, 140, 255)
_TEXT = (0, 0, 0)
_HEADER_BG = (235, 235, 235)

_FONT = cv2.FONT_HERSHEY_SIMPLEX
_FONT_SCALE = 0.4
_LINE_HEIGHT = 16  # px
_MARKER_RADIUS = 5  # px


def _known_bounds(grid: np.ndarray, margin: int) -> tuple[int, int, int, int]:
    """확인된 칸을 포함하는 (row_min, row_max, col_min, col_max). 여백 포함, 격자 범위로 제한."""
    rows, cols = np.nonzero(grid != -1)
    if rows.size == 0:
        return 0, grid.shape[0] - 1, 0, grid.shape[1] - 1
    return (
        max(int(rows.min()) - margin, 0),
        min(int(rows.max()) + margin, grid.shape[0] - 1),
        max(int(cols.min()) - margin, 0),
        min(int(cols.max()) + margin, grid.shape[1] - 1),
    )


def render_map(
    grid: np.ndarray,
    to_cell: ToCell,
    trajectory: Sequence[Sequence[float]] = (),
    path: Sequence[Sequence[float]] = (),
    rescued: Sequence[Sequence[float]] = (),
    start: Sequence[float] | None = None,
    pose: Sequence[float] | None = None,
    title: str = "",
    margin: int = config.MAP_VIEW_MARGIN,
    scale: int = config.MAP_VIEW_SCALE,
) -> np.ndarray:
    """지도 그림 (BGR uint8) 생성.

    trajectory·path·start·pose는 월드 좌표 (x, y, ...) 목록.
    rescued는 (x, y, t) 목록이며 구조 순서 번호와 시각을 표시.
    """
    grid = np.asarray(grid)
    r0, r1, c0, c1 = _known_bounds(grid, margin)
    view = grid[r0 : r1 + 1, c0 : c1 + 1]

    image = np.empty((*view.shape, 3), np.uint8)
    image[view == -1] = _UNKNOWN
    image[view == 0] = _FREE
    image[view == 1] = _OCCUPIED
    image = np.flipud(image)
    image = cv2.resize(
        image, (view.shape[1] * scale, view.shape[0] * scale), interpolation=cv2.INTER_NEAREST
    )

    def pixel(x: float, y: float) -> tuple[int, int]:
        row, col = to_cell(x, y)
        return (col - c0) * scale + scale // 2, (r1 - row) * scale + scale // 2

    if len(trajectory) >= 2:
        points = np.array([pixel(p[0], p[1]) for p in trajectory], np.int32)
        cv2.polylines(image, [points], False, _TRAJECTORY, 1, cv2.LINE_AA)
    if len(path) >= 2:
        points = np.array([pixel(p[0], p[1]) for p in path], np.int32)
        cv2.polylines(image, [points], False, _PATH, 2, cv2.LINE_AA)
    if start is not None:
        sx, sy = pixel(start[0], start[1])
        r = _MARKER_RADIUS
        cv2.rectangle(image, (sx - r, sy - r), (sx + r, sy + r), _START, 2)
    for i, item in enumerate(rescued, start=1):
        center = pixel(item[0], item[1])
        cv2.circle(image, center, _MARKER_RADIUS, _RESCUED, -1, cv2.LINE_AA)
        label_pos = (center[0] + _MARKER_RADIUS + 2, center[1] + _MARKER_RADIUS)
        cv2.putText(image, str(i), label_pos, _FONT, _FONT_SCALE, _RESCUED, 1, cv2.LINE_AA)
    if pose is not None:
        cv2.circle(image, pixel(pose[0], pose[1]), _MARKER_RADIUS, _ROBOT, 2, cv2.LINE_AA)

    lines = [title] if title else []
    lines += [f"#{i} ({x:.2f}, {y:.2f}) t={t:.1f}s" for i, (x, y, t) in enumerate(rescued, 1)]
    if not lines:
        return image
    header = np.full((_LINE_HEIGHT * len(lines) + 4, image.shape[1], 3), _HEADER_BG, np.uint8)
    for i, text in enumerate(lines):
        baseline = (4, _LINE_HEIGHT * (i + 1) - 2)
        cv2.putText(header, text, baseline, _FONT, _FONT_SCALE, _TEXT, 1, cv2.LINE_AA)
    return np.vstack([header, image])


def save_map(path: str, image: np.ndarray) -> bool:
    """그림을 PNG로 저장. 폴더가 없으면 생성. 실패하면 False."""
    try:
        folder = os.path.dirname(path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        return bool(cv2.imwrite(path, image))
    except (OSError, cv2.error):
        return False


if __name__ == "__main__":
    # Webots 없이 실행하는 단독 확인: 합성 격자 그림 저장
    demo = np.full((200, 200), -1, np.int8)
    demo[50:150, 50:150] = 0
    demo[50, 50:150] = demo[149, 50:150] = demo[50:150, 50] = demo[50:150, 149] = 1

    def demo_cell(x: float, y: float) -> tuple[int, int]:
        return int(y / config.MAP_RES) + 100, int(x / config.MAP_RES) + 100

    img = render_map(
        demo,
        demo_cell,
        trajectory=[(0, 0), (1, 0.5), (1.5, 1.5)],
        path=[(1.5, 1.5), (-1, 1.5), (-1, -1)],
        rescued=[(1.5, 1.5, 42.0)],
        start=(0, 0),
        pose=(-1, -1),
        title="demo",
    )
    out = os.path.join(config.MAP_SAVE_DIR, "viz_demo.png")
    print(out, save_map(out, img))
