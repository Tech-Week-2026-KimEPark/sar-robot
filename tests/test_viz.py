import numpy as np

from sar.viz import render_map, save_map

SIZE = 100


def grid_with_room() -> np.ndarray:
    grid = np.full((SIZE, SIZE), -1, np.int8)
    grid[40:60, 40:60] = 0
    grid[40, 40:60] = 1  # 방 아래쪽 벽 (y 최소)
    return grid


def to_cell(x: float, y: float) -> tuple[int, int]:
    return int(y) + SIZE // 2, int(x) + SIZE // 2


def test_crop_to_known_area_and_flip():
    image = render_map(grid_with_room(), to_cell, margin=0, scale=1)
    assert image.shape == (20, 20, 3)
    # row 40(벽)은 y가 가장 작으므로 그림 맨 아래 줄
    assert image[-1].tolist() == [[0, 0, 0]] * 20
    assert image[0, 0].tolist() == [255, 255, 255]


def test_empty_grid_shows_whole_map():
    grid = np.full((10, 12), -1, np.int8)
    image = render_map(grid, to_cell, margin=0, scale=2)
    assert image.shape == (20, 24, 3)


def test_header_lists_rescued_targets():
    plain = render_map(grid_with_room(), to_cell, margin=0, scale=4)
    with_header = render_map(
        grid_with_room(),
        to_cell,
        rescued=[(0.0, 0.0, 10.0), (5.0, 5.0, 20.0)],
        title="t=30.0s",
        margin=0,
        scale=4,
    )
    assert with_header.shape[1] == plain.shape[1]
    assert with_header.shape[0] > plain.shape[0]


def test_draws_overlays_without_error():
    image = render_map(
        grid_with_room(),
        to_cell,
        trajectory=[(0, 0), (5, 0), (5, 5)],
        path=[(5, 5), (-5, 5)],
        start=(0, 0),
        pose=(-5, 5),
        margin=5,
        scale=3,
    )
    assert image.dtype == np.uint8
    assert image.shape == (30 * 3, 30 * 3, 3)


def test_save_map(tmp_path):
    path = tmp_path / "out" / "map.png"
    assert save_map(str(path), np.zeros((4, 4, 3), np.uint8))
    assert path.exists()


def test_save_map_failure_returns_false(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("x")
    assert save_map(str(blocker / "map.png"), np.zeros((4, 4, 3), np.uint8)) is False
