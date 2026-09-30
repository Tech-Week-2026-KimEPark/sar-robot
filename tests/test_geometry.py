import math

import pytest

from sar.geometry import Pose, grid_to_world, normalize_angle, to_robot_frame, world_to_grid


@pytest.mark.parametrize(
    ("angle", "expected"),
    [
        (0.0, 0.0),
        (math.pi, -math.pi),
        (3 * math.pi / 2, -math.pi / 2),
        (-3 * math.pi / 2, math.pi / 2),
    ],
)
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == pytest.approx(expected)


def test_world_grid_round_trip():
    origin, res = (-1.0, -2.0), 0.05
    row, col = world_to_grid(0.12, 0.33, origin, res)
    assert (row, col) == (46, 22)  # row는 y, col은 x
    x, y = grid_to_world(row, col, origin, res)
    assert world_to_grid(x, y, origin, res) == (row, col)


def test_to_robot_frame_heading_left():
    # 로봇이 +y 방향(90°)을 볼 때 월드 +y 1 m 점은 로봇 전방 1 m
    x, y = to_robot_frame(0.0, 1.0, Pose(0.0, 0.0, math.pi / 2))
    assert (x, y) == pytest.approx((1.0, 0.0))
