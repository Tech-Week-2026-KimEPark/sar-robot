"""좌표·단위 규칙과 좌표 변환.

- 월드 좌표: (x, y) [m], 방향 theta [rad], 반시계 방향이 양수
- 격자 좌표: grid[row][col], row는 y, col은 x에 대응
- 격자 (0, 0)의 좌하단 모서리가 월드 좌표 origin
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


def normalize_angle(angle: float) -> float:
    """각도를 [-pi, pi) 범위로 변환."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def world_to_grid(
    x: float, y: float, origin: tuple[float, float], resolution: float
) -> tuple[int, int]:
    """월드 좌표 (x, y)를 격자 (row, col)로 변환."""
    col = math.floor((x - origin[0]) / resolution)
    row = math.floor((y - origin[1]) / resolution)
    return row, col


def grid_to_world(
    row: int, col: int, origin: tuple[float, float], resolution: float
) -> tuple[float, float]:
    """격자 (row, col)의 중심 월드 좌표를 반환."""
    x = origin[0] + (col + 0.5) * resolution
    y = origin[1] + (row + 0.5) * resolution
    return x, y


def to_robot_frame(px: float, py: float, pose: Pose) -> tuple[float, float]:
    """월드 좌표 점을 로봇 기준 좌표로 변환. x는 전방, y는 좌측 거리."""
    dx, dy = px - pose.x, py - pose.y
    cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
    return cos_t * dx + sin_t * dy, -sin_t * dx + cos_t * dy
