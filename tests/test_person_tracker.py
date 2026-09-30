import math

import pytest

from sar import config
from sar.grid_map import GridMap
from sar.perception import PersonTracker

DT = 0.064
LEG_R = 0.06  # m, 다리 반지름
LEG_GAP = 0.20  # m, 두 다리 중심 거리


def known_free_grid() -> GridMap:
    grid = GridMap(0.0, 0.0, size=12.0)
    grid.seen[:] = True
    grid.logodds[:] = config.L_MIN  # 확인된 빈칸
    grid.invalidate()
    return grid


def scan_legs(robot, person):
    """로봇 (x, y, theta)에서 사람 중심 주변 다리 원 2개를 향한 라이다 360개."""
    x, y, theta = robot
    legs = [(person[0], person[1] - LEG_GAP / 2), (person[0], person[1] + LEG_GAP / 2)]
    ranges = []
    for i in range(360):
        a = theta + math.pi - i * 2 * math.pi / 360
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for lx, ly in legs:
            fx, fy = x - lx, y - ly
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - LEG_R**2
            disc = b * b - c
            if disc >= 0:
                t = -b - math.sqrt(disc)
                if 0 < t < best:
                    best = t
        ranges.append(best if best <= config.LIDAR_MAX else math.inf)
    return ranges


def test_tracks_walking_person_velocity():
    grid, tracker = known_free_grid(), PersonTracker()
    robot = (0.0, 0.0, 0.0)
    people = []
    for k in range(int(3.0 / DT)):
        t = k * DT
        person = (2.0, -1.0 + 0.2 * t)  # +y 방향 0.2 m/s
        people = tracker.update(t, robot, scan_legs(robot, person), grid)
    assert len(people) == 1
    p = people[0]
    assert (p["x"], p["y"]) == pytest.approx(person, abs=0.1)
    assert (p["vx"], p["vy"]) == pytest.approx((0.0, 0.2), abs=0.05)
    assert p["moving"] is True


def test_wall_points_are_not_dynamic():
    grid, tracker = known_free_grid(), PersonTracker()
    r, c = grid.to_cell(2.0, 0.0)
    grid.logodds[r - 20 : r + 21, c] = config.L_MAX  # x = 2 m 벽
    grid.invalidate()
    ranges = [math.inf] * 360
    ranges[180] = 2.0  # 정면 벽 반사
    for k in range(10):
        assert tracker.update(k * DT, (0.0, 0.0, 0.0), ranges, grid) == []
    assert PersonTracker.dynamic_points((0.0, 0.0, 0.0), ranges, grid) == []
