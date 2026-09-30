"""planner와 local_control을 연결한 합성 주행 확인 (Webots 미사용).

순간 이동 대신 pure_pursuit·safety_filter의 속도 명령을 64 ms 간격으로 적분한다.
매 스텝 실제 벽과 로봇 몸체의 충돌을 검사하고 목표 도달과 시간 초과를 확인한다.
위치는 오차 없는 값을 사용한다. 오도메트리 오차, 바퀴 미끄러짐, 가감속, 보행자는 포함하지
않으므로 Webots 주행 검증을 대체하지 않는다.
"""

import math

import numpy as np
from planner_checks import rect

from sar import config, planner
from sar.grid_map import GridMap, simulate_scan
from sar.local_control import pure_pursuit, safety_filter

DT = 0.064  # s, Webots basicTimeStep
REPLAN_PERIOD = 2.0  # s


def _truth(size: float = 8.0) -> tuple[GridMap, np.ndarray]:
    """x -3~3 / y -2~2 방 2개와 0.8 m 문. 지도는 빈 상태, 벽 격자는 주행 확인용 실제 벽."""
    gm = GridMap(0.0, 0.0, size=size)
    wall = np.zeros((gm.n, gm.n), bool)
    wall[rect(gm, -3.0, -2.0, 3.0, -2.0)] = wall[rect(gm, -3.0, 2.0, 3.0, 2.0)] = True
    wall[rect(gm, -3.0, -2.0, -3.0, 2.0)] = wall[rect(gm, 3.0, -2.0, 3.0, 2.0)] = True
    wall[rect(gm, 0.0, -2.0, 0.0, 2.0)] = True
    wall[rect(gm, 0.0, -0.4, 0.0, 0.4)] = False
    wall[rect(gm, -1.6, -1.0, -1.3, -0.7)] = True  # 방 안 장애물
    return gm, wall


class _Sim:
    """오차 없는 차동 구동 로봇. 매 스텝 실제 벽 칸(정사각형)과 몸체 원의 거리를 확인."""

    def __init__(self, gm: GridMap, wall: np.ndarray, pose):
        self.gm, self.wall = gm, wall
        self.pose = tuple(pose)
        self.time = 0.0
        self.min_gap = math.inf  # 몸체 표면과 벽 사이 최소 거리 [m]
        self.blocked_steps = 0
        rows, cols = np.nonzero(wall)
        self._wx = gm.x0 + (cols + 0.5) * gm.res
        self._wy = gm.y0 + (rows + 0.5) * gm.res

    def scan(self) -> list[float]:
        return simulate_scan(self.wall, self.gm.to_cell, self.pose)

    def step(self, v: float, w: float) -> None:
        x, y, th = self.pose
        th_mid = th + w * DT / 2
        self.pose = (x + v * math.cos(th_mid) * DT, y + v * math.sin(th_mid) * DT, th + w * DT)
        self.time += DT
        half = self.gm.res / 2
        dx = np.maximum(np.abs(self._wx - self.pose[0]) - half, 0.0)
        dy = np.maximum(np.abs(self._wy - self.pose[1]) - half, 0.0)
        self.min_gap = min(self.min_gap, float(np.hypot(dx, dy).min()) - config.ROBOT_RADIUS)

    def follow(self, next_path, timeout: float) -> bool:
        """next_path(pose) 경로를 REPLAN_PERIOD마다 다시 받아 추종. 도착하면 True."""
        path, last_plan = None, -math.inf
        start = self.time
        while self.time - start < timeout:
            ranges = self.scan()
            self.gm.update(self.pose, ranges)
            if path is None or self.time - last_plan >= REPLAN_PERIOD:
                path, last_plan = next_path(self.pose), self.time
                if path is None:
                    return False
            v, w, reached = pure_pursuit(self.pose, path)
            if reached:
                return True
            v, w, blocked = safety_filter(v, w, ranges)
            self.blocked_steps += blocked
            self.step(v, w)
        return False


def test_drive_to_goal_through_door_with_online_mapping():
    # 지도 없이 출발. 모르는 칸 통과를 허용해 목표로 이동하며 2초마다 재계획
    gm, wall = _truth()
    sim = _Sim(gm, wall, (-2.0, 1.2, math.pi))
    goal = (2.0, -1.0)
    arrived = sim.follow(lambda p: planner.plan(gm, p[:2], goal, allow_unknown=True), timeout=90.0)
    assert arrived, f"시간 초과 또는 경로 없음 (t={sim.time:.1f}s, pose={sim.pose})"
    assert math.dist(sim.pose[:2], goal) < config.GOAL_TOL
    assert sim.min_gap > 0.0, f"충돌: 몸체와 벽 거리 {sim.min_gap:.3f} m"


def test_explore_then_return_home():
    # 프론티어 탐색(선택과 경로를 한 번에 계산) 후 거리 지도 휴리스틱으로 시작점 복귀
    gm, wall = _truth()
    home = (-2.0, 1.2)
    sim = _Sim(gm, wall, (*home, 0.0))
    gm.update(sim.pose, sim.scan())
    for _ in range(40):  # 목표 선택 횟수 상한
        found = planner.choose_frontier_path(gm, sim.pose, [])
        if found is None or sim.time > 300.0:
            break
        target = found[0]
        sim.follow(lambda p, t=target: planner.plan(gm, p[:2], t), timeout=25.0)
    assert planner.choose_frontier_path(gm, sim.pose, []) is None, "탐색 미완료"
    inside = np.zeros_like(wall)
    inside[rect(gm, -2.9, -1.9, 2.9, 1.9)] = True
    assert gm.seen[inside & ~wall].mean() > 0.97

    def back(p):
        field = planner.DistanceField(gm, home)
        return planner.plan(gm, p[:2], home, allow_unknown=True, field=field)

    assert sim.follow(back, timeout=90.0), f"복귀 실패 (pose={sim.pose})"
    assert math.dist(sim.pose[:2], home) < config.GOAL_TOL
    assert sim.min_gap > 0.0, f"충돌: 몸체와 벽 거리 {sim.min_gap:.3f} m"
    assert sim.time < 300.0
