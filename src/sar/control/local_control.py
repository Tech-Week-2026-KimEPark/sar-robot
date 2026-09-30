"""경로 추종(pure pursuit)과 라이다 기반 안전 필터.

CONTEXT.md 9장 설계 결정을 따른다: look-ahead 0.35 m, 목표점과 각도 차이가
55˚ 이상이면 전진 대신 제자리 회전으로 방향을 맞춘다.
"""

import math

from sar.geometry import Pose, to_robot_frame

# CONTEXT.md 10장 기본값
_V_MAX = 0.18  # m/s
_W_MAX = 1.8  # rad/s
_LOOKAHEAD = 0.35  # m
_GOAL_TOL = 0.15  # m
_PIVOT_ANGLE = math.radians(55)  # 이보다 많이 틀어지면 제자리 회전
_STOP_DIST = 0.20  # m, 정면 즉시 정지 거리
_FRONT_HALF_ANGLE = math.radians(25)  # 정면 장애물 판정 반각


def _lookahead_point(pose: Pose, path: list[tuple[float, float]], lookahead: float) -> tuple:
    """`path`에서 로봇으로부터 `lookahead` 이상 떨어진 첫 점을 찾음. 없으면 마지막 점."""
    for x, y in path:
        if math.hypot(x - pose.x, y - pose.y) >= lookahead:
            return x, y
    return path[-1]


def pure_pursuit(
    pose: Pose,
    path: list[tuple[float, float]] | None,
    lookahead: float = _LOOKAHEAD,
) -> tuple[float, float, bool]:
    """경로를 따라가는 속도 명령을 계산.

    반환: (v, w, reached). `path`가 비어 있으면 정지 상태로 도착 처리함.
    """
    if not path:
        return 0.0, 0.0, True

    goal_x, goal_y = path[-1]
    if math.hypot(goal_x - pose.x, goal_y - pose.y) < _GOAL_TOL:
        return 0.0, 0.0, True

    target_x, target_y = _lookahead_point(pose, path, lookahead)
    local_x, local_y = to_robot_frame(target_x, target_y, pose)
    bearing = math.atan2(local_y, local_x)

    if abs(bearing) > _PIVOT_ANGLE:
        w = math.copysign(_W_MAX, bearing)
        return 0.0, w, False

    distance = math.hypot(local_x, local_y)
    if distance < 1e-6:
        return 0.0, 0.0, False

    curvature = 2 * local_y / (distance**2)
    v = _V_MAX
    w = max(-_W_MAX, min(_W_MAX, v * curvature))
    return v, w, False


def safety_filter(v: float, w: float, ranges: list[float] | None) -> tuple[float, float, bool]:
    """정면 장애물이 정지 거리 안에 있으면 전진을 막음.

    반환: (v, w, blocked). `ranges`가 없으면(센서 미가동) 그대로 통과시킴.
    라이다 인덱스 규칙(0=뒤, 90=왼쪽, 180=정면, 270=오른쪽)은 CONTEXT.md 5.4절.
    """
    if not ranges or v <= 0.0:
        return v, w, False

    n = len(ranges)
    front_index = n // 2
    half_span = round(n * _FRONT_HALF_ANGLE / (2 * math.pi))
    front_ranges = [
        ranges[i % n] for i in range(front_index - half_span, front_index + half_span + 1)
    ]
    front_min = min(front_ranges, default=math.inf)

    if front_min < _STOP_DIST:
        return 0.0, w, True
    return v, w, False
