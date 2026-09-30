"""경로 추종(pure pursuit)과 라이다 기반 안전 필터.

CONTEXT.md 9장 설계 결정을 따른다: look-ahead와 목표 반경은 config.py 값을 쓰고,
목표점과 각도 차이가 55° 이상이면 전진 대신 제자리 회전으로 방향부터 맞춘다.
"""

import math

from sar import config

# 이 값들은 config.py에 없는 구현 세부 값(튜닝 파라미터)이라 이 모듈에 둔다.
_PIVOT_ANGLE = math.radians(55)  # 이보다 많이 틀어지면 제자리 회전
# 정지 거리(STOP_DIST)에서 좌우 반폭이 로봇 반지름+SAFETY_MARGIN을 덮도록 계산.
# 하드코딩된 각도(25°)는 STOP_DIST=0.20m에서 반폭 0.093m로 로봇 반지름(0.105m)보다
# 좁아 모서리 방향 장애물을 놓칠 수 있었음 (sar-robot-병합-분석.md 6.3절).
_FRONT_HALF_ANGLE = math.atan((config.ROBOT_RADIUS + config.SAFETY_MARGIN) / config.STOP_DIST)


def _to_robot_frame(target: tuple[float, float], pose: tuple[float, float, float]) -> tuple:
    """월드 좌표 점을 로봇 기준 좌표로 변환. x는 전방, y는 좌측."""
    x, y, theta = pose
    dx, dy = target[0] - x, target[1] - y
    c, s = math.cos(theta), math.sin(theta)
    return c * dx + s * dy, -s * dx + c * dy


def _lookahead_point(
    pose: tuple[float, float, float], path: list[tuple[float, float]], lookahead: float
) -> tuple[float, float]:
    """로봇과 가장 가까운 경로점 이후에서 `lookahead` 이상 떨어진 첫 점을 찾음.

    경로 첫 점부터 검색하면 로봇이 이미 지나온 점(뒤쪽)이 선택될 수 있어, 가장
    가까운 점의 인덱스부터 검색을 시작한다 (sar-robot-병합-분석.md 6.1절 1번).
    없으면 마지막 점.
    """
    x, y, _ = pose
    nearest_idx = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - x, path[i][1] - y))
    for px, py in path[nearest_idx:]:
        if math.hypot(px - x, py - y) >= lookahead:
            return px, py
    return path[-1]


def pure_pursuit(
    pose: tuple[float, float, float],
    path: list[tuple[float, float]] | None,
    lookahead: float = config.LOOKAHEAD,
) -> tuple[float, float, bool]:
    """경로를 따라가는 속도 명령을 계산.

    `pose`는 `Odometry.pose()`의 (x, y, theta). `path`는 월드 좌표 [(x, y), ...].
    반환: (v, w, reached). `path`가 비어 있으면 정지 상태로 도착 처리함.
    """
    if not path:
        return 0.0, 0.0, True

    x, y, _ = pose
    goal_x, goal_y = path[-1]
    if math.hypot(goal_x - x, goal_y - y) < config.GOAL_TOL:
        return 0.0, 0.0, True

    target = _lookahead_point(pose, path, lookahead)
    local_x, local_y = _to_robot_frame(target, pose)
    bearing = math.atan2(local_y, local_x)

    if abs(bearing) > _PIVOT_ANGLE:
        return 0.0, math.copysign(config.W_MAX, bearing), False

    distance = math.hypot(local_x, local_y)
    if distance < 1e-6:
        return 0.0, 0.0, False

    curvature = 2 * local_y / (distance**2)
    v = config.V_MAX
    w = max(-config.W_MAX, min(config.W_MAX, v * curvature))
    return v, w, False


def safety_filter(v: float, w: float, ranges: list[float] | None) -> tuple[float, float, bool]:
    """진행 방향(전진 시 정면, 후진 시 후면)에 장애물이 정지 거리 안에 있으면 막음.

    반환: (v, w, blocked). `ranges`가 없으면(센서 미가동) 그대로 통과시킴. `v == 0`이면
    검사하지 않음. 라이다 인덱스 규칙(0=뒤, 90=왼쪽, 180=정면, 270=오른쪽)은
    CONTEXT.md 5.4절. 후진 미검사 문제는 sar-robot-병합-분석.md 6.1절 4번.
    """
    if not ranges or v == 0.0:
        return v, w, False

    n = len(ranges)
    center = n // 2 if v > 0.0 else 0
    half_span = round(n * _FRONT_HALF_ANGLE / (2 * math.pi))
    nearest = min(ranges[i % n] for i in range(center - half_span, center + half_span + 1))

    if nearest < config.STOP_DIST:
        return 0.0, w, True
    return v, w, False


if __name__ == "__main__":
    # Webots 없이 실행: controllers/sar_main에서 python -m sar.local_control
    v, w, reached = pure_pursuit((0.0, 0.0, 0.0), [(1.0, 0.0)])
    assert v > 0.0 and abs(w) < 1e-9 and not reached
    v, w, reached = pure_pursuit((0.0, 0.0, 0.0), [(-1.0, 0.0)])
    assert v == 0.0 and w != 0.0 and not reached
    v, w, reached = pure_pursuit((1.0, 1.0, 0.0), [(1.05, 1.0)])
    assert (v, w, reached) == (0.0, 0.0, True)

    ranges = [3.0] * 360
    v, w, blocked = safety_filter(config.V_MAX, 0.0, ranges)
    assert not blocked
    ranges[180] = 0.1
    v, w, blocked = safety_filter(config.V_MAX, 0.0, ranges)
    assert v == 0.0 and blocked
    print("local_control self-test ok")
