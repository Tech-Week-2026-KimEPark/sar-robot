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

# 창의성 제안(과제와 구현 기준 12장 "대피 인원 이동"): 정지 거리 안에 닿기 전부터
# 서서히 감속하고, 여유 있는 쪽으로 살짝 방향을 틀어 통로를 양보한다. 라이다는
# 장애물이 사람인지 구분하지 못하므로 이동하는 모든 근접 장애물에 동일하게 반응한다.
_SLOW_DIST = config.STOP_DIST + 0.30  # m, 이 거리부터 감속 시작
_YIELD_GAIN = 1.0  # rad/s per m, 좌우 여유 차이에 비례한 양보 회전

# 이동하는 사람 회피 설계(docs 저장소 human/explanation/사람-회피-설계.md) 4장 제안 값.
# 아직 config.py에 없어 이 모듈에 둔다.
_PREDICT_HORIZON = 3.0  # s, 충돌 예측 시간 T_h
_PERSON_SAFE_RADIUS = 0.50  # m, 로봇 반지름 + 사람 반경 + 여유
_MOVING_SPEED = 0.05  # m/s, 이 미만이면 정지 물체로 봄
_YIELD_DIST = 0.5  # m, 비켜서는 거리


def _wrap(angle: float) -> float:
    """각도를 atan2(sin, cos)로 [-pi, pi] 범위로 정리."""
    return math.atan2(math.sin(angle), math.cos(angle))


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

    `STOP_DIST`와 `_SLOW_DIST` 사이에서는 거리에 비례해 감속하고, 좌우 중 여유
    있는 쪽으로 살짝 방향을 틀어 통로를 양보한다 (대피 인원 등 이동 장애물 대응).

    반환: (v, w, blocked). `ranges`가 없으면(센서 미가동) 그대로 통과시킴. `v == 0`이면
    검사하지 않음. 라이다 인덱스 규칙(0=뒤, 90=왼쪽, 180=정면, 270=오른쪽)은
    CONTEXT.md 5.4절. 후진 미검사 문제는 sar-robot-병합-분석.md 6.1절 4번.
    """
    if not ranges or v == 0.0:
        return v, w, False

    n = len(ranges)
    center = n // 2 if v > 0.0 else 0
    half_span = round(n * _FRONT_HALF_ANGLE / (2 * math.pi))
    left = range(center - half_span, center + 1)  # 낮은 인덱스 = 왼쪽(90 방향)
    right = range(center, center + half_span + 1)  # 높은 인덱스 = 오른쪽(270 방향)
    left_min = min(ranges[i % n] for i in left)
    right_min = min(ranges[i % n] for i in right)
    nearest = min(left_min, right_min)

    if nearest < config.STOP_DIST:
        return 0.0, w, True

    if config.NEAR_SLOWDOWN and nearest < _SLOW_DIST:
        scale = (nearest - config.STOP_DIST) / (_SLOW_DIST - config.STOP_DIST)
        yield_w = _YIELD_GAIN * (left_min - right_min)  # 오른쪽이 좁으면 음수(우회전)
        return v * scale, max(-config.W_MAX, min(config.W_MAX, w + yield_w)), False

    return v, w, False


def predict_conflict(
    pose: tuple[float, float, float],
    robot_v: float,
    person: dict,
    horizon: float = _PREDICT_HORIZON,
) -> tuple[float, float]:
    """등속 가정으로 로봇과 사람의 최근접 시각·거리를 예측.

    `person`은 `PersonTracker`의 출력 항목 하나: `{"x", "y", "vx", "vy", ...}` (월드
    좌표, m/s). `robot_v`는 현재 진행 방향(`pose`의 theta) 기준 선속도 [m/s].
    반환: (t_star [s], d_min [m]). 사람-회피-설계.md 3.3절.
    """
    x, y, theta = pose
    rx, ry = person["x"] - x, person["y"] - y
    ux = person.get("vx", 0.0) - robot_v * math.cos(theta)
    uy = person.get("vy", 0.0) - robot_v * math.sin(theta)
    u_sq = ux * ux + uy * uy
    if u_sq < 1e-9:  # 상대 속도가 거의 없으면 현재 거리 유지로 봄
        return 0.0, math.hypot(rx, ry)
    t_star = max(0.0, min(horizon, -(rx * ux + ry * uy) / u_sq))
    d_min = math.hypot(rx + ux * t_star, ry + uy * t_star)
    return t_star, d_min


def _lidar_at(ranges: list[float] | None, direction: tuple[float, float], theta: float) -> float:
    """월드 방향 벡터 쪽 라이다 거리 1개를 읽음. `ranges`가 없으면 무한대."""
    if not ranges:
        return math.inf
    bearing = _wrap(math.atan2(direction[1], direction[0]) - theta)
    n = len(ranges)
    idx = round((math.pi - bearing) * n / (2 * math.pi)) % n
    return ranges[idx]


def yield_command(
    pose: tuple[float, float, float],
    ranges: list[float] | None,
    person: dict,
) -> tuple[float, float, bool]:
    """충돌 위험이 있는 사람을 비켜서는 속도 명령. 사람-회피-설계.md 3.4절.

    반환: (v, w, done). `done`이 참이면 사람이 멀어지는 중이거나 정지 상태라
    YIELD를 끝내도 됨. 매 step 다시 계산하므로 별도 내부 상태가 없다.
    """
    x, y, theta = pose
    rx, ry = person["x"] - x, person["y"] - y
    vx, vy = person.get("vx", 0.0), person.get("vy", 0.0)

    if rx * vx + ry * vy > 0:  # 사람이 로봇에서 멀어지는 중
        return 0.0, 0.0, True

    speed = math.hypot(vx, vy)
    if speed < _MOVING_SPEED:  # 정지 물체는 YIELD 대상이 아님(호출자가 걸러야 함)
        return 0.0, 0.0, True

    dir_x, dir_y = vx / speed, vy / speed
    left = (-dir_y, dir_x)
    right = (dir_y, -dir_x)
    left_clear = _lidar_at(ranges, left, theta)
    right_clear = _lidar_at(ranges, right, theta)
    open_side = left if left_clear >= right_clear else right
    open_clear = max(left_clear, right_clear)

    if open_clear >= _YIELD_DIST + config.ROBOT_RADIUS + config.SAFETY_MARGIN:
        # 여유 있는 쪽으로 비켜섬
        bearing = _wrap(math.atan2(open_side[1], open_side[0]) - theta)
        if abs(bearing) > _PIVOT_ANGLE:
            return 0.0, math.copysign(config.W_MAX, bearing), False
        return config.V_APPROACH, 0.0, False

    # 양쪽 다 좁음(복도): 사람이 오는 방향으로 후진해 물러남
    return -config.V_APPROACH, 0.0, False


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

    # 대피 인원 이동: 정지 거리 도달 전 감속, 막힌 쪽 반대로 양보
    ranges = [3.0] * 360
    ranges[200] = (config.STOP_DIST + _SLOW_DIST) / 2  # 오른쪽만 좁음
    v, w, blocked = safety_filter(config.V_MAX, 0.0, ranges)
    assert not blocked and 0.0 < v < config.V_MAX and w > 0.0

    # 충돌 예측: 정면으로 마주 오는 사람은 t_star > 0이고 최근접 거리가 작음
    t_star, d_min = predict_conflict(
        (0.0, 0.0, 0.0), config.V_MAX, {"x": 1.0, "y": 0.0, "vx": -0.3, "vy": 0.0}
    )
    assert t_star > 0.0 and d_min < 0.05
    # 멀어지는 사람은 t_star = 0(현재 거리 유지)
    t_star, d_min = predict_conflict(
        (0.0, 0.0, 0.0), 0.0, {"x": 1.0, "y": 0.0, "vx": 0.3, "vy": 0.0}
    )
    assert t_star == 0.0 and abs(d_min - 1.0) < 1e-9

    # 비키기: 다가오는 사람에게는 정지하지 않고 옆으로 비켜섬
    v, w, done = yield_command(
        (0.0, 0.0, 0.0), [3.0] * 360, {"x": 1.0, "y": 0.0, "vx": -0.3, "vy": 0.0}
    )
    assert not done
    # 멀어지는 사람은 done
    _, _, done = yield_command(
        (0.0, 0.0, 0.0), [3.0] * 360, {"x": 1.0, "y": 0.0, "vx": 0.3, "vy": 0.0}
    )
    assert done
    print("local_control self-test ok")
