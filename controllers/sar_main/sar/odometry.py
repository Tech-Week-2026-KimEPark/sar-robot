"""엔코더 오도메트리와 방향 1차원 칼만 필터.

- 바퀴 이동 거리 d = R·phi, 로봇 이동 ds = (d_r + d_l)/2, 회전 dtheta = (d_r - d_l)/L
  (강의 자료 슬라이드 59~62)
- 방향 칼만 필터: 예측(엔코더 dtheta, P += Q) → 갱신(나침반 방향 z, K = P/(P+R))
"""

import math

from sar import config

# 직진 판정 기준 회전각 [rad]. 이보다 작으면 원호 대신 직선으로 적분
_STRAIGHT_EPS = 1e-9


def wrap(angle: float) -> float:
    """각도를 atan2(sin, cos)로 [-pi, pi] 범위로 정리."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Odometry:
    def __init__(self, x: float, y: float, theta: float) -> None:
        self.x, self.y, self.theta = x, y, wrap(theta)
        self.p = 0.0  # 방향 분산 [rad^2]. 시작 방향은 알고 있으므로 0
        self._last: tuple[float, float] | None = None

    def update(self, enc_l: float, enc_r: float, compass: float | None = None) -> None:
        """누적 바퀴 회전각 [rad]으로 pose 갱신. compass는 보정된 나침반 방향 [rad]."""
        if self._last is not None:
            d_l = (enc_l - self._last[0]) * config.WHEEL_RADIUS
            d_r = (enc_r - self._last[1]) * config.WHEEL_RADIUS
            ds = (d_r + d_l) / 2
            dth = (d_r - d_l) / config.WHEEL_SEPARATION
            if abs(dth) < _STRAIGHT_EPS:
                fx, fy = ds, 0.0
            else:
                rc = ds / dth
                fx, fy = rc * math.sin(dth), rc * (1 - math.cos(dth))
            c, s = math.cos(self.theta), math.sin(self.theta)
            self.x += c * fx - s * fy
            self.y += s * fx + c * fy
            self.theta = wrap(self.theta + dth)
            self.p += config.HEADING_Q
        self._last = (enc_l, enc_r)
        if compass is not None:
            k = self.p / (self.p + config.HEADING_R)
            self.theta = wrap(self.theta + k * wrap(compass - self.theta))
            self.p = (1 - k) * self.p

    def pose(self) -> tuple[float, float, float]:
        """(x [m], y [m], theta [rad])."""
        return self.x, self.y, self.theta

    def heading_var(self) -> float:
        """방향 분산 P [rad^2]."""
        return self.p

    def correct(self, dx: float, dy: float, dth: float) -> None:
        """스캔 매칭 보정값 적용."""
        self.x += dx
        self.y += dy
        self.theta = wrap(self.theta + dth)


if __name__ == "__main__":
    # Webots 없이 실행: controllers/sar_main에서 python -m sar.odometry
    odom = Odometry(0.0, 0.0, 0.0)
    odom.update(0.0, 0.0)
    odom.update(10.0, 10.0)  # 직진 10 rad
    x, y, th = odom.pose()
    assert abs(x - 10.0 * config.WHEEL_RADIUS) < 1e-9 and abs(y) < 1e-9 and abs(th) < 1e-9
    predicted = odom.heading_var() + config.HEADING_Q  # 다음 호출의 예측 분산
    odom.update(10.0, 10.0, compass=0.1)  # 나침반 갱신 시 방향이 z 쪽으로 이동, 분산 감소
    assert 0.0 < odom.pose()[2] < 0.1 and odom.heading_var() < predicted
    print("odometry self-test ok")
