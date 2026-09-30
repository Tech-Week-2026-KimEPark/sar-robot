"""차동 구동 로봇의 Wheel Odometry.

강의 자료 슬라이드 59~62의 식을 따른다.
- 바퀴 이동 거리: d = R * phi
- 로봇 이동 거리·회전각: ds = (d_r + d_l) / 2, dtheta = (d_r - d_l) / L
"""

import math

from sar.geometry import Pose, normalize_angle

# 직진 판정 기준 회전각 [rad]. 이보다 작으면 원호 대신 직선으로 적분
_STRAIGHT_EPS = 1e-9


def integrate(pose: Pose, d_left: float, d_right: float, wheel_separation: float) -> Pose:
    """좌우 바퀴 이동 거리 [m]로 다음 pose를 계산."""
    ds = (d_right + d_left) / 2
    dtheta = (d_right - d_left) / wheel_separation
    if abs(dtheta) < _STRAIGHT_EPS:
        dx_local, dy_local = ds, 0.0
    else:
        rc = ds / dtheta
        dx_local = rc * math.sin(dtheta)
        dy_local = rc * (1 - math.cos(dtheta))
    cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
    return Pose(
        x=pose.x + cos_t * dx_local - sin_t * dy_local,
        y=pose.y + sin_t * dx_local + cos_t * dy_local,
        theta=normalize_angle(pose.theta + dtheta),
    )


class WheelOdometry:
    """바퀴 PositionSensor 누적 회전각 [rad]으로 pose를 갱신."""

    def __init__(self, wheel_radius: float, wheel_separation: float, start: Pose | None = None):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.pose = start or Pose()
        self._last: tuple[float, float] | None = None

    def update(self, left_angle: float, right_angle: float) -> Pose:
        if self._last is not None:
            d_left = (left_angle - self._last[0]) * self.wheel_radius
            d_right = (right_angle - self._last[1]) * self.wheel_radius
            self.pose = integrate(self.pose, d_left, d_right, self.wheel_separation)
        self._last = (left_angle, right_angle)
        return self.pose
