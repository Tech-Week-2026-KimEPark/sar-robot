"""차동 구동 로봇의 Wheel Odometry.

강의 자료 슬라이드 59~62의 식을 따른다.
- 바퀴 이동 거리: d = R * phi
- 로봇 이동 거리·회전각: ds = (d_r + d_l) / 2, dtheta = (d_r - d_l) / L

방향은 나침반 관측으로 보정하는 1차원 칼만 필터를 함께 적용한다. 예측 단계는
엔코더로 계산한 회전량, 갱신 단계는 나침반이 알려주는 절대 방향이다.
"""

import math
from dataclasses import replace

from sar.geometry import Pose, normalize_angle

# 직진 판정 기준 회전각 [rad]. 이보다 작으면 원호 대신 직선으로 적분
_STRAIGHT_EPS = 1e-9

# 방향 칼만 필터 잡음 기본값 (CONTEXT.md 10장)
_HEADING_Q = 0.01**2  # rad^2, 예측 단계 잡음 (1 스텝)
_HEADING_R = 0.05**2  # rad^2, 나침반 관측 잡음


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
    """바퀴 PositionSensor 누적 회전각 [rad]으로 pose를 갱신.

    방향은 나침반 관측이 주어질 때마다 1차원 칼만 필터로 보정한다. 위치(x, y)는
    항상 바퀴 오도메트리로만 적분하며, 나침반은 방향에만 반영한다.
    """

    def __init__(
        self,
        wheel_radius: float,
        wheel_separation: float,
        start: Pose | None = None,
        heading_q: float = _HEADING_Q,
        heading_r: float = _HEADING_R,
    ):
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation
        self.pose = start or Pose()
        self._last: tuple[float, float] | None = None
        self._heading_q = heading_q
        self._heading_r = heading_r
        self._heading_var = heading_r  # 초기 분산은 관측 잡음과 동일하게 둠

    def update(
        self, left_angle: float, right_angle: float, compass_theta: float | None = None
    ) -> Pose:
        """엔코더로 pose를 예측하고, `compass_theta`가 있으면 방향을 갱신.

        `compass_theta`는 이미 로봇 좌표계 방향[rad]으로 변환된 값이다. 나침반
        원시값(3축 벡터)의 변환과 부호·오프셋 보정은 이 모듈 밖에서 수행한다.
        """
        if self._last is not None:
            d_left = (left_angle - self._last[0]) * self.wheel_radius
            d_right = (right_angle - self._last[1]) * self.wheel_radius
            self.pose = integrate(self.pose, d_left, d_right, self.wheel_separation)
            self._heading_var += self._heading_q
        self._last = (left_angle, right_angle)
        if compass_theta is not None:
            self._correct_heading(compass_theta)
        return self.pose

    def _correct_heading(self, compass_theta: float) -> None:
        """칼만 필터 갱신 단계: 나침반 관측으로 방향 분산을 줄임."""
        gain = self._heading_var / (self._heading_var + self._heading_r)
        innovation = normalize_angle(compass_theta - self.pose.theta)
        new_theta = normalize_angle(self.pose.theta + gain * innovation)
        self.pose = replace(self.pose, theta=new_theta)
        self._heading_var *= 1.0 - gain

    def heading_var(self) -> float:
        """방향 분산 P [rad^2]. 디버깅·발표용."""
        return self._heading_var

    def correct(self, dx: float, dy: float, dtheta: float) -> None:
        """스캔-지도 매칭 등 외부에서 계산한 보정값을 pose에 직접 적용 (선택 기능 훅)."""
        self.pose = Pose(
            x=self.pose.x + dx,
            y=self.pose.y + dy,
            theta=normalize_angle(self.pose.theta + dtheta),
        )
