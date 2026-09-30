import math

import pytest

from sar.geometry import Pose
from sar.localization.odometry import WheelOdometry, integrate

L = 0.16


def test_straight():
    pose = integrate(Pose(), 0.5, 0.5, L)
    assert (pose.x, pose.y, pose.theta) == pytest.approx((0.5, 0.0, 0.0))


def test_rotate_in_place():
    d = math.pi / 2 * L / 2  # 90° 제자리 회전 시 바퀴 이동 거리
    pose = integrate(Pose(), -d, d, L)
    assert (pose.x, pose.y, pose.theta) == pytest.approx((0.0, 0.0, math.pi / 2))


def test_quarter_arc_left():
    # 반지름 1 m 원호로 90° 좌회전하면 (1, 1)에 도착
    r = 1.0
    d_left = (r - L / 2) * math.pi / 2
    d_right = (r + L / 2) * math.pi / 2
    pose = integrate(Pose(), d_left, d_right, L)
    assert (pose.x, pose.y, pose.theta) == pytest.approx((1.0, 1.0, math.pi / 2))


def test_wheel_odometry_uses_angle_difference():
    odom = WheelOdometry(wheel_radius=0.033, wheel_separation=L)
    odom.update(10.0, 10.0)  # 첫 값은 기준값
    pose = odom.update(20.0, 20.0)
    assert pose.x == pytest.approx(10.0 * 0.033)
