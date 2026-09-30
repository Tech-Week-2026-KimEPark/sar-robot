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


def test_compass_pulls_heading_toward_measurement():
    odom = WheelOdometry(wheel_radius=0.033, wheel_separation=L, start=Pose(theta=0.0))
    pose = odom.update(0.0, 0.0, compass_theta=0.2)
    assert 0.0 < pose.theta < 0.2


def test_compass_updates_reduce_heading_variance():
    odom = WheelOdometry(wheel_radius=0.033, wheel_separation=L)
    var_before = odom.heading_var()
    odom.update(0.0, 0.0, compass_theta=0.0)
    assert odom.heading_var() < var_before


def test_no_compass_keeps_heading_from_encoders_only():
    odom = WheelOdometry(wheel_radius=0.033, wheel_separation=L)
    odom.update(10.0, 10.0)
    pose = odom.update(20.0, 20.0, compass_theta=None)
    assert pose.theta == pytest.approx(0.0)


def test_correct_applies_offset_directly():
    odom = WheelOdometry(wheel_radius=0.033, wheel_separation=L)
    odom.update(10.0, 10.0)
    odom.correct(dx=0.1, dy=-0.05, dtheta=0.02)
    pose = odom.pose
    assert (pose.x, pose.y, pose.theta) == pytest.approx((0.1, -0.05, 0.02))
