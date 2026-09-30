import math

import pytest

from sar import config
from sar.odometry import Odometry

R, L = config.WHEEL_RADIUS, config.WHEEL_SEPARATION


def drive(odom: Odometry, d_left: float, d_right: float) -> tuple[float, float, float]:
    """좌우 바퀴 이동 거리 [m]만큼 엔코더 값을 주고 pose를 반환."""
    odom.update(0.0, 0.0)
    odom.update(d_left / R, d_right / R)
    return odom.pose()


def test_straight():
    assert drive(Odometry(0.0, 0.0, 0.0), 0.5, 0.5) == pytest.approx((0.5, 0.0, 0.0))


def test_rotate_in_place():
    d = math.pi / 2 * L / 2  # 90° 제자리 회전 시 바퀴 이동 거리
    assert drive(Odometry(0.0, 0.0, 0.0), -d, d) == pytest.approx((0.0, 0.0, math.pi / 2))


def test_quarter_arc_left():
    # 반지름 1 m 원호로 90° 좌회전하면 (1, 1)에 도착
    r = 1.0
    pose = drive(Odometry(0.0, 0.0, 0.0), (r - L / 2) * math.pi / 2, (r + L / 2) * math.pi / 2)
    assert pose == pytest.approx((1.0, 1.0, math.pi / 2))


def test_start_pose_heading_west():
    # 시작 방향 π(서쪽)에서 직진하면 x가 감소
    x, y, _ = drive(Odometry(-0.3, -7.5, math.pi), 1.0, 1.0)
    assert (x, y) == pytest.approx((-1.3, -7.5))


def test_first_update_is_baseline():
    odom = Odometry(0.0, 0.0, 0.0)
    odom.update(10.0, 10.0)
    assert odom.pose() == (0.0, 0.0, 0.0)


def test_compass_update_moves_heading_and_reduces_variance():
    with_compass, without = Odometry(0.0, 0.0, 0.0), Odometry(0.0, 0.0, 0.0)
    for odom in (with_compass, without):
        odom.update(0.0, 0.0)
        odom.update(1.0, 1.0)
    with_compass.update(1.0, 1.0, compass=0.1)
    without.update(1.0, 1.0)
    assert 0.0 < with_compass.pose()[2] < 0.1
    assert with_compass.heading_var() < without.heading_var()


def test_compass_wraps_across_pi():
    # 방향 π 근처에서 나침반 -π+0.1은 +0.1 rad 차이로 처리
    odom = Odometry(0.0, 0.0, math.pi)
    odom.update(0.0, 0.0)
    odom.update(1.0, 1.0)
    odom.update(1.0, 1.0, compass=-math.pi + 0.1)
    assert abs(odom.pose()[2]) > 3.0


def test_correct():
    odom = Odometry(1.0, 2.0, 0.0)
    odom.correct(0.1, -0.2, 0.05)
    assert odom.pose() == pytest.approx((1.1, 1.8, 0.05))


def test_wheel_separation_scale_default_is_unchanged():
    d = math.pi / 2 * L / 2
    default_theta = drive(Odometry(0.0, 0.0, 0.0), -d, d)[2]

    scaled = Odometry(0.0, 0.0, 0.0)
    scaled.set_wheel_separation_scale(1.0)
    scaled.update(0.0, 0.0)
    scaled.update(-d / R, d / R)
    assert scaled.pose()[2] == pytest.approx(default_theta)


def test_wheel_separation_scale_above_one_reduces_measured_rotation():
    # mission.fit_compass()가 측정한 "오도메트리/나침반 회전 비율"(예: 1.104)을
    # 그대로 전달하면, 같은 바퀴 회전각 차이에서도 dth가 그만큼 줄어들어야 함
    d = math.pi / 2 * L / 2  # 보정 없이 90도에 해당하는 바퀴 이동거리
    scale = 1.104

    plain = Odometry(0.0, 0.0, 0.0)
    plain.update(0.0, 0.0)
    plain.update(-d / R, d / R)

    scaled = Odometry(0.0, 0.0, 0.0)
    scaled.set_wheel_separation_scale(scale)
    scaled.update(0.0, 0.0)
    scaled.update(-d / R, d / R)

    assert scaled.pose()[2] == pytest.approx(plain.pose()[2] / scale)
