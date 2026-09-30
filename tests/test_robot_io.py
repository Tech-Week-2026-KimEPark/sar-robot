import pytest

from sar import config
from sar.robot_io import wheel_speeds


def test_straight():
    speed = 0.1 / config.WHEEL_RADIUS
    assert wheel_speeds(0.1, 0.0) == pytest.approx((speed, speed))


def test_rotate_left_in_place():
    wl, wr = wheel_speeds(0.0, 1.0)
    assert wl == pytest.approx(-wr)
    assert wr == pytest.approx(config.WHEEL_SEPARATION / 2 / config.WHEEL_RADIUS)


def test_scale_keeps_ratio():
    wl, wr = wheel_speeds(1.0, 0.5)
    raw_l, raw_r = 1.0 - 0.5 * config.WHEEL_SEPARATION / 2, 1.0 + 0.5 * config.WHEEL_SEPARATION / 2
    assert max(abs(wl), abs(wr)) == pytest.approx(config.MAX_WHEEL_SPEED)
    assert wl / wr == pytest.approx(raw_l / raw_r)
