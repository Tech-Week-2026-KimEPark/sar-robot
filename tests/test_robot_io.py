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


def test_decode_keys():
    from sar.robot_io import decode_keys

    arrows = {315: "up", 314: "left"}
    names, shift = decode_keys([ord("W"), 315, 0x10000 | ord("A")], arrows, 0xFFFF, 0x10000)
    assert names == {"w", "up", "a"}
    assert shift is True
    assert decode_keys([], arrows, 0xFFFF, 0x10000) == (set(), False)
    assert decode_keys([4], arrows, 0xFFFF, 0x10000) == (set(), False)  # 제어 문자 무시
