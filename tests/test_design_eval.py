"""scripts/design_eval.py의 계산 함수 확인 (Webots 기록 불필요)."""

import math

import numpy as np

from design_eval import (
    carpet_rect,
    clear_steps,
    fit_compass_rms,
    red_apples,
    replay_odometry,
    safety_run,
    world_objects,
)
from sar import config, local_control
from sar.mission import fit_compass


def test_world_objects_reads_red_apples_and_skips_table_fruit():
    objects = world_objects()
    assert red_apples(objects) == [(-12.02, -3.02), (-5.34, -10.54)]
    assert all(name != "Apple" for name, _, _ in objects)  # 식탁 위 Apple 모델 제외
    x0, y0, x1, y1 = carpet_rect()
    assert (round(x1 - x0, 3), round(y1 - y0, 3)) == (1.6, 2.4)


def _synthetic_run() -> dict:
    """제자리 1회전 후 직진. 엔코더와 나침반(Webots 규약)이 실제 운동과 일치."""
    w_spin = config.INIT_SPIN_W
    wheel = w_spin * config.WHEEL_SEPARATION / 2 / config.WHEEL_RADIUS * 0.064
    spin_steps = int(round(2 * math.pi / (w_spin * 0.064))) + 1
    theta = config.START_THETA
    enc_l = enc_r = 0.0
    x, y = config.START_X, config.START_Y
    rows = {"enc": [], "compass": [], "state": [], "gt": []}
    for i in range(spin_steps + 100):
        spinning = i < spin_steps
        if i > 0:
            if spinning:
                enc_l, enc_r, theta = enc_l - wheel, enc_r + wheel, theta + w_spin * 0.064
            else:
                enc_l, enc_r = enc_l + 0.3, enc_r + 0.3
                x += 0.3 * config.WHEEL_RADIUS * math.cos(theta)
                y += 0.3 * config.WHEEL_RADIUS * math.sin(theta)
        raw = math.pi / 2 - theta
        rows["enc"].append((enc_l, enc_r))
        rows["compass"].append((math.cos(raw), math.sin(raw), 0.0))
        rows["state"].append(0 if i < spin_steps - 1 else 1)
        rows["gt"].append((x, y, theta))
    return {k: np.asarray(v, dtype=float) for k, v in rows.items()}


def test_replay_odometry_tracks_consistent_sensors():
    run = _synthetic_run()
    for variant in ("enc", "kf_scale", "compass"):
        poses = replay_odometry(run, variant)
        err = np.hypot(poses[:, 0] - run["gt"][:, 0], poses[:, 1] - run["gt"][:, 1])
        assert err.max() < 1e-6, variant


def test_logodds_clamp_bounds_clear_time():
    assert clear_steps(5, clamp=True) == clear_steps(100, clamp=True) == 8
    assert clear_steps(100, clamp=False) > 5 * clear_steps(5, clamp=False)


def test_safety_half_angle_blind_spot():
    assert safety_run(0.12, math.radians(25)) < 0  # 25°: 측면 0.12 m 기둥과 충돌
    assert safety_run(0.12, local_control._FRONT_HALF_ANGLE) > 0


def test_compass_fit_methods_on_rotation_scale_error():
    n = int(round(2 * math.pi / (config.INIT_SPIN_W * 0.064)))
    true = config.START_THETA + config.INIT_SPIN_W * 0.064 * np.arange(n)
    odom = config.START_THETA + 1.104 * (true - config.START_THETA)
    samples = [
        (math.remainder(a, 2 * math.pi), math.pi / 2 - b) for a, b in zip(odom, true, strict=True)
    ]
    assert fit_compass_rms(samples) is None  # 기존 방식: 회전 비율 1.104에서 실패
    sign, offset, scale = fit_compass(samples)
    assert sign == -1 and abs(offset - math.pi / 2) < 1e-6 and abs(scale - 1.104) < 0.01
