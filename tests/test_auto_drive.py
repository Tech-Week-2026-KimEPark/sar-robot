import math
import os
import sys

import pytest

from sar import config

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "controllers", "hsv_tuner"))

from auto_drive import AutoApproach, SpinSurvey  # noqa: E402

POSE = (0.0, 0.0, 0.0)


def det(dist: float, bearing: float = 0.0) -> list[dict]:
    return [{"dist": dist, "bearing": bearing}]


def test_scan_rotates_then_gives_up_after_full_turn():
    auto = AutoApproach(0.0, POSE)
    cmd = auto.update(0.0, POSE, [], True)
    assert cmd.v == 0.0 and cmd.w == pytest.approx(config.TUNER_SCAN_W)
    theta, t, cmd = 0.0, 0.0, None
    for _ in range(100):  # 0.1 rad씩 회전
        theta += 0.1
        t += 0.1
        cmd = auto.update(t, (0.0, 0.0, theta), [], True)
        if cmd.done:
            break
    assert cmd.done
    assert theta >= config.TUNER_SCAN_TURNS * 2 * math.pi


def test_align_turns_toward_target():
    auto = AutoApproach(0.0, POSE)
    auto.update(0.0, POSE, det(2.5, 0.3), True)  # SCAN → ALIGN
    assert auto.state == "ALIGN"
    left = auto.update(0.1, POSE, det(2.5, 0.3), True)
    assert left.v == 0.0 and left.w > 0  # 왼쪽 대상 → 좌회전
    right = auto.update(0.2, POSE, det(2.5, -0.3), True)
    assert right.w < 0
    assert abs(left.w) <= config.TUNER_SCAN_W


def test_approach_measures_checkpoints_and_stops():
    auto = AutoApproach(0.0, POSE, dists=(3.0, 2.0, 1.0))
    auto.update(0.0, POSE, det(2.5), True)  # SCAN → ALIGN
    cmd = auto.update(0.1, POSE, det(2.5), True)  # 정렬 완료 → APPROACH
    assert auto.state == "APPROACH"
    assert "3.0" in cmd.message  # 이미 지난 3 m 지점 생략

    measures, t = [], 0.2
    for dist in [2.4, 2.1, 1.99, 1.5, 1.0, 0.8, 0.49]:
        cmd = auto.update(t, POSE, det(dist), True)
        t += 0.1
        if cmd.measure:
            measures.append(cmd.measure)
    assert measures == ["auto 2.0m d=1.99m", "auto 1.0m d=1.00m", "auto stop d=0.49m"]
    assert cmd.done


def test_approach_drives_forward_when_aligned():
    auto = AutoApproach(0.0, POSE)
    auto.update(0.0, POSE, det(2.5), True)
    auto.update(0.1, POSE, det(2.5), True)
    cmd = auto.update(0.2, POSE, det(2.4, 0.01), True)
    assert cmd.v == pytest.approx(config.TUNER_APPROACH_V)
    realign = auto.update(0.3, POSE, det(2.4, 0.5), True)
    assert realign.v == 0.0 and realign.w > 0


def test_no_measure_without_fresh_detection():
    auto = AutoApproach(0.0, POSE, dists=(2.0,))
    auto.update(0.0, POSE, det(2.5), True)
    auto.update(0.1, POSE, det(2.5), True)
    stale = auto.update(0.2, POSE, det(1.9), False)
    assert stale.measure is None
    assert auto.update(0.3, POSE, det(1.9), True).measure == "auto 2.0m d=1.90m"


def test_lost_target_returns_to_scan():
    auto = AutoApproach(0.0, POSE)
    auto.update(0.0, POSE, det(2.5), True)
    auto.update(0.1, POSE, det(2.5), True)
    wait = auto.update(0.5, POSE, [], True)
    assert (wait.v, wait.w, auto.state) == (0.0, 0.0, "APPROACH")
    cmd = auto.update(0.1 + config.TUNER_LOST_TIMEOUT, POSE, [], True)
    assert auto.state == "SCAN"
    assert "놓쳐" in cmd.message


def test_spin_survey_measures_every_step_for_one_turn():
    survey = SpinSurvey(0.0, POSE)
    measures, theta, cmd = [], 0.0, None
    for i in range(1000):
        cmd = survey.update(i * 0.1, (0.0, 0.0, theta), [], True)
        if cmd.measure:
            measures.append(cmd.measure)
        if cmd.done:
            break
        assert cmd.w == pytest.approx(config.TUNER_SCAN_W)
        theta += 0.05
    assert cmd.done
    expected = math.ceil(2 * math.pi / config.TUNER_SURVEY_STEP)
    assert len(measures) == expected
    assert measures[0] == "survey 0deg"
    assert measures[1] == "survey 30deg"
