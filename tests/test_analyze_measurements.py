import os
import sys

import pytest

import analyze_measurements as am
from sar import config
from sar.perception import focal_length_px

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "controllers", "hsv_tuner"))

import hsv_tuner  # noqa: E402

TARGETS = [(2.0, 0.0), (-5.0, 5.0)]


def target_row(dist, world_xy, pose=(0.0, 0.0, 0.0), note="auto 2.0m"):
    return {
        "kind": "target",
        "note": note,
        "source": "yolo",
        "cls_name": "apple",
        "robot_x": pose[0],
        "robot_y": pose[1],
        "robot_theta_deg": pose[2],
        "dist": dist,
        "world_x": world_xy[0],
        "world_y": world_xy[1],
    }


def raw_row(dist, ratio, cx=320.0):
    size = focal_length_px(am.IMAGE_WIDTH) * config.TARGET_DIAMETER / dist
    return {
        "kind": "yolo_raw",
        "cls_name": "apple",
        "conf": 0.5,
        "color_ratio": ratio,
        "robot_x": 0.0,
        "robot_y": 0.0,
        "robot_theta_deg": 0.0,
        "cx": cx,
        "w": size,
        "h": size,
    }


def test_parse_target():
    assert am.parse_target("-12.02,-3.02") == (-12.02, -3.02)


def test_analyze_targets_error_and_implied_diameter():
    rows = [target_row(2.1, (2.1, 0.0)), target_row(1.0, (0.0, 9.0))]
    matched, unmatched = am.analyze_targets(rows, TARGETS)
    assert matched["matched"] and matched["target"] == 1
    assert matched["true_dist"] == pytest.approx(2.0)
    assert matched["err"] == pytest.approx(0.1)
    assert matched["world_err"] == pytest.approx(0.1)
    assert matched["implied_diameter"] == pytest.approx(config.TARGET_DIAMETER * 2.0 / 2.1)
    assert unmatched["matched"] is False


def test_classify_raw_by_projected_position():
    rows = [raw_row(2.0, 0.8), raw_row(2.0, 0.0, cx=40.0)]  # 정면 2 m 사과, 왼쪽 끝 다른 물체
    result = am.classify_raw(rows, TARGETS)
    assert [r["is_target"] for r in result] == [True, False]


def test_suggest_ratio_min():
    hit = {"row": {"color_ratio": 0.6}, "is_target": True}
    other = {"row": {"color_ratio": 0.1}, "is_target": False}
    overlap = {"row": {"color_ratio": 0.7}, "is_target": False}
    assert am.suggest_ratio_min([hit, other]) == pytest.approx(0.35)
    assert am.suggest_ratio_min([hit, overlap]) is None
    assert am.suggest_ratio_min([hit]) == pytest.approx(0.3)
    assert am.suggest_ratio_min([other]) is None


def test_report_from_tuner_csv(tmp_path):
    path = str(tmp_path / "m.csv")
    det = {
        "cx": 320.0,
        "cy": 250.0,
        "w": 26.3,
        "h": 26.3,
        "conf": 0.7,
        "cls": 47,
        "color": "red",
        "dist": 2.0,
        "bearing": 0.0,
        "source": "yolo",
    }
    raw = [{"xyxy": (306.85, 236.85, 333.15, 263.15), "conf": 0.7, "cls": 47, "ratio": 0.8}]
    rows = hsv_tuner.measurement_rows(
        1.0, (0.0, 0.0, 0.0), [det], raw, {47: "apple"}, [], 60.0, "auto 2.0m"
    )
    assert hsv_tuner.append_csv(path, rows)
    text = am.report(am.load_rows(path), TARGETS)
    assert "| auto 2.0m | yolo | apple | 1 | 2.00 | 2.00 | +0.00 |" in text
    assert "| 실제 사과 | apple | 1 |" in text
    assert "60.00" in text
