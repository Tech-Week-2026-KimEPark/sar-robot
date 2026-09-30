import math
import sys
import types

import cv2
import numpy as np
import pytest

from sar import config
from sar.perception import (
    Confirm,
    TargetDetector,
    bearing_from_cx,
    color_mask,
    distance_from_size,
    focal_length_px,
    is_excluded,
    to_world,
)

W, H = 640, 480
RED = (0, 0, 255)  # BGR
GREEN = (0, 200, 0)
ORANGE = (0, 165, 255)
BACKGROUND = 90  # 회색 바닥


def blank() -> np.ndarray:
    return np.full((H, W, 3), BACKGROUND, np.uint8)


def draw_ball(image, cx, cy, radius, color):
    cv2.circle(image, (cx, cy), radius, color, -1)
    return image


def test_focal_length_matches_spec():
    assert focal_length_px(W) == pytest.approx(554.3, abs=0.1)


def test_bearing_sign():
    assert bearing_from_cx(W / 2, W) == pytest.approx(0.0)
    assert bearing_from_cx(0, W) == pytest.approx(config.CAMERA_FOV / 2)  # 왼쪽 끝
    assert bearing_from_cx(W, W) == pytest.approx(-config.CAMERA_FOV / 2)


def test_distance_on_axis():
    # 과제 기준 6장: 3 m 거리 사과는 약 17.6 px
    assert distance_from_size(17.6, 0.0, W) == pytest.approx(3.0, rel=0.01)


def test_distance_off_axis_is_longer_than_depth():
    depth = distance_from_size(20.0, 0.0, W)
    assert distance_from_size(20.0, math.radians(30), W) == pytest.approx(
        depth / math.cos(math.radians(30))
    )


def test_distance_zero_size():
    assert distance_from_size(0.0, 0.0, W) is None


def test_red_mask_covers_hue_wraparound():
    image = np.zeros((1, 2, 3), np.uint8)
    image[0, 0] = (0, 0, 255)  # H = 0
    image[0, 1] = cv2.cvtColor(np.uint8([[[175, 255, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
    mask = color_mask(image, config.HSV_RANGES["red"])
    assert mask.tolist() == [[255, 255]]


def test_unknown_color_raises():
    with pytest.raises(ValueError):
        TargetDetector("blue", model_path=None)


def test_color_fallback_detects_red_ball():
    detector = TargetDetector("red", model_path=None)
    det = detector.detect(draw_ball(blank(), 400, 300, 15, RED))
    assert det is not None
    assert det["source"] == "color"
    assert det["color"] == "red"
    assert det["cls"] is None
    assert det["cx"] == pytest.approx(400, abs=1)
    assert det["bearing"] < 0  # 화면 오른쪽
    expected = distance_from_size(det["w"], det["bearing"], W)
    assert det["dist"] == pytest.approx(expected)
    assert det["w"] == pytest.approx(30, abs=2)


def test_other_colors_ignored():
    image = blank()
    draw_ball(image, 200, 300, 15, GREEN)
    draw_ball(image, 400, 300, 15, ORANGE)
    assert TargetDetector("red", model_path=None).detect(image) is None


def test_small_and_elongated_blobs_ignored():
    image = blank()
    draw_ball(image, 100, 300, 3, RED)  # 면적 하한 미만
    cv2.rectangle(image, (300, 300), (500, 305), RED, -1)  # 원형도 낮음
    assert TargetDetector("red", model_path=None).detect(image) is None


def test_above_horizon_ignored():
    # 식탁 위 사과처럼 화면 가운데선보다 확실히 위에 있는 덩어리
    image = draw_ball(blank(), 320, H // 2 - config.HORIZON_MARGIN - 20, 15, RED)
    assert TargetDetector("red", model_path=None).detect(image) is None


def test_none_image():
    detector = TargetDetector("red", model_path=None)
    assert detector.detect(None) is None
    assert detector.detect_all(None) == []


def test_two_apples_sorted_nearest_first():
    image = blank()
    draw_ball(image, 150, 300, 8, RED)  # 멀리
    draw_ball(image, 450, 320, 20, RED)  # 가까이
    detections = TargetDetector("red", model_path=None).detect_all(image)
    assert len(detections) == 2
    assert detections[0]["cx"] == pytest.approx(450, abs=1)
    assert detections[0]["dist"] < detections[1]["dist"]


class _Array:
    def __init__(self, values):
        self._values = np.asarray(values, float)

    def cpu(self):
        return self

    def numpy(self):
        return self._values


class _FakeYolo:
    """ultralytics 결과 형식(results[0].boxes.xyxy/conf/cls)을 흉내 내는 모델."""

    def __init__(self, boxes):
        self.boxes = boxes  # [(x1, y1, x2, y2, conf, cls), ...]

    def predict(self, image, **kwargs):
        self.kwargs = kwargs
        rows = self.boxes or np.zeros((0, 6))
        arr = np.asarray(rows, float).reshape(-1, 6)
        boxes = types.SimpleNamespace(
            xyxy=_Array(arr[:, :4]), conf=_Array(arr[:, 4]), cls=_Array(arr[:, 5])
        )
        return [types.SimpleNamespace(boxes=boxes)]


def detector_with_yolo(boxes) -> TargetDetector:
    detector = TargetDetector("red", model_path=None)
    detector.model = _FakeYolo(boxes)
    return detector


def test_yolo_box_with_target_color_accepted():
    image = draw_ball(blank(), 400, 300, 15, RED)
    det = detector_with_yolo([(385, 285, 415, 315, 0.8, 47)]).detect(image)
    assert det["source"] == "yolo"
    assert det["cls"] == 47
    assert det["conf"] == pytest.approx(0.8)
    assert det["w"] == pytest.approx(30)


def test_yolo_box_with_other_color_rejected_then_fallback():
    image = blank()
    draw_ball(image, 200, 300, 15, GREEN)
    draw_ball(image, 450, 300, 15, RED)
    # YOLO는 초록 사과만 찾음 → 색 판별로 제외 → 색 분할로 빨간 사과 검출
    det = detector_with_yolo([(185, 285, 215, 315, 0.9, 47)]).detect(image)
    assert det["source"] == "color"
    assert det["cx"] == pytest.approx(450, abs=1)


def test_yolo_predict_error_falls_back_to_color():
    class Broken:
        def predict(self, *args, **kwargs):
            raise RuntimeError("boom")

    detector = TargetDetector("red", model_path=None)
    detector.model = Broken()
    det = detector.detect(draw_ball(blank(), 400, 300, 15, RED))
    assert det["source"] == "color"
    assert "boom" in detector.last_error


def test_model_load_failure_recorded(monkeypatch):
    def fail(path):
        raise FileNotFoundError(path)

    monkeypatch.setitem(sys.modules, "ultralytics", types.SimpleNamespace(YOLO=fail))
    detector = TargetDetector("red", model_path="missing.pt")
    assert detector.model is None
    assert "missing.pt" in detector.load_error


def test_to_world():
    det = {"dist": 2.0, "bearing": 0.0}
    assert to_world(det, (1.0, 2.0, math.pi / 2)) == pytest.approx((1.0, 4.0))
    det = {"dist": 1.0, "bearing": math.pi / 2}  # 정면 기준 왼쪽 90°
    assert to_world(det, (0.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0))
    detector = TargetDetector("red", model_path=None)
    assert detector.to_world(det, (0.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0))


def test_is_excluded():
    found = [(-5.34, -10.54)]
    assert is_excluded((-5.0, -10.5), found)
    assert not is_excluded((-12.02, -3.02), found)
    assert not is_excluded((0.0, 0.0), [])


def test_confirm_after_consecutive_frames():
    confirm = Confirm(frames=4)
    for _ in range(3):
        assert confirm.update(True, (1.0, 1.0), 2.0)[0] is False
    ok, xy = confirm.update(True, (1.0, 1.0), 2.0)
    assert ok is True
    assert xy == pytest.approx((1.0, 1.0))


def test_confirm_miss_resets():
    confirm = Confirm(frames=2)
    confirm.update(True, (1.0, 1.0), 2.0)
    assert confirm.update(False, None) == (False, None)
    assert confirm.update(True, (1.0, 1.0), 2.0)[0] is False


def test_confirm_jump_restarts():
    confirm = Confirm(frames=2, match_radius=0.5)
    confirm.update(True, (0.0, 0.0), 1.0)
    ok, xy = confirm.update(True, (3.0, 0.0), 1.0)
    assert ok is False
    assert xy == pytest.approx((3.0, 0.0))


def test_confirm_weighted_mean_prefers_near():
    confirm = Confirm(frames=1, match_radius=1.0, min_dist=0.3)
    confirm.update(True, (0.0, 0.0), 0.5)  # 가중치 4
    _, xy = confirm.update(True, (0.3, 0.0), 1.0)  # 가중치 1
    assert xy == pytest.approx((0.06, 0.0))


def test_confirm_min_dist_clamp():
    confirm = Confirm(frames=1, match_radius=1.0, min_dist=0.3)
    confirm.update(True, (0.0, 0.0), 0.01)  # 0.3 m로 취급, 가중치 1/0.09
    _, xy = confirm.update(True, (0.3, 0.0), 0.3)  # 같은 가중치
    assert xy == pytest.approx((0.15, 0.0))


def test_last_yolo_keeps_rejected_boxes_with_ratio():
    image = blank()
    draw_ball(image, 200, 300, 15, GREEN)
    detector = detector_with_yolo([(185, 285, 215, 315, 0.9, 47)])
    detector.detect_all(image)
    assert len(detector.last_yolo) == 1
    assert detector.last_yolo[0]["ratio"] < config.COLOR_RATIO_MIN
    assert detector.last_yolo_ms is not None
    detector.detect_all(None)
    assert detector.last_yolo == []


def test_yolo_target_not_duplicated_by_color():
    image = draw_ball(blank(), 400, 300, 15, RED)
    detections = detector_with_yolo([(385, 285, 415, 315, 0.8, 47)]).detect_all(image)
    assert [d["source"] for d in detections] == ["yolo"]


def test_color_adds_second_apple_missed_by_yolo():
    # 병합 분석 6.1절 8번: YOLO가 한 사과만 찾으면 두 번째 사과가 누락되던 문제
    image = blank()
    draw_ball(image, 450, 320, 20, RED)  # 가까운 사과, YOLO 검출
    draw_ball(image, 150, 300, 8, RED)  # 먼 사과, YOLO 미검출
    detections = detector_with_yolo([(430, 300, 470, 340, 0.7, 47)]).detect_all(image)
    assert [d["source"] for d in detections] == ["yolo", "color"]
    assert detections[1]["cx"] == pytest.approx(150, abs=1)


def test_yolo_uses_class_agnostic_nms():
    # 실제 프레임에서 한 사과가 apple·sports ball 두 상자로 검출되던 문제
    detector = detector_with_yolo([(385, 285, 415, 315, 0.8, 47)])
    detector.detect_all(draw_ball(blank(), 400, 300, 15, RED))
    assert detector.model.kwargs["agnostic_nms"] is True
    assert detector.model.kwargs["verbose"] is False
