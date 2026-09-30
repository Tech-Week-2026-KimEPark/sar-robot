"""HSV 임계값 튜닝 컨트롤러.

OpenCV 창에서 로봇을 조종하며 트랙바로 HSV 범위를 조정한다. 키 입력은 OpenCV 창에
포커스가 있을 때만 동작한다. H 하한이 상한보다 크면 빨강처럼 0을 넘는 범위로 처리한다.

| 키 | 동작 |
|---|---|
| w / s | 전진 / 후진 |
| a / d | 좌회전 / 우회전 |
| space | 정지 |
| p | 원본 프레임 PNG 저장 (config.TUNER_FRAME_DIR) |
| r | 현재 범위를 config.HSV_RANGES 형식으로 출력 |
| q | 종료 |
"""

import os

import cv2
import numpy as np

from sar import config
from sar.perception import TargetDetector, color_mask
from sar.robot_io import RobotIO

WINDOW = "hsv_tuner"
TRACKBARS = [
    ("H lo", 179),
    ("H hi", 179),
    ("S lo", 255),
    ("S hi", 255),
    ("V lo", 255),
    ("V hi", 255),
]
PREVIEW_SCALE = 0.5  # 원본·마스크를 나란히 표시할 때 축소 비율
KEY_WAIT_MS = 1

# 키: (v 부호, w 부호)
DRIVE_KEYS = {
    ord("w"): (1, 0),
    ord("s"): (-1, 0),
    ord("a"): (0, 1),
    ord("d"): (0, -1),
    ord(" "): (0, 0),
}


def initial_values(ranges: list) -> list[int]:
    """config 범위를 트랙바 초기값 [H lo, H hi, S lo, S hi, V lo, V hi]으로 변환."""
    lo = [min(r[0][i] for r in ranges) for i in range(3)]
    hi = [max(r[1][i] for r in ranges) for i in range(3)]
    if len(ranges) == 2:  # 빨강: (0~a) ∪ (b~179) → H lo = b, H hi = a
        lo[0], hi[0] = max(r[0][0] for r in ranges), min(r[1][0] for r in ranges)
    return [lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]]


def ranges_from_values(values: list[int]) -> list:
    """트랙바 값을 HSV 범위 목록으로 변환. H lo > H hi면 두 범위로 분할."""
    h_lo, h_hi, s_lo, s_hi, v_lo, v_hi = values
    if h_lo <= h_hi:
        return [((h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi))]
    return [((0, s_lo, v_lo), (h_hi, s_hi, v_hi)), ((h_lo, s_lo, v_lo), (179, s_hi, v_hi))]


def draw_detections(image: np.ndarray, detections: list[dict]) -> None:
    for det in detections:
        x1, y1 = int(det["cx"] - det["w"] / 2), int(det["cy"] - det["h"] / 2)
        x2, y2 = int(det["cx"] + det["w"] / 2), int(det["cy"] + det["h"] / 2)
        color = (0, 255, 0) if det["source"] == "yolo" else (255, 255, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        label = f"{det['source']} {det['dist']:.2f}m {np.degrees(det['bearing']):.0f}deg"
        cv2.putText(image, label, (x1, max(y1 - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def main() -> None:
    io = RobotIO()
    detector = TargetDetector(config.TARGET_COLOR, config.YOLO_MODEL)
    if detector.load_error:
        print(f"YOLO 로드 실패, 색 분할만 사용: {detector.load_error}")

    cv2.namedWindow(WINDOW)
    for (name, maximum), value in zip(
        TRACKBARS, initial_values(config.HSV_RANGES[config.TARGET_COLOR]), strict=True
    ):
        cv2.createTrackbar(name, WINDOW, value, maximum, lambda _: None)

    v, w = 0.0, 0.0
    step = 0
    detections: list[dict] = []
    while io.step():
        frame = io.camera_bgr()
        values = [cv2.getTrackbarPos(name, WINDOW) for name, _ in TRACKBARS]
        detector.ranges = ranges_from_values(values)
        if step % config.YOLO_EVERY == 0:
            detections = detector.detect_all(frame)
        step += 1

        overlay = frame.copy()
        draw_detections(overlay, detections)
        mask = cv2.cvtColor(color_mask(frame, detector.ranges), cv2.COLOR_GRAY2BGR)
        preview = cv2.resize(np.hstack([overlay, mask]), None, fx=PREVIEW_SCALE, fy=PREVIEW_SCALE)
        cv2.imshow(WINDOW, preview)

        key = cv2.waitKey(KEY_WAIT_MS) & 0xFF
        if key in DRIVE_KEYS:
            sv, sw = DRIVE_KEYS[key]
            v, w = sv * config.TUNER_V, sw * config.TUNER_W
        elif key == ord("p"):
            os.makedirs(config.TUNER_FRAME_DIR, exist_ok=True)
            path = os.path.join(config.TUNER_FRAME_DIR, f"frame_{io.time():07.2f}.png")
            print(f"저장: {path} {cv2.imwrite(path, frame)}")
        elif key == ord("r"):
            print(f'"{config.TARGET_COLOR}": {detector.ranges},')
        elif key == ord("q"):
            break

        half_track = w * config.WHEEL_SEPARATION / 2
        io.set_wheel_speeds(v - half_track, v + half_track)

    io.set_wheel_speeds(0.0, 0.0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
