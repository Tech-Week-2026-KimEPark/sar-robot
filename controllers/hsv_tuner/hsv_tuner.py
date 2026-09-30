"""HSV 임계값 튜닝과 인식 성능 측정 컨트롤러.

OpenCV 창에서 로봇을 조종하며 트랙바로 HSV 범위를 조정한다. 키 입력은 OpenCV 창에
포커스가 있을 때만 동작한다. H 하한이 상한보다 크면 빨강처럼 0을 넘는 범위로 처리한다.
위치는 시작 pose(config.START_*)와 엔코더 오도메트리로 추정한다.

| 키 | 동작 |
|---|---|
| w / s | 전진 / 후진 |
| a / d | 좌회전 / 우회전 |
| space | 정지 |
| m | 현재 프레임 측정값을 CSV(config.TUNER_LOG_FILE)에 추가하고 프레임 저장 |
| g | 자동 접근: 제자리 회전 탐색 → 정렬 → 접근. 3·2·1 m와 정지 지점에서 자동 측정 |
| o | 회전 측정: 제자리 1회전하며 30°마다 자동 측정 |
| p | 원본 프레임 PNG 저장 (config.TUNER_FRAME_DIR) |
| r | 현재 범위를 config.HSV_RANGES 형식으로 출력 |
| q | 종료. 창 닫기도 같음 |

자동 주행 중 조종 키(w, s, a, d, space)를 누르면 즉시 수동 조종으로 돌아간다.
수동·자동 모두 라이다 안전 필터(local_control.safety_filter)로 정면 충돌을 막는다.

콘솔에는 config.LOG_INTERVAL 주기로 pose, YOLO 추론 시간, 실시간 배율, 검출 요약을 출력한다.
"""

import csv
import math
import os
import sys
import time
from collections.abc import Sequence

import cv2
import numpy as np

# 팀 패키지 sar는 controllers/sar_main/에 있음. Webots는 이 컨트롤러 폴더만 import 경로에 포함
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sar_main"))

from auto_drive import AutoApproach, SpinSurvey  # noqa: E402

from sar import config  # noqa: E402
from sar.local_control import safety_filter  # noqa: E402
from sar.odometry import Odometry  # noqa: E402
from sar.perception import TargetDetector, color_mask, to_world  # noqa: E402
from sar.robot_io import RobotIO  # noqa: E402

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

CSV_FIELDS = [
    "sim_t",
    "note",
    "robot_x",
    "robot_y",
    "robot_theta_deg",
    "kind",
    "source",
    "cls",
    "cls_name",
    "conf",
    "color_ratio",
    "cx",
    "cy",
    "w",
    "h",
    "dist",
    "bearing_deg",
    "world_x",
    "world_y",
    "yolo_ms",
    "hsv_ranges",
]
_DIGITS = 3  # CSV 소수점 자리수


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


def _r(value: float | None) -> float | str:
    return "" if value is None else round(float(value), _DIGITS)


def _cls_name(cls: int | None, names: dict) -> str:
    return "" if cls is None else str(names.get(cls, cls))


def measurement_rows(
    t: float,
    pose: Sequence[float],
    detections: list[dict],
    yolo_boxes: list[dict],
    names: dict,
    ranges: list,
    yolo_ms: float | None = None,
    note: str = "",
) -> list[dict]:
    """측정 1회의 CSV 행 목록.

    kind는 target(최종 대상 검출), yolo_raw(색 판별 전 YOLO 상자), none(검출 없음).
    """
    x, y, theta = pose
    base = {
        "sim_t": _r(t),
        "note": note,
        "robot_x": _r(x),
        "robot_y": _r(y),
        "robot_theta_deg": _r(math.degrees(theta)),
        "yolo_ms": _r(yolo_ms),
        "hsv_ranges": str(ranges),
    }
    rows = []
    for det in detections:
        wx, wy = to_world(det, pose)
        rows.append(
            {
                **base,
                "kind": "target",
                "source": det["source"],
                "cls": "" if det["cls"] is None else det["cls"],
                "cls_name": _cls_name(det["cls"], names),
                "conf": _r(det["conf"]),
                "cx": _r(det["cx"]),
                "cy": _r(det["cy"]),
                "w": _r(det["w"]),
                "h": _r(det["h"]),
                "dist": _r(det["dist"]),
                "bearing_deg": _r(math.degrees(det["bearing"])),
                "world_x": _r(wx),
                "world_y": _r(wy),
            }
        )
    for box in yolo_boxes:
        x1, y1, x2, y2 = box["xyxy"]
        rows.append(
            {
                **base,
                "kind": "yolo_raw",
                "source": "yolo",
                "cls": box["cls"],
                "cls_name": _cls_name(box["cls"], names),
                "conf": _r(box["conf"]),
                "color_ratio": _r(box.get("ratio")),
                "cx": _r((x1 + x2) / 2),
                "cy": _r((y1 + y2) / 2),
                "w": _r(x2 - x1),
                "h": _r(y2 - y1),
            }
        )
    if not rows:
        rows.append({**base, "kind": "none"})
    return rows


def append_csv(path: str, rows: list[dict]) -> bool:
    """CSV 파일에 행 추가. 새 파일이면 머리행 작성. 실패하면 False."""
    try:
        folder = os.path.dirname(path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        new_file = not os.path.exists(path)
        # Excel에서 한글이 깨지지 않도록 BOM 포함 UTF-8 사용
        with open(path, "a", newline="", encoding="utf-8-sig" if new_file else "utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, restval="")
            if new_file:
                writer.writeheader()
            writer.writerows(rows)
        return True
    except OSError:
        return False


def summary_line(
    t: float,
    pose: Sequence[float],
    detections: list[dict],
    yolo_boxes: list[dict],
    names: dict,
    yolo_ms: float | None,
    rtf: float | None,
) -> str:
    """주기 로그 1줄. 가장 가까운 대상과 YOLO 원본 상자 요약."""
    x, y, theta = pose
    parts = [f"[t={t:.1f}s] pose=({x:.2f}, {y:.2f}, {math.degrees(theta):.0f}deg)"]
    parts.append("yolo=-" if yolo_ms is None else f"yolo={yolo_ms:.0f}ms")
    parts.append("rtf=-" if rtf is None else f"rtf={rtf:.2f}")
    if detections:
        det = detections[0]
        wx, wy = to_world(det, pose)
        label = det["source"] if det["cls"] is None else _cls_name(det["cls"], names)
        parts.append(
            f"target={label} conf={det['conf']:.2f} d={det['dist']:.2f}m "
            f"b={math.degrees(det['bearing']):.0f}deg at ({wx:.2f}, {wy:.2f})"
        )
    else:
        parts.append("target=none")
    if yolo_boxes:
        raw = ", ".join(
            f"{_cls_name(b['cls'], names)} {b['conf']:.2f} r={b.get('ratio', 0):.2f}"
            for b in yolo_boxes
        )
        parts.append(f"raw=[{raw}]")
    return " ".join(parts)


def draw_detections(image: np.ndarray, detections: list[dict]) -> None:
    for det in detections:
        x1, y1 = int(det["cx"] - det["w"] / 2), int(det["cy"] - det["h"] / 2)
        x2, y2 = int(det["cx"] + det["w"] / 2), int(det["cy"] + det["h"] / 2)
        color = (0, 255, 0) if det["source"] == "yolo" else (255, 255, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        label = f"{det['source']} {det['dist']:.2f}m {np.degrees(det['bearing']):.0f}deg"
        cv2.putText(image, label, (x1, max(y1 - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def save_frame(frame: np.ndarray, t: float) -> str | None:
    """프레임을 PNG로 저장하고 경로 반환. 실패하면 None."""
    os.makedirs(config.TUNER_FRAME_DIR, exist_ok=True)
    path = os.path.join(config.TUNER_FRAME_DIR, f"frame_{t:07.2f}.png")
    return path if cv2.imwrite(path, frame) else None


def main() -> None:
    io = RobotIO()
    odom = Odometry(config.START_X, config.START_Y, config.START_THETA)
    detector = TargetDetector(config.TARGET_COLOR, config.YOLO_MODEL)
    names = dict(detector.model.names) if detector.model is not None else {}
    if detector.load_error:
        print(f"YOLO 로드 실패, 색 분할만 사용: {detector.load_error}")

    cv2.namedWindow(WINDOW)
    for (name, maximum), value in zip(
        TRACKBARS, initial_values(config.HSV_RANGES[config.TARGET_COLOR]), strict=True
    ):
        cv2.createTrackbar(name, WINDOW, value, maximum, lambda _: None)

    def record(frame: np.ndarray, detections: list[dict], note: str) -> None:
        rows = measurement_rows(
            io.time(),
            odom.pose(),
            detections,
            detector.last_yolo,
            names,
            detector.ranges,
            detector.last_yolo_ms,
            note,
        )
        saved = append_csv(config.TUNER_LOG_FILE, rows)
        frame_path = save_frame(frame, io.time())
        print(f"측정 [{note}] {len(rows)}행 기록 {saved}, 프레임 {frame_path}")

    v, w = 0.0, 0.0
    step = 0
    auto = None  # AutoApproach 또는 SpinSurvey. None이면 수동 조종
    detections: list[dict] = []
    last_log_sim, last_log_wall = io.time(), time.perf_counter()
    while io.step():
        odom.update(*io.encoders())
        frame = io.camera_bgr()
        if frame is None:
            continue
        # 창 닫기 버튼으로 창이 사라지면 트랙바 조회가 cv2.error로 종료되므로 q와 같이 처리
        if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            print("hsv_tuner 창이 닫혀 종료합니다.")
            break
        values = [cv2.getTrackbarPos(name, WINDOW) for name, _ in TRACKBARS]
        detector.ranges = ranges_from_values(values)
        fresh = step % config.YOLO_EVERY == 0
        if fresh:
            detections = detector.detect_all(frame)
        step += 1

        now_sim, now_wall = io.time(), time.perf_counter()
        if now_sim - last_log_sim >= config.LOG_INTERVAL:
            wall = now_wall - last_log_wall
            rtf = (now_sim - last_log_sim) / wall if wall > 0 else None
            print(
                summary_line(
                    now_sim,
                    odom.pose(),
                    detections,
                    detector.last_yolo,
                    names,
                    detector.last_yolo_ms,
                    rtf,
                )
            )
            last_log_sim, last_log_wall = now_sim, now_wall

        if auto is not None:
            cmd = auto.update(now_sim, odom.pose(), detections, fresh)
            if cmd.message:
                print(f"[auto] {cmd.message}")
            if cmd.measure:
                record(frame, detections, cmd.measure)
            v, w = cmd.v, cmd.w
            if cmd.done:
                auto = None

        overlay = frame.copy()
        draw_detections(overlay, detections)
        mode = "manual" if auto is None else type(auto).__name__
        state = getattr(auto, "state", "")
        cv2.putText(
            overlay, f"{mode} {state}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
        )
        mask = cv2.cvtColor(color_mask(frame, detector.ranges), cv2.COLOR_GRAY2BGR)
        preview = cv2.resize(np.hstack([overlay, mask]), None, fx=PREVIEW_SCALE, fy=PREVIEW_SCALE)
        cv2.imshow(WINDOW, preview)

        key = cv2.waitKey(KEY_WAIT_MS) & 0xFF
        if key in DRIVE_KEYS:
            if auto is not None:
                print("[auto] 조종 키 입력으로 수동 조종 전환")
                auto = None
            sv, sw = DRIVE_KEYS[key]
            v, w = sv * config.TUNER_V, sw * config.TUNER_W
        elif key == ord("g"):
            auto = AutoApproach(now_sim, odom.pose())
            print("[auto] 자동 접근 시작")
        elif key == ord("o"):
            auto = SpinSurvey(now_sim, odom.pose())
            print("[auto] 회전 측정 시작")
        elif key == ord("m"):
            detections = detector.detect_all(frame)  # 측정 시점 프레임으로 다시 검출
            record(frame, detections, "manual")
        elif key == ord("p"):
            print(f"저장: {save_frame(frame, io.time())}")
        elif key == ord("r"):
            print(f'"{config.TARGET_COLOR}": {detector.ranges},')
        elif key == ord("q"):
            break

        v_safe, w_safe, blocked = safety_filter(v, w, io.lidar())
        if blocked and v > 0:
            if auto is not None:
                print("[auto] 정면 장애물로 자동 주행을 종료합니다.")
                auto = None
            v, w = 0.0, 0.0
        io.drive(v_safe, w_safe)

    io.drive(0.0, 0.0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
