"""HSV 임계값 튜닝과 인식 성능 측정 컨트롤러.

OpenCV 창에서 트랙바로 HSV 범위를 조정한다. H 하한이 상한보다 크면 빨강처럼 0을 넘는
범위로 처리한다. 위치는 시작 pose(config.START_*)와 엔코더 오도메트리로 추정한다.

조종 키는 누르고 있는 동안만 움직인다. Webots 3D 화면과 OpenCV 창 어느 쪽에 포커스가
있어도 입력된다. Webots 화면에서는 키를 누르고 있는 동안 매 step 입력되고, OpenCV 창에서는
키 입력 1회를 config.TUNER_KEY_HOLD 동안 유지한다. 두 키를 함께 누르면 곡선 주행한다.

| 키 | 동작 |
|---|---|
| w / s 또는 ↑ / ↓ | 전진 / 후진 (누르는 동안) |
| a / d 또는 ← / → | 좌회전 / 우회전 (누르는 동안) |
| Shift (Webots 화면), 대문자 (OpenCV 창) | 저속 (config.TUNER_FINE_SCALE배) |
| space | 정지, 자동 주행 취소 |
| m | 현재 프레임 측정값을 CSV(config.TUNER_LOG_FILE)에 추가하고 프레임 저장 |
| e | 자동 탐색: 미션 상태 머신 실행 (나침반 보정 회전 → 프론티어 A* → 접근·구조 → 복귀) |
| g | 자동 접근: 제자리 회전 탐색 → 정렬 → 접근. 3·2·1 m와 정지 지점에서 자동 측정 |
| o | 회전 측정: 제자리 1회전하며 30°마다 자동 측정 |
| p | 원본 프레임 PNG 저장 (config.TUNER_FRAME_DIR) |
| r | 현재 범위를 config.HSV_RANGES 형식으로 출력 |
| q | 종료. 창 닫기도 같음 |

자동 주행·자동 탐색 중 조종 키를 누르면 즉시 수동 조종으로 돌아간다.
자동 탐색 중에는 트랙바 HSV 범위가 미션 검출에 그대로 적용되고, 검출이 있으면
config.TUNER_EXPLORE_LOG_PERIOD 간격으로 측정을 기록한다. 탐색에서 얻은 나침반 보정은
수동 조종으로 돌아온 뒤에도 위치 추정에 계속 사용한다. 지도는 별도 창에 표시한다.
수동·자동 모두 라이다 안전 필터(local_control.safety_filter)로 충돌을 막는다.

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

from sar import (
    config,  # noqa: E402
    viz,  # noqa: E402
)
from sar.grid_map import GridMap  # noqa: E402
from sar.local_control import safety_filter  # noqa: E402
from sar.mission import DONE, Mission, compass_heading  # noqa: E402
from sar.odometry import Odometry  # noqa: E402
from sar.perception import TargetDetector, color_mask, to_world  # noqa: E402
from sar.robot_io import RobotIO  # noqa: E402

WINDOW = "hsv_tuner"
MAP_WINDOW = "hsv_tuner_map"
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
CV2_MAX_EVENTS = 16  # 한 step에 처리하는 OpenCV 키 입력 최대 개수. 밀린 입력을 한 번에 비움

# 키 이름: (v 부호, w 부호)
DRIVE_KEYS = {
    "w": (1, 0),
    "up": (1, 0),
    "s": (-1, 0),
    "down": (-1, 0),
    "a": (0, 1),
    "left": (0, 1),
    "d": (0, -1),
    "right": (0, -1),
}
COMMAND_KEYS = {" ", "m", "e", "g", "o", "p", "r", "q"}
# cv2.waitKeyEx 방향키 코드 (Windows, Linux GTK)
CV2_ARROWS = {
    2490368: "up",
    2621440: "down",
    2424832: "left",
    2555904: "right",
    65362: "up",
    65364: "down",
    65361: "left",
    65363: "right",
}
HUD = "WASD/arrows: drive  Shift: slow  e: explore  g: approach  o: survey  m: measure  q: quit"

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


def cv2_key_name(code: int) -> tuple[str | None, bool]:
    """cv2.waitKeyEx 코드를 (키 이름, 대문자 여부)로 변환. 입력 없음·미지원 키는 None."""
    if code == -1:
        return None, False
    if code in CV2_ARROWS:
        return CV2_ARROWS[code], False
    base = code & 0xFF
    if ord(" ") <= base <= ord("~"):
        char = chr(base)
        return char.lower(), char.isupper()
    return None, False


class KeyHold:
    """OpenCV 창 키 입력을 일정 시간 유지해 누르고 있는 상태로 취급.

    OpenCV는 키를 뗀 시점을 알려주지 않음. 키 반복 입력이 들어오는 동안 유지 시간이 갱신되므로
    누르고 있으면 계속 움직이고, 떼면 hold 초 뒤 멈춤.
    """

    def __init__(self, hold: float = config.TUNER_KEY_HOLD):
        self.hold = hold
        self._until: dict[str, float] = {}
        self._fine: dict[str, bool] = {}

    def press(self, name: str, now: float, fine: bool = False) -> None:
        """조종 키 입력 1회 반영. 반대 방향 키는 즉시 해제."""
        sv, sw = DRIVE_KEYS[name]
        for other, (ov, ow) in DRIVE_KEYS.items():
            if (sv and ov == -sv) or (sw and ow == -sw):
                self._until.pop(other, None)
        self._until[name] = now + self.hold
        self._fine[name] = fine

    def clear(self) -> None:
        """유지 중인 키 전체 해제."""
        self._until.clear()

    def held(self, now: float) -> tuple[set[str], bool]:
        """(유지 중인 키 집합, 저속 여부)."""
        keys = {k for k, until in self._until.items() if until > now}
        return keys, any(self._fine.get(k, False) for k in keys)


def drive_from_keys(keys: set[str], fine: bool) -> tuple[float, float]:
    """누른 조종 키로 속도 명령 (v [m/s], w [rad/s]) 계산. 반대 방향 키는 상쇄."""
    sv = sum(DRIVE_KEYS[k][0] for k in keys if k in DRIVE_KEYS)
    sw = sum(DRIVE_KEYS[k][1] for k in keys if k in DRIVE_KEYS)
    sv, sw = max(-1, min(1, sv)), max(-1, min(1, sw))
    scale = config.TUNER_FINE_SCALE if fine else 1.0
    return sv * config.TUNER_V * scale, sw * config.TUNER_W * scale


def map_view(
    grid: GridMap,
    trajectory: Sequence[Sequence[float]],
    path: Sequence[Sequence[float]] | None,
    rescued: Sequence[Sequence[float]],
    start: Sequence[float],
    pose: Sequence[float],
    title: str,
) -> np.ndarray:
    """지도 창 그림. 긴 변이 config.TUNER_MAP_VIEW_MAX를 넘으면 축소."""
    image = viz.render_map(
        grid.public(),
        grid.to_cell,
        trajectory=trajectory,
        path=path or (),
        rescued=rescued,
        start=start,
        pose=pose,
        title=title,
    )
    scale = config.TUNER_MAP_VIEW_MAX / max(image.shape[:2])
    if scale < 1:
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return image


def save_frame(frame: np.ndarray, t: float) -> str | None:
    """프레임을 PNG로 저장하고 경로 반환. 실패하면 None."""
    os.makedirs(config.TUNER_FRAME_DIR, exist_ok=True)
    path = os.path.join(config.TUNER_FRAME_DIR, f"frame_{t:07.2f}.png")
    return path if cv2.imwrite(path, frame) else None


def main() -> None:
    io = RobotIO()
    odom = Odometry(config.START_X, config.START_Y, config.START_THETA)
    grid = GridMap()
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
    hold = KeyHold()
    prev_webots_keys: set[str] = set()
    auto = None  # AutoApproach 또는 SpinSurvey
    mission: Mission | None = None  # 자동 탐색 중이면 미션 상태 머신
    compass_fit = None  # 자동 탐색 시작 회전에서 얻은 (sign, offset, scale)
    start_xy = odom.pose()[:2]
    trajectory = [start_xy]
    last_count = detector.detect_count
    last_explore_log = last_map_view = -math.inf
    last_log_sim, last_log_wall = io.time(), time.perf_counter()
    while io.step():
        if mission is not None:
            mission.tick()  # 오도메트리·지도·검출·주행을 미션이 수행
            compass_fit = mission.compass_fit or compass_fit
        else:
            vec = io.compass()
            heading = None
            if compass_fit is not None and vec is not None:
                heading = compass_heading(vec, *compass_fit[:2])
            odom.update(*io.encoders(), compass=heading)
            grid.update(odom.pose(), io.lidar())
        pose = odom.pose()
        if math.dist(trajectory[-1], pose[:2]) >= config.TRAJ_STEP:
            trajectory.append(pose[:2])

        frame = io.camera_bgr()
        if frame is None:
            continue
        # 창 닫기 버튼으로 창이 사라지면 트랙바 조회가 cv2.error로 종료되므로 q와 같이 처리
        if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            print("hsv_tuner 창이 닫혀 종료합니다.")
            break
        values = [cv2.getTrackbarPos(name, WINDOW) for name, _ in TRACKBARS]
        detector.ranges = ranges_from_values(values)
        if mission is None and step % config.YOLO_EVERY == 0:
            detector.detect_all(frame)
        step += 1
        fresh = detector.detect_count != last_count  # 이번 step에 검출을 새로 실행했는지
        last_count = detector.detect_count
        detections = detector.last_detections

        now_sim, now_wall = io.time(), time.perf_counter()
        if now_sim - last_log_sim >= config.LOG_INTERVAL:
            wall = now_wall - last_log_wall
            rtf = (now_sim - last_log_sim) / wall if wall > 0 else None
            print(
                summary_line(
                    now_sim, pose, detections, detector.last_yolo, names, detector.last_yolo_ms, rtf
                )
            )
            last_log_sim, last_log_wall = now_sim, now_wall

        if mission is not None:
            has_det = bool(detections or detector.last_yolo)
            if fresh and has_det and now_sim - last_explore_log >= config.TUNER_EXPLORE_LOG_PERIOD:
                record(frame, detections, f"explore {mission.state}")
                last_explore_log = now_sim
            if mission.state == DONE:
                print("[explore] 미션 종료(DONE). 수동 조종으로 전환")
                mission = None
        elif auto is not None:
            cmd = auto.update(now_sim, pose, detections, fresh)
            if cmd.message:
                print(f"[auto] {cmd.message}")
            if cmd.measure:
                record(frame, detections, cmd.measure)
            v, w = cmd.v, cmd.w
            if cmd.done:
                auto = None

        if now_sim - last_map_view >= config.TUNER_MAP_VIEW_PERIOD:
            last_map_view = now_sim
            if mission is not None:
                view = map_view(
                    grid,
                    mission.trajectory,
                    mission.path,
                    mission.rescued,
                    mission.start,
                    pose,
                    f"t={now_sim:.1f}s explore {mission.state}",
                )
            else:
                view = map_view(grid, trajectory, None, (), start_xy, pose, f"t={now_sim:.1f}s")
            cv2.imshow(MAP_WINDOW, view)

        overlay = frame.copy()
        draw_detections(overlay, detections)
        if mission is not None:
            status = f"explore {mission.state}"
        else:
            status = (
                "manual" if auto is None else f"{type(auto).__name__} {getattr(auto, 'state', '')}"
            )
            status += f" v={v:+.2f} w={w:+.2f}"
        cv2.putText(overlay, status, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(overlay, HUD, (8, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        mask = cv2.cvtColor(color_mask(frame, detector.ranges), cv2.COLOR_GRAY2BGR)
        preview = cv2.resize(np.hstack([overlay, mask]), None, fx=PREVIEW_SCALE, fy=PREVIEW_SCALE)
        cv2.imshow(WINDOW, preview)

        # 입력 수집: OpenCV 창 이벤트(밀린 입력 모두)와 Webots 화면의 현재 눌린 키
        commands: list[str] = []
        for _ in range(CV2_MAX_EVENTS):
            name, upper = cv2_key_name(cv2.waitKeyEx(KEY_WAIT_MS))
            if name is None:
                break
            if name in DRIVE_KEYS:
                hold.press(name, now_wall, fine=upper)
            elif name in COMMAND_KEYS:
                commands.append(name)
        webots_keys, webots_shift = io.keys()
        commands += sorted((webots_keys - prev_webots_keys) & COMMAND_KEYS)  # 새로 누른 키만
        prev_webots_keys = webots_keys
        cv2_drive, cv2_fine = hold.held(now_wall)
        drive_keys = cv2_drive | (webots_keys & DRIVE_KEYS.keys())
        fine = webots_shift or cv2_fine

        if drive_keys and (auto is not None or mission is not None):
            print("[auto] 조종 키 입력으로 수동 조종 전환")
            auto = mission = None
        if auto is None and mission is None:
            v, w = drive_from_keys(drive_keys, fine)

        quit_requested = False
        for key in commands:
            if key == " ":
                hold.clear()
                if auto is not None or mission is not None:
                    print("[auto] 자동 주행 취소")
                auto = mission = None
                v, w = 0.0, 0.0
            elif key == "e":
                if mission is not None:
                    print("[explore] 자동 탐색 중지")
                    mission = None
                    v, w = 0.0, 0.0
                else:
                    auto = None
                    mission = Mission(io, odom, grid, detector, log=print)
                    print("[explore] 자동 탐색 시작: 나침반 보정 회전 후 프론티어 탐색")
            elif key in ("g", "o"):
                mission = None
                auto = (AutoApproach if key == "g" else SpinSurvey)(now_sim, pose)
                print("[auto] 자동 접근 시작" if key == "g" else "[auto] 회전 측정 시작")
            elif key == "m":
                detector.detect_all(frame)  # 측정 시점 프레임으로 다시 검출
                last_count = detector.detect_count
                record(frame, detector.last_detections, "manual")
            elif key == "p":
                print(f"저장: {save_frame(frame, io.time())}")
            elif key == "r":
                print(f'"{config.TARGET_COLOR}": {detector.ranges},')
            elif key == "q":
                quit_requested = True
        if quit_requested:
            break

        if mission is None:  # 자동 탐색 중에는 미션이 안전 필터를 거쳐 구동함
            v_safe, w_safe, blocked = safety_filter(v, w, io.lidar())
            if blocked and auto is not None:
                print("[auto] 진행 방향 장애물로 자동 주행을 종료합니다.")
                auto = None
            io.drive(v_safe, w_safe)

    io.drive(0.0, 0.0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
