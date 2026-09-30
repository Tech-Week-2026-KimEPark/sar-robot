"""설계 근거 측정 컨트롤러. sar_main과 같은 Mission을 실행하고 센서·실제 pose·계산 시간을 기록.

측정 월드 worlds/sar_apartment_eval.wbt 전용 (로봇 supervisor TRUE).
팀 코드(sar/)는 수정하지 않는다.
controller.Robot을 Supervisor로 교체한 뒤 RobotIO를 생성해 실제 pose를 읽는다.

SAR_EVAL_MODE
- mission (기본): 설정 변경 없음. 실제 pose는 기록에만 사용. 완주 지표 측정
- explore: mission + TARGET_COUNT = 99. 프론티어가 없어질 때까지 탐색
- truth: 오도메트리 대신 실제 pose를 미션에 입력. 위치 추정 오차를 제거한 비교 조건
- truth_explore: truth + TARGET_COUNT = 99 (기준 지도·인식 프레임 수집)

SAR_EVAL_SET (선택): 설정값 덮어쓰기. 예: "CONFIRM_FRAMES=3,LOOKAHEAD=0.2"
sar 모듈 import 전에 적용하므로 함수 기본 인자로 쓰인 설정값에도 반영된다.

출력 (컨트롤러 폴더 기준 output/<mode>[__<SAR_EVAL_SET>]/)
- run.npz: step별 시간, 엔코더, 나침반, 라이다, 추정·실제 pose, 상태, 속도 명령, 계산 시간
- frames/<step>.jpg: 인식을 실행한 step의 카메라 화면 (JPG 품질 EVAL_JPG_QUALITY)
- detections.csv: 인식 step별 현재 검출 결과 (원본 화면 기준)
- log.txt: 미션 로그
"""

import csv
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sar_main"))

import controller  # noqa: E402

controller.Robot = controller.Supervisor  # RobotIO.__init__의 Robot()을 Supervisor로 생성

from sar import config, planner  # noqa: E402

OVERRIDES = [kv.split("=") for kv in filter(None, os.environ.get("SAR_EVAL_SET", "").split(","))]
for _key, _value in OVERRIDES:
    _old = getattr(config, _key)  # 없는 이름이면 AttributeError
    setattr(config, _key, _value == "True" if isinstance(_old, bool) else type(_old)(_value))

from sar.grid_map import GridMap  # noqa: E402
from sar.mission import DONE, Mission  # noqa: E402
from sar.odometry import Odometry  # noqa: E402
from sar.perception import TargetDetector  # noqa: E402
from sar.robot_io import RobotIO  # noqa: E402

EVAL_JPG_QUALITY = 95
EVAL_CONTACT_Z = 0.02  # m, 이보다 높은 접촉점은 몸체 충돌 (바퀴·바닥 접촉 제외)
EVAL_DONE_HOLD = 1.0  # s, DONE 후 기록을 계속하는 시간
EVAL_TIME_MARGIN = 60.0  # s, TIME_LIMIT 초과 후 강제 종료까지 여유
DET_KEYS = ["cx", "cy", "w", "h", "dist", "bearing"]
STATES = ["INIT_SPIN", "EXPLORE", "APPROACH", "RESCUE", "RETURN", "RECOVERY", "DONE"]


def _timed(fn, name: str, calls: list):
    """planner 함수 호출 시간 [ms]과 결과 경로 길이를 calls에 기록하는 래퍼."""

    def wrapper(*args, **kwargs):
        start = time.perf_counter()
        result = fn(*args, **kwargs)
        calls.append((name, (time.perf_counter() - start) * 1000, result is not None))
        return result

    return wrapper


def true_pose(node) -> tuple[float, float, float]:
    """Supervisor 노드의 실제 (x, y, theta)."""
    pos, rot = node.getPosition(), node.getOrientation()
    return pos[0], pos[1], math.atan2(rot[3], rot[0])


class TruthOdometry(Odometry):
    """위치 추정 오차를 제거한 비교 조건. update()가 엔코더 대신 실제 pose를 사용."""

    def __init__(self, node) -> None:
        super().__init__(config.START_X, config.START_Y, config.START_THETA)
        self.node = node

    def update(self, enc_l: float, enc_r: float, compass: float | None = None) -> None:
        self.x, self.y, self.theta = true_pose(self.node)


def main() -> None:
    mode = os.environ.get("SAR_EVAL_MODE", "mission")
    if mode not in ("mission", "explore", "truth", "truth_explore"):
        raise ValueError(f"SAR_EVAL_MODE 값 오류: {mode}")
    if mode.endswith("explore"):
        config.TARGET_COUNT = 99
    suffix = "__" + "_".join(f"{k}-{v}" for k, v in OVERRIDES) if OVERRIDES else ""
    out = os.path.join("output", mode + suffix)
    os.makedirs(os.path.join(out, "frames"), exist_ok=True)
    config.MAP_SAVE_DIR = out
    log_file = open(os.path.join(out, "log.txt"), "w", encoding="utf-8")  # noqa: SIM115

    def log(msg: str) -> None:
        print(msg)
        log_file.write(msg + "\n")

    plan_calls: list = []
    planner.plan = _timed(planner.plan, "plan", plan_calls)
    planner.choose_frontier = _timed(planner.choose_frontier, "choose_frontier", plan_calls)

    io = RobotIO()
    sup = io.robot
    node = sup.getSelf()
    node.enableContactPointsTracking(io.timestep, True)
    command = [0.0, 0.0]
    drive = io.drive

    def recorded_drive(v: float, w: float) -> None:
        command[0], command[1] = v, w
        drive(v, w)

    io.drive = recorded_drive

    if mode.startswith("truth"):
        odom = TruthOdometry(node)
    else:
        odom = Odometry(config.START_X, config.START_Y, config.START_THETA)
    detector = TargetDetector(config.TARGET_COLOR, config.YOLO_MODEL)
    if detector.load_error:
        log(f"YOLO 로드 실패, 색 분할만 사용: {detector.load_error}")
    mission = Mission(io, odom, GridMap(), detector, log=log)
    log(f"mode={mode} TARGET_COUNT={config.TARGET_COUNT} TIME_LIMIT={config.TIME_LIMIT}")
    for key, _ in OVERRIDES:
        log(f"설정 변경 {key}={getattr(config, key)}")

    rows: dict[str, list] = {k: [] for k in ("t", "enc", "compass", "lidar", "gt", "est")}
    rows.update(
        {k: [] for k in ("state", "cmd", "tick_ms", "yolo_ms", "contact", "plan_ms", "wall")}
    )
    det_file = open(os.path.join(out, "detections.csv"), "w", newline="", encoding="utf-8")  # noqa: SIM115
    det_csv = csv.writer(det_file)
    det_csv.writerow(["step", "t", "source", "cls", "conf"] + DET_KEYS)
    step = 0
    done_t = None
    try:
        while io.step():
            t = io.time()
            gt = true_pose(node)
            enc = io.encoders()
            vec = io.compass()
            ranges = io.lidar()
            count = detector.detect_count
            n_calls = len(plan_calls)
            start = time.perf_counter()
            mission.tick()
            tick_ms = (time.perf_counter() - start) * 1000
            inferred = detector.detect_count != count
            contacts = [c.point for c in node.getContactPoints(True) if c.point[2] > EVAL_CONTACT_Z]

            rows["t"].append(t)
            rows["wall"].append(time.perf_counter())
            rows["enc"].append(enc)
            rows["compass"].append(vec if vec is not None else (math.nan,) * 3)
            rows["lidar"].append(ranges if ranges else [math.nan] * 360)
            rows["gt"].append(gt)
            rows["est"].append(odom.pose())
            rows["state"].append(STATES.index(mission.state))
            rows["cmd"].append(tuple(command))
            rows["tick_ms"].append(tick_ms)
            yolo_ms = detector.last_yolo_ms if inferred and detector.model else None
            rows["yolo_ms"].append(math.nan if yolo_ms is None else yolo_ms)
            rows["contact"].append(len(contacts))
            rows["plan_ms"].append(sum(c[1] for c in plan_calls[n_calls:]))
            if inferred:
                bgr = io.camera_bgr()
                if bgr is not None:
                    cv2.imwrite(
                        os.path.join(out, "frames", f"{step:06d}.jpg"),
                        bgr,
                        [cv2.IMWRITE_JPEG_QUALITY, EVAL_JPG_QUALITY],
                    )
                for d in detector.last_detections:
                    det_csv.writerow(
                        [step, f"{t:.3f}", d["source"], d["cls"], d["conf"]]
                        + [f"{d[k]:.4f}" for k in DET_KEYS]
                    )
            if contacts:
                log(f"[t={t:.1f}s] 몸체 접촉 {len(contacts)}점 z={max(p[2] for p in contacts):.3f}")
            step += 1

            if mission.state == DONE and done_t is None:
                done_t = t
            if (done_t is not None and t - done_t >= EVAL_DONE_HOLD) or (
                t > config.TIME_LIMIT + EVAL_TIME_MARGIN
            ):
                break
    finally:
        np.savez_compressed(
            os.path.join(out, "run.npz"),
            **{
                k: np.asarray(v, dtype=np.float32 if k == "lidar" else float)
                for k, v in rows.items()
            },
            rescued=np.asarray(mission.rescued, dtype=np.float64).reshape(-1, 3),
            rejected=np.asarray(mission.rejected, dtype=np.float64).reshape(-1, 2),
            compass_fit=np.asarray(mission.compass_fit or (math.nan,) * 3, dtype=np.float64),
            plan_calls=np.asarray([c[1:] for c in plan_calls], dtype=np.float64).reshape(-1, 2),
            timestep=io.timestep,
        )
        log(f"기록 저장: {step} step, 인식 {detector.detect_count}회, 계획 {len(plan_calls)}회")
        det_file.close()
        log_file.close()
    sup.simulationQuit(0)


if __name__ == "__main__":
    main()
