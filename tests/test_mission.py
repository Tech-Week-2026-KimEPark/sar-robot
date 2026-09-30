import math

import pytest

from sar import config, perception
from sar.grid_map import GridMap
from sar.mission import (
    APPROACH,
    DONE,
    EXPLORE,
    INIT_SPIN,
    RECOVERY,
    RETURN,
    Mission,
    compass_heading,
    fit_compass,
)
from sar.odometry import Odometry, wrap
from sar.robot_io import wheel_speeds

DT = 0.064  # s, basicTimeStep


class FakeIO:
    """RobotIO 대체. 속도 명령대로 차동 구동 로봇을 적분. 벽 없는 공간(라이다 inf)."""

    def __init__(self, move: bool = True, compass: bool = True):
        self.x = self.y = self.th = 0.0
        self.t = 0.0
        self.enc = [0.0, 0.0]
        self.move = move
        self.compass_on = compass

    def time(self) -> float:
        return self.t

    def encoders(self) -> tuple[float, float]:
        return self.enc[0], self.enc[1]

    def lidar(self) -> list[float]:
        return [math.inf] * 360

    def compass(self):
        # Webots 북쪽 +y 규약: 로봇 좌표계의 북쪽 벡터
        return (math.sin(self.th), math.cos(self.th), 0.0) if self.compass_on else None

    def camera_bgr(self):
        return "frame"

    def drive(self, v: float, w: float) -> None:
        wl, wr = wheel_speeds(v, w) if self.move else (0.0, 0.0)
        self.enc[0] += wl * DT
        self.enc[1] += wr * DT
        dl, dr = wl * config.WHEEL_RADIUS * DT, wr * config.WHEEL_RADIUS * DT
        ds, dth = (dr + dl) / 2, (dr - dl) / config.WHEEL_SEPARATION
        if abs(dth) < 1e-9:
            fx, fy = ds, 0.0
        else:
            fx, fy = ds / dth * math.sin(dth), ds / dth * (1 - math.cos(dth))
        c, s = math.cos(self.th), math.sin(self.th)
        self.x += c * fx - s * fy
        self.y += s * fx + c * fy
        self.th = wrap(self.th + dth)
        self.t += DT


class FakeDetector:
    """실제 로봇 위치와 사과 위치로 검출 결과를 만드는 TargetDetector 대체."""

    def __init__(self, io: FakeIO, apples, fov: float = config.CAMERA_FOV, max_range=3.0):
        self.io, self.apples, self.fov, self.max_range = io, apples, fov, max_range
        self.calls = 0

    def detect_all(self, bgr):
        self.calls += 1
        out = []
        for ax, ay in self.apples:
            d = math.hypot(ax - self.io.x, ay - self.io.y)
            b = wrap(math.atan2(ay - self.io.y, ax - self.io.x) - self.io.th)
            if d <= self.max_range and abs(b) <= self.fov / 2:
                out.append({"dist": d, "bearing": b})
        return sorted(out, key=lambda det: det["dist"])

    to_world = staticmethod(perception.to_world)


@pytest.fixture(autouse=True)
def map_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(config, "MAP_SAVE_DIR", str(tmp_path))
    return tmp_path


def make_mission(apples=(), move=True, fov=config.CAMERA_FOV):
    io = FakeIO(move=move)
    detector = FakeDetector(io, list(apples), fov=fov)
    mission = Mission(io, Odometry(0.0, 0.0, 0.0), GridMap(0.0, 0.0, size=12.0), detector)
    return mission, io, detector


def test_fit_compass_webots_convention_with_odometry_scale_error():
    # 북쪽 +y 규약: 원시각 = π/2 - 방향. 오도메트리 회전량이 12% 크게 측정되어도
    # 첫 표본(시작 방향) 기준 offset은 정확함
    samples = [(wrap(k * 0.3 * 1.12), wrap(math.pi / 2 - k * 0.3)) for k in range(22)]
    sign, offset, scale = fit_compass(samples)
    assert sign == -1
    assert wrap(offset - math.pi / 2) == pytest.approx(0.0, abs=1e-9)
    assert scale == pytest.approx(1.12)


def test_fit_compass_positive_sign_with_offset():
    start = math.pi  # 시작 방향 서쪽
    samples = [(wrap(start + k * 0.3), wrap(k * 0.3 - 0.3)) for k in range(22)]
    sign, offset, _ = fit_compass(samples)
    assert sign == 1
    assert compass_heading((math.cos(-0.3), math.sin(-0.3), 0.0), sign, offset) == pytest.approx(
        start
    )


def test_fit_compass_rejects_small_turn_and_scale_mismatch():
    assert fit_compass([(0.0, 0.0), (0.1, 0.1)]) is None
    assert fit_compass([]) is None
    wrong_scale = [(wrap(k * 0.9), wrap(k * 0.3)) for k in range(22)]  # 비율 3
    assert fit_compass(wrong_scale) is None


def test_confirm_counts_only_inference_steps():
    # 분석 문서 6장 5번: 추론하지 않은 step에 seen=False를 넘기면 확정되지 않음
    mission, _, detector = make_mission(apples=[(1.0, 0.0)], move=False, fov=2 * math.pi)
    inference_steps = config.CONFIRM_FRAMES * config.YOLO_EVERY - (config.YOLO_EVERY - 1)
    for _ in range(inference_steps - 1):
        mission.tick()
    assert mission.target is None
    mission.tick()
    assert detector.calls == config.CONFIRM_FRAMES
    assert mission.target == pytest.approx((1.0, 0.0))


def test_rescued_apple_is_ignored():
    mission, _, _ = make_mission(apples=[(1.1, 0.0)], move=False, fov=2 * math.pi)
    mission.rescued.append((1.0, 0.0, 0.0))
    for _ in range(40):
        mission.tick()
    assert mission.target is None
    assert mission.candidates == []


def test_time_reserve_forces_return():
    mission, io, _ = make_mission()
    logs = []
    mission.log = logs.append
    io.t = config.TIME_LIMIT - config.RETURN_RESERVE
    mission.tick()
    # 시작점에 있으므로 같은 step에 RETURN을 거쳐 DONE까지 전환
    assert any("INIT_SPIN -> RETURN (시간 예비)" in line for line in logs)
    assert mission.state in (RETURN, DONE)


def test_stuck_triggers_recovery():
    mission, _, _ = make_mission(move=False)
    mission.state = EXPLORE
    for _ in range(int(config.STUCK_TIME / DT) + 2):
        mission.tick()
        if mission.state == RECOVERY:
            break
    assert mission.state == RECOVERY


def test_full_mission_rescues_two_apples_and_returns(map_dir):
    apples = [(1.5, 0.4), (-1.2, 1.0)]
    mission, io, _ = make_mission(apples=apples)
    visited = set()
    for _ in range(int(400 / DT)):
        mission.tick()
        visited.add(mission.state)
        if mission.state == DONE:
            break
    assert mission.state == DONE
    assert {INIT_SPIN, APPROACH, RETURN} <= visited
    assert mission.compass_fit is not None and mission.compass_fit[0] == -1
    assert len(mission.rescued) == 2
    for ax, ay in apples:
        assert min(math.dist((ax, ay), (rx, ry)) for rx, ry, _ in mission.rescued) < 0.1
    assert math.hypot(io.x, io.y) <= config.RETURN_TOL
    assert (map_dir / "map_final.png").exists()
    assert (map_dir / "map_rescue_2.png").exists()
