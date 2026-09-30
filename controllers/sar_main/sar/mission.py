"""미션 상태 머신과 모듈 통합 (CONTEXT.md 7장).

매 step 순서: 센서 → 오도메트리 → 지도 → 인식(YOLO_EVERY step마다) → 상태 머신 → 안전 필터 → drive

| 상태 | 동작 |
|---|---|
| INIT_SPIN | 제자리 1회전. 지도 채우기, 나침반 부호·오프셋 보정 |
| EXPLORE | 미구조 후보 또는 프론티어로 이동. 대상 확정 시 APPROACH |
| APPROACH | 대상 앞 APPROACH_DIST 지점으로 이동 후 정면 정렬 |
| RESCUE | RESCUE_HOLD 정지, 위치·시각 기록, 지도 저장 |
| RETURN | 시작점으로 이동 (모르는 칸 통과 허용) |
| RECOVERY | 후진 후 넓은 쪽으로 회전, 이전 상태로 복귀 |
| DONE | 정지, 최종 지도 저장 |

Webots API는 호출하지 않는다. 장치 접근은 주입받은 io(RobotIO 인터페이스)로만 수행한다.
"""

import math
import os
from collections.abc import Callable, Sequence

from sar import config, planner, viz
from sar.local_control import pure_pursuit, safety_filter
from sar.odometry import wrap
from sar.perception import Confirm, is_excluded

INIT_SPIN, EXPLORE, APPROACH, RESCUE, RETURN, RECOVERY, DONE = (
    "INIT_SPIN",
    "EXPLORE",
    "APPROACH",
    "RESCUE",
    "RETURN",
    "RECOVERY",
    "DONE",
)
_MOVING = (EXPLORE, APPROACH, RETURN)

Point = tuple[float, float]


def compass_angle(vec: Sequence[float]) -> float:
    """나침반 벡터의 원시각 [rad]. Intro tb3_teleop_sensors.py와 같은 식."""
    return math.atan2(vec[1], vec[0])


def fit_compass(samples: Sequence[tuple[float, float]]) -> tuple[int, float, float] | None:
    """시작 회전 표본 [(오도메트리 방향, 나침반 원시각), ...]으로 방향 = sign·원시각 + offset 추정.

    오도메트리는 제자리 회전에서 회전량 오차가 커서(#9 측정 방향 오차 약 0.59 rad) 방향 값을
    직접 맞추지 않는다.
    - sign: 오도메트리 증분과 원시각 증분의 곱의 합 부호
    - offset: 첫 표본에서 계산. 첫 표본의 방향은 설정한 시작 방향(START_THETA)이므로 정확함
    - scale: 오도메트리 누적 회전량 / 나침반 누적 회전량. 오도메트리 회전 오차 확인용
    반환: (sign, offset, scale). 나침반 누적 회전량이 π 미만이거나 scale이 1에서
    COMPASS_SCALE_TOL 이상 벗어나면 None.
    """
    corr = odom_turn = compass_turn = 0.0
    for (t0, r0), (t1, r1) in zip(samples, samples[1:], strict=False):
        dt, dr = wrap(t1 - t0), wrap(r1 - r0)
        corr += dt * dr
        odom_turn += abs(dt)
        compass_turn += abs(dr)
    if compass_turn < math.pi or corr == 0.0:
        return None
    scale = odom_turn / compass_turn
    if abs(scale - 1.0) > config.COMPASS_SCALE_TOL:
        return None
    sign = 1 if corr > 0 else -1
    theta0, raw0 = samples[0]
    return sign, wrap(theta0 - sign * raw0), scale


def compass_heading(vec: Sequence[float], sign: int, offset: float) -> float:
    """보정된 나침반 방향 [rad]."""
    return wrap(sign * compass_angle(vec) + offset)


class Mission:
    """상태 머신. io·odom·grid·detector는 CONTEXT.md 6장 인터페이스를 따르는 객체."""

    def __init__(self, io, odom, grid, detector, log: Callable[[str], None] | None = None):
        self.io, self.odom, self.grid, self.detector = io, odom, grid, detector
        self.log = log or (lambda _msg: None)
        x, y, _ = odom.pose()
        self.start: Point = (x, y)
        self.state = INIT_SPIN
        self.t = 0.0
        self.state_t = 0.0
        self.steps = 0
        # 나침반 보정
        self.compass_fit: tuple[int, float, float] | None = None
        self._compass_vec = None
        self._spin_samples: list[tuple[float, float]] = []
        self._spin_turn = 0.0
        self._last_theta: float | None = None
        # 인식
        self.confirm = Confirm()
        self.track = Confirm()
        self.target: Point | None = None
        self.candidates: list[Point] = []
        self.rescued: list[tuple[float, float, float]] = []
        self.rejected: list[Point] = []
        self.target_det: dict | None = None  # 대상과 일치한 최근 검출 (진단 로그용)
        self._approach_t = 0.0
        # 경로
        self.goal: Point | None = None
        self.goal_t = 0.0
        self.path: list[Point] | None = None
        self.plan_t = -math.inf
        self.blacklist: list[Point] = []
        # 안전·정체
        self._blocked_since: float | None = None
        self._stuck_ref = (x, y, 0.0)
        self._resume = EXPLORE
        self._turn_dir = 1.0
        # 출력
        self.trajectory: list[Point] = [self.start]
        self._map_t = -math.inf
        self._status_t = -math.inf

    # 매 step 처리

    def tick(self) -> None:
        """센서 → 오도메트리 → 지도 → 인식 → 상태 머신 → 안전 필터 → drive."""
        self.t = self.io.time()
        ranges = self.io.lidar()
        self._compass_vec = self.io.compass()
        heading = None
        if self.compass_fit is not None and self._compass_vec is not None:
            heading = compass_heading(self._compass_vec, *self.compass_fit[:2])
        self.odom.update(*self.io.encoders(), compass=heading)
        pose = self.odom.pose()
        self.grid.update(pose, ranges)
        if self.steps % config.YOLO_EVERY == 0 and self.state in (INIT_SPIN, EXPLORE, APPROACH):
            self._perceive(pose)
        self.steps += 1

        if (
            self.state not in (RETURN, DONE, RECOVERY)
            and self.t >= config.TIME_LIMIT - config.RETURN_RESERVE
        ):
            self._set_state(RETURN, "시간 예비")

        v, w = self._handlers[self.state](self, pose, ranges)
        v, w, blocked = safety_filter(v, w, ranges)
        self._watch(pose, blocked)
        self.io.drive(v, w)
        self._record(pose)

    # 상태별 처리. 반환: (v, w)

    def _init_spin(self, pose, ranges) -> tuple[float, float]:
        theta = pose[2]
        if self._compass_vec is not None:
            raw = compass_angle(self._compass_vec)
            if self._spin_samples:
                # 나침반이 있으면 실제 회전량으로 1회전 판정 (오도메트리 회전량 오차 회피)
                self._spin_turn += abs(wrap(raw - self._spin_samples[-1][1]))
            self._spin_samples.append((theta, raw))
        elif self._last_theta is not None:
            self._spin_turn += abs(wrap(theta - self._last_theta))
        self._last_theta = theta
        spin_limit = 2 * (2 * math.pi / config.INIT_SPIN_W)  # 정상 회전 시간의 2배
        if self._spin_turn >= 2 * math.pi or self.t - self.state_t > spin_limit:
            self.compass_fit = fit_compass(self._spin_samples)
            if self.compass_fit is None:
                self.log(f"[t={self.t:.1f}s] 나침반 보정 실패, 엔코더 방향만 사용")
            else:
                sign, offset, scale = self.compass_fit
                self.odom.set_wheel_separation_scale(scale)
                self.log(
                    f"[t={self.t:.1f}s] 나침반 보정 sign={sign} offset={offset:.3f} "
                    f"오도메트리/나침반 회전 비율={scale:.3f}"
                )
            self._set_state(APPROACH if self.target else EXPLORE, "시작 회전 완료")
            return 0.0, 0.0
        return 0.0, config.INIT_SPIN_W

    def _explore(self, pose, ranges) -> tuple[float, float]:
        if self.target is not None:
            self._set_state(APPROACH, self._target_text())
            return 0.0, 0.0
        x, y, _ = pose
        # 미구조 후보를 프론티어보다 먼저 확인 (과제 문서 8.2절)
        self.candidates = [
            c for c in self.candidates if math.dist(c, (x, y)) > config.CANDIDATE_DROP_DIST
        ]
        if self.path is None or self.t - self.plan_t >= config.REPLAN_PERIOD:
            goal = self._nearest_candidate((x, y))
            if goal is None:
                goal = planner.choose_frontier(self.grid, pose, self.blacklist)
            if goal is None:
                self._set_state(RETURN, "프론티어 없음")
                return 0.0, 0.0
            if self.goal is None or math.dist(goal, self.goal) > config.BLACKLIST_RADIUS:
                self.goal_t = self.t
            self.goal = goal
            if self.t - self.goal_t > config.FRONTIER_TIMEOUT:
                self._reject_goal("목표 제한 시간 초과")
                return 0.0, 0.0
            self.path = planner.plan(self.grid, (x, y), goal)
            self.plan_t = self.t
            if self.path is None:
                self._reject_goal("경로 없음")
                return 0.0, 0.0
        v, w, reached = pure_pursuit(pose, self.path)
        if reached:
            self.path = None  # 다음 step에 새 목표 선택
        return v, w

    def _approach(self, pose, ranges) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = self.target
        dist = math.hypot(tx - x, ty - y)
        if self.t - self._approach_t > config.APPROACH_TIMEOUT:
            self.rejected.append(self.target)
            self.target = None
            self.path = None
            self._set_state(EXPLORE, "접근 제한 시간 초과")
            return 0.0, 0.0
        if dist <= config.APPROACH_DIST + config.GOAL_TOL:
            bearing = wrap(math.atan2(ty - y, tx - x) - theta)
            if abs(bearing) <= config.FACE_TOL:
                self._set_state(RESCUE, self._target_text())
                return 0.0, 0.0
            return 0.0, self._face(bearing)
        if self.path is None or self.t - self.plan_t >= config.REPLAN_PERIOD:
            ux, uy = (tx - x) / dist, (ty - y) / dist
            stand = (tx - ux * config.APPROACH_DIST, ty - uy * config.APPROACH_DIST)
            self.path = planner.plan(self.grid, (x, y), stand) or planner.plan(
                self.grid, (x, y), stand, allow_unknown=True
            )
            self.plan_t = self.t
            if self.path is None:
                return 0.0, 0.0  # 지도 갱신 후 다음 주기에 재계획
        v, w, _ = pure_pursuit(pose, self.path)
        return min(v, config.V_APPROACH), w

    def _rescue(self, pose, ranges) -> tuple[float, float]:
        if self.t - self.state_t >= config.RESCUE_HOLD:
            self.target = None
            self.path = None
            self.confirm.reset()
            if len(self.rescued) >= config.TARGET_COUNT:
                self._set_state(RETURN, f"구조 {len(self.rescued)}개 완료")
            else:
                self._set_state(EXPLORE, f"구조 {len(self.rescued)}/{config.TARGET_COUNT}")
        return 0.0, 0.0

    def _return(self, pose, ranges) -> tuple[float, float]:
        x, y, theta = pose
        dist = math.dist((x, y), self.start)
        if dist <= config.RETURN_TOL:
            self._set_state(DONE, f"시작점 거리 {dist:.3f} m")
            return 0.0, 0.0
        if dist <= config.GOAL_TOL:
            # pure_pursuit는 GOAL_TOL에서 멈추므로 RETURN_TOL까지 직진으로 접근
            bearing = wrap(math.atan2(self.start[1] - y, self.start[0] - x) - theta)
            if abs(bearing) > config.FACE_TOL:
                return 0.0, self._face(bearing)
            return min(config.V_APPROACH, dist), 0.0
        if self.path is None or self.t - self.plan_t >= config.REPLAN_PERIOD:
            self.path = planner.plan(self.grid, (x, y), self.start, allow_unknown=True) or self.path
            self.plan_t = self.t
        if self.path is None:
            return 0.0, config.INIT_SPIN_W  # 경로 없음: 회전하며 지도 갱신
        v, w, _ = pure_pursuit(pose, self.path)
        return v, w

    def _recovery(self, pose, ranges) -> tuple[float, float]:
        elapsed = self.t - self.state_t
        if elapsed < config.RECOVERY_BACK_TIME:
            return -config.V_APPROACH, 0.0
        if elapsed < config.RECOVERY_BACK_TIME + config.RECOVERY_TURN_TIME:
            return 0.0, self._turn_dir * config.W_MAX
        self.path = None
        self._set_state(self._resume, "복구 완료")
        return 0.0, 0.0

    def _done(self, pose, ranges) -> tuple[float, float]:
        return 0.0, 0.0

    _handlers = {
        INIT_SPIN: _init_spin,
        EXPLORE: _explore,
        APPROACH: _approach,
        RESCUE: _rescue,
        RETURN: _return,
        RECOVERY: _recovery,
        DONE: _done,
    }

    # 인식

    def _perceive(self, pose) -> None:
        """YOLO 실행 step에서만 호출. Confirm은 이 시점에만 갱신 (분석 문서 6장 5번)."""
        found = [(r[0], r[1]) for r in self.rescued] + self.rejected
        seen: list[tuple[Point, dict]] = []
        for det in self.detector.detect_all(self.io.camera_bgr()):
            xy = tuple(self.detector.to_world(det, pose))
            if not is_excluded(xy, found):
                seen.append((xy, det))
        if self.target is not None:
            # 확정된 대상이 있으면 일치 검출로 위치만 갱신하고 나머지는 후보로 저장
            for xy, det in seen:
                if math.dist(xy, self.target) <= config.CONFIRM_MATCH_RADIUS:
                    _, estimate = self.track.update(True, xy, det["dist"])
                    self.target = estimate
                    self.target_det = det
                else:
                    self._add_candidate(xy)
            return
        if not seen:
            self.confirm.update(False, None)
            return
        (xy, det), others = seen[0], seen[1:]
        confirmed, estimate = self.confirm.update(True, xy, det["dist"])
        for other, _ in others:
            self._add_candidate(other)
        if confirmed:
            self.target = estimate
            self.target_det = det
            self.confirm.reset()
            self.track = Confirm()
            self.track.update(True, estimate, det["dist"])
            self.log(f"[t={self.t:.1f}s] 대상 확정 {self._det_text()}")
            self.candidates = [
                c for c in self.candidates if math.dist(c, estimate) > config.FOUND_EXCLUDE_RADIUS
            ]

    def _add_candidate(self, xy: Point) -> None:
        for i, c in enumerate(self.candidates):
            if math.dist(c, xy) <= config.FOUND_EXCLUDE_RADIUS:
                self.candidates[i] = xy  # 최근 관측 위치로 갱신
                return
        if self.target is None or math.dist(xy, self.target) > config.FOUND_EXCLUDE_RADIUS:
            self.candidates.append(xy)

    def _nearest_candidate(self, xy: Point) -> Point | None:
        return min(self.candidates, key=lambda c: math.dist(c, xy), default=None)

    # 공통 처리

    def _set_state(self, new: str, reason: str) -> None:
        if new == self.state:
            return
        old = self.state
        self.log(f"[t={self.t:.1f}s] {old} -> {new} ({reason})")
        self.state = new
        self.state_t = self.t
        self.path = None
        self.plan_t = -math.inf
        self._blocked_since = None
        x, y, _ = self.odom.pose()
        self._stuck_ref = (x, y, self.t)
        if new == APPROACH and old != RECOVERY:
            self._approach_t = self.t  # RECOVERY 복귀는 접근 제한 시간을 초기화하지 않음
        if new == RESCUE:
            self.rescued.append((self.target[0], self.target[1], self.t))
            n = len(self.rescued)
            self._save_map(f"map_rescue_{n}.png")
            # 구조 대상 확인용 카메라 화면. 오탐 여부를 사람이 확인
            frame = self.io.camera_bgr()
            if frame is not None:
                viz.save_map(os.path.join(config.MAP_SAVE_DIR, f"rescue_{n}_camera.png"), frame)
            self.log(f"[t={self.t:.1f}s] 구조 {n} 검출 {self._det_text()}")
        elif new == DONE:
            self._save_map("map_final.png")
            spots = ", ".join(f"({rx:.2f}, {ry:.2f})" for rx, ry, _ in self.rescued)
            self.log(f"[t={self.t:.1f}s] 결과: 구조 {len(self.rescued)}개 {spots}")

    def _reject_goal(self, reason: str) -> None:
        if self.goal in self.candidates:
            self.candidates.remove(self.goal)
        else:
            self.blacklist.append(self.goal)
        self.log(f"[t={self.t:.1f}s] 목표 ({self.goal[0]:.2f},{self.goal[1]:.2f}) 제외: {reason}")
        self.goal = None
        self.path = None

    def _face(self, bearing: float) -> float:
        return max(-config.W_MAX, min(config.W_MAX, config.FACE_GAIN * bearing))

    def _watch(self, pose, blocked: bool) -> None:
        """안전 필터 연속 차단 → 재계획, 이동 정체 → RECOVERY."""
        x, y, _ = pose
        if blocked:
            if self._blocked_since is None:
                self._blocked_since = self.t
            elif self.t - self._blocked_since >= config.BLOCKED_TIME:
                self.path = None
                self._blocked_since = self.t
        else:
            self._blocked_since = None
        rx, ry, rt = self._stuck_ref
        if self.state not in _MOVING or math.hypot(x - rx, y - ry) >= config.STUCK_DIST:
            self._stuck_ref = (x, y, self.t)
        elif self.t - rt >= config.STUCK_TIME:
            self._resume = self.state
            self._turn_dir = self._wider_side()
            self._set_state(RECOVERY, f"{config.STUCK_TIME:.0f}초 정체")

    def _wider_side(self) -> float:
        """라이다 왼쪽(인덱스 90 주변)과 오른쪽(270 주변) 중 평균 거리가 긴 쪽. 왼쪽 +1."""
        ranges = self.io.lidar()
        if not ranges:
            return 1.0
        n = len(ranges)

        def mean(center: int) -> float:
            quarter = n // 8
            vals = [
                min(ranges[(center + k) % n], config.LIDAR_MAX) for k in range(-quarter, quarter)
            ]
            return sum(vals) / len(vals)

        return 1.0 if mean(n // 4) >= mean(3 * n // 4) else -1.0

    def _det_text(self) -> str:
        d = self.target_det or {}
        return (
            f"source={d.get('source')} cls={d.get('cls')} conf={d.get('conf', 0):.2f} "
            f"w={d.get('w', 0):.0f} h={d.get('h', 0):.0f} dist={d.get('dist', 0):.2f}"
        )

    def _target_text(self) -> str:
        return f"{config.TARGET_COLOR} at {self.target[0]:.2f},{self.target[1]:.2f}"

    def _record(self, pose) -> None:
        x, y, theta = pose
        if math.dist(self.trajectory[-1], (x, y)) >= config.TRAJ_STEP:
            self.trajectory.append((x, y))
        if self.state != DONE and self.t - self._map_t >= config.MAP_SAVE_PERIOD:
            self._map_t = self.t
            self._save_map("map_latest.png")
        if self.t - self._status_t >= config.LOG_INTERVAL:
            self._status_t = self.t
            self.log(
                f"[t={self.t:.1f}s] {self.state} pose=({x:.2f}, {y:.2f}, {theta:.2f}) "
                f"구조 {len(self.rescued)}/{config.TARGET_COUNT} 후보 {len(self.candidates)}"
            )

    def _save_map(self, name: str) -> None:
        image = viz.render_map(
            self.grid.public(),
            self.grid.to_cell,
            trajectory=self.trajectory,
            path=self.path or (),
            rescued=self.rescued,
            start=self.start,
            pose=self.odom.pose(),
            title=f"t={self.t:.1f}s {self.state}",
        )
        if not viz.save_map(os.path.join(config.MAP_SAVE_DIR, name), image):
            self.log(f"[t={self.t:.1f}s] 지도 저장 실패: {name}")


if __name__ == "__main__":
    # Webots 없이 실행: controllers/sar_main에서 python -m sar.mission
    # 나침반 보정: Webots 규약(북쪽 +y)에서 원시각 = π/2 - 방향 → sign -1, offset π/2.
    # 오도메트리 회전량이 10% 크게 측정되어도 시작 방향 기준으로 정확히 보정됨
    spin = [(wrap(k * 0.33), wrap(math.pi / 2 - k * 0.3)) for k in range(22)]
    sign, offset, scale = fit_compass(spin)
    assert sign == -1 and abs(wrap(offset - math.pi / 2)) < 1e-9 and abs(scale - 1.1) < 1e-9
    assert fit_compass([(0.0, 0.0), (0.1, 0.1)]) is None  # 회전량 부족
    assert abs(compass_heading((1.0, 0.0), sign, offset) - math.pi / 2) < 1e-9
    print("mission self-test ok")
