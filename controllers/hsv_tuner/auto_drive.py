"""hsv_tuner 자동 주행. Webots 없이 실행되는 판단 로직만 포함한다.

- AutoApproach: 제자리 회전으로 대상 탐색 → 정면 정렬 → 접근. 측정 지점(3, 2, 1 m)과
  정지 지점에서 측정 요청
- SpinSurvey: 제자리 1회전하며 일정 각도마다 측정 요청. 방해 요소 기록용

update()는 매 step 호출한다. fresh는 이번 step에 검출을 새로 실행했는지 여부이며,
측정 요청은 새 검출 결과가 있는 step에서만 발생한다.
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass

from sar import config

_FULL_TURN = 2 * math.pi
_REALIGN_FACTOR = 4  # 방위각이 정렬 허용 오차의 이 배수를 넘으면 전진을 멈추고 재정렬


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass
class Command:
    """자동 주행 1 step 결과."""

    v: float = 0.0  # m/s
    w: float = 0.0  # rad/s
    measure: str | None = None  # 측정 요청 사유. CSV note 열에 기록
    message: str | None = None  # 콘솔 출력 문구
    done: bool = False  # True면 자동 주행 종료


class _TurnCounter:
    """누적 회전량 [rad] 계산."""

    def __init__(self, theta: float):
        self.last = theta
        self.total = 0.0

    def update(self, theta: float) -> float:
        self.total += abs(_wrap(theta - self.last))
        self.last = theta
        return self.total


class AutoApproach:
    """가장 가까운 대상을 찾아 접근하며 거리별 측정 요청."""

    def __init__(self, t: float, pose: Sequence[float], dists=config.TUNER_MEASURE_DISTS):
        self.state = "SCAN"
        self._turns = _TurnCounter(pose[2])
        self._last_seen: float | None = None
        self._pending = sorted(dists, reverse=True)

    def _align_w(self, bearing: float) -> float:
        return _clip(config.TUNER_ALIGN_GAIN * bearing, config.TUNER_SCAN_W)

    def update(
        self, t: float, pose: Sequence[float], detections: list[dict], fresh: bool
    ) -> Command:
        """현재 검출 결과로 속도 명령과 측정 요청 계산."""
        turned = self._turns.update(pose[2])
        target = detections[0] if detections else None
        if target is not None and fresh:
            self._last_seen = t
        lost = target is None and (
            self._last_seen is None or t - self._last_seen >= config.TUNER_LOST_TIMEOUT
        )

        if self.state == "SCAN":
            if target is not None:
                self.state = "ALIGN"
                return Command(message=f"대상 발견 d={target['dist']:.2f}m, 정렬 시작")
            if turned >= config.TUNER_SCAN_TURNS * _FULL_TURN:
                return Command(message="대상을 찾지 못해 자동 접근을 종료합니다.", done=True)
            return Command(w=config.TUNER_SCAN_W)

        if target is None:
            if lost:
                self.state = "SCAN"
                self._turns = _TurnCounter(pose[2])
                return Command(message="대상을 놓쳐 다시 탐색합니다.")
            return Command()  # 다음 검출까지 정지 대기

        bearing, dist = target["bearing"], target["dist"]
        if self.state == "ALIGN":
            if abs(bearing) > config.TUNER_ALIGN_TOL:
                return Command(w=self._align_w(bearing))
            self.state = "APPROACH"
            skipped = [d for d in self._pending if d > dist]
            self._pending = [d for d in self._pending if d <= dist]
            note = f", 이미 지난 측정 지점 {skipped} 생략" if skipped else ""
            return Command(message=f"정렬 완료 d={dist:.2f}m, 접근 시작{note}")

        # APPROACH
        if fresh and dist <= config.TUNER_STOP_DIST:
            return Command(
                measure=f"auto stop d={dist:.2f}m",
                message=f"정지 거리 도달 d={dist:.2f}m, 자동 접근 종료",
                done=True,
            )
        if fresh and self._pending and dist <= self._pending[0]:
            mark = self._pending.pop(0)
            return Command(
                w=self._align_w(bearing),
                measure=f"auto {mark:.1f}m d={dist:.2f}m",
                message=f"{mark:.1f} m 지점 측정 (추정 {dist:.2f}m)",
            )
        if abs(bearing) > _REALIGN_FACTOR * config.TUNER_ALIGN_TOL:
            return Command(w=self._align_w(bearing))
        return Command(v=config.TUNER_APPROACH_V, w=self._align_w(bearing))


class SpinSurvey:
    """제자리 1회전하며 TUNER_SURVEY_STEP마다 측정 요청."""

    def __init__(self, t: float, pose: Sequence[float]):
        self._turns = _TurnCounter(pose[2])
        self._next = 0.0

    def update(
        self, t: float, pose: Sequence[float], detections: list[dict], fresh: bool
    ) -> Command:
        """회전 명령과 각도별 측정 요청 계산."""
        turned = self._turns.update(pose[2])
        if turned >= _FULL_TURN:
            return Command(message="1회전 측정 완료", done=True)
        if fresh and turned >= self._next:
            deg = round(math.degrees(self._next))
            self._next += config.TUNER_SURVEY_STEP
            return Command(w=config.TUNER_SCAN_W, measure=f"survey {deg}deg")
        return Command(w=config.TUNER_SCAN_W)
