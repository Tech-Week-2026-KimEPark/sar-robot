"""대상 사과 검출과 월드 좌표 추정.

검출 순서 (과제와 구현 기준 9장)
1. YOLO11n으로 apple·orange·sports ball 후보 상자 검출
2. 상자 안 대상 색 픽셀 비율이 COLOR_RATIO_MIN 이상인 후보만 대상으로 판정
3. 같은 프레임의 색 분할(면적·원형도 조건) 덩어리 중 YOLO 대상 상자 밖에 있는 것을 추가.
   YOLO가 놓친 먼 사과나 한 화면의 두 번째 사과를 보완
4. 중심이 화면 가운데선보다 HORIZON_MARGIN 이상 위인 후보는 식탁 위 물체로 제외

거리는 사과 지름과 상자 크기로 추정하고, 위치는 Confirm의 거리 가중 평균으로 확정한다.
"""

import math
import time
from collections.abc import Iterable, Sequence

import cv2
import numpy as np

from sar import config

HsvRange = tuple[tuple[int, int, int], tuple[int, int, int]]


def focal_length_px(image_width: int, fov: float = config.CAMERA_FOV) -> float:
    """수평 화각 [rad]과 이미지 폭 [px]으로 초점거리 [px] 계산."""
    return (image_width / 2) / math.tan(fov / 2)


def bearing_from_cx(cx: float, image_width: int, fov: float = config.CAMERA_FOV) -> float:
    """대상 중심 가로 좌표 [px]의 방위각 [rad]. 왼쪽이 양수."""
    return -math.atan((cx - image_width / 2) / focal_length_px(image_width, fov))


def distance_from_size(
    size_px: float,
    bearing: float,
    image_width: int,
    fov: float = config.CAMERA_FOV,
    diameter: float = config.TARGET_DIAMETER,
) -> float | None:
    """대상 겉보기 지름 [px]으로 카메라에서 대상까지 거리 [m] 추정. 크기가 0이면 None.

    f·D/w는 광축 방향 깊이이므로 방위각의 cos으로 나눠 직선거리로 변환.
    """
    if size_px <= 0:
        return None
    depth = focal_length_px(image_width, fov) * diameter / size_px
    return depth / math.cos(bearing)


def color_mask(bgr: np.ndarray, ranges: Sequence[HsvRange]) -> np.ndarray:
    """HSV 범위 합집합의 마스크 (uint8, 0 또는 255) 반환."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    return mask


def box_color_ratio(mask: np.ndarray, x1: float, y1: float, x2: float, y2: float) -> float:
    """상자 안 마스크 픽셀 비율 (0~1). 상자가 비어 있으면 0."""
    h, w = mask.shape[:2]
    c1, r1 = max(int(x1), 0), max(int(y1), 0)
    c2, r2 = min(int(math.ceil(x2)), w), min(int(math.ceil(y2)), h)
    if c2 <= c1 or r2 <= r1:
        return 0.0
    return float(np.count_nonzero(mask[r1:r2, c1:c2])) / ((r2 - r1) * (c2 - c1))


def _blob_body(contour: np.ndarray) -> np.ndarray | None:
    """덩어리에서 꼭지 같은 가는 돌출부를 지운 본체 윤곽. 본체가 사라지면 None.

    열림 연산 커널은 덩어리 짧은 변의 BLOB_OPEN_RATIO배. 3 px 미만이면 원래 윤곽 사용.
    """
    x, y, w, h = cv2.boundingRect(contour)
    size = round(min(w, h) * config.BLOB_OPEN_RATIO)
    if size < config.MASK_KERNEL_SIZE:
        return contour
    blob = np.zeros((h + 2, w + 2), np.uint8)
    cv2.drawContours(blob, [contour - (x - 1, y - 1)], -1, 255, -1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size | 1, size | 1))
    body = cv2.morphologyEx(blob, cv2.MORPH_OPEN, kernel)
    parts, _ = cv2.findContours(body, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not parts:
        return None
    return max(parts, key=cv2.contourArea) + (x - 1, y - 1)


def find_color_blobs(mask: np.ndarray) -> list[dict]:
    """마스크에서 사과 모양 덩어리 목록 반환.

    조건: 면적 MIN_BLOB_AREA 이상, 화면 가장자리에 닿지 않음, 꼭지 제거 후 원형도
    MIN_CIRCULARITY 이상, 외접원 채움 비율 MIN_BLOB_FILL 이상.
    반환 항목: {"cx", "cy", "w", "h", "conf"}. w·h는 본체 외접원 지름, conf는 원형도.
    """
    height, width = mask.shape[:2]
    k = config.MASK_KERNEL_SIZE
    clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for contour in contours:
        if cv2.contourArea(contour) < config.MIN_BLOB_AREA:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 0 or y <= 0 or x + w >= width or y + h >= height:
            continue  # 화면 밖으로 잘린 덩어리는 모양·크기를 판단할 수 없음
        body = _blob_body(contour)
        if body is None:
            continue
        area = cv2.contourArea(body)
        perimeter = cv2.arcLength(body, True)
        if perimeter <= 0:
            continue
        circularity = 4 * math.pi * area / perimeter**2
        (cx, cy), radius = cv2.minEnclosingCircle(body)
        fill = area / (math.pi * radius**2) if radius > 0 else 0.0
        if circularity < config.MIN_CIRCULARITY or fill < config.MIN_BLOB_FILL:
            continue
        blobs.append({"cx": cx, "cy": cy, "w": 2 * radius, "h": 2 * radius, "conf": circularity})
    return blobs


def _center_inside(item: dict, box: dict) -> bool:
    """item 중심이 box 상자(cx, cy, w, h) 안에 있으면 True."""
    return (
        abs(item["cx"] - box["cx"]) <= box["w"] / 2 and abs(item["cy"] - box["cy"]) <= box["h"] / 2
    )


def to_world(det: dict, pose: Sequence[float]) -> tuple[float, float]:
    """검출 결과의 거리·방위각과 로봇 pose (x, y, theta)로 대상 월드 좌표 (x, y) 계산."""
    x, y, theta = pose
    angle = theta + det["bearing"]
    return x + det["dist"] * math.cos(angle), y + det["dist"] * math.sin(angle)


def is_excluded(
    xy: Sequence[float],
    found: Iterable[Sequence[float]],
    radius: float = config.FOUND_EXCLUDE_RADIUS,
) -> bool:
    """구조 완료 위치 중 radius [m] 이내에 있는 위치면 True. 같은 사과 재구조 방지용."""
    return any(math.dist(xy, f) <= radius for f in found)


class TargetDetector:
    """YOLO 후보 검출과 HSV 색 판별로 대상 색 사과 검출.

    YOLO 모델은 생성 시 1회만 로드. model_path가 None이거나 로드에 실패하면
    색 분할만 사용하고 실패 사유를 load_error에 기록.
    """

    def __init__(
        self,
        color: str,
        model_path: str | None,
        device: str = config.YOLO_DEVICE,
        ranges: Sequence[HsvRange] | None = None,
    ):
        if ranges is None:
            if color not in config.HSV_RANGES:
                raise ValueError(f"HSV_RANGES에 없는 색: {color}")
            ranges = config.HSV_RANGES[color]
        self.color = color
        self.ranges = list(ranges)
        self.model = None
        self.load_error: str | None = None
        self.last_error: str | None = None
        # 마지막 detect_all() 호출의 YOLO 원본 상자(색 판별 탈락 포함)와 추론 시간 [ms]. 측정·튜닝용
        self.last_yolo: list[dict] = []
        self.last_yolo_ms: float | None = None
        if model_path is not None:
            try:
                from ultralytics import YOLO

                self.model = YOLO(model_path)
                self.model.to(device)
            except Exception as exc:  # noqa: BLE001 - 모델 없이 색 분할로 계속 동작
                self.model = None
                self.load_error = f"{type(exc).__name__}: {exc}"

    def _yolo_boxes(self, bgr: np.ndarray) -> list[dict]:
        if self.model is None:
            return []
        start = time.perf_counter()
        try:
            result = self.model.predict(
                bgr,
                conf=config.YOLO_CONF,
                classes=config.YOLO_CLASSES,
                agnostic_nms=True,  # 클래스 무관 NMS. 한 사과의 apple·sports ball 중복 방지
                verbose=False,
            )[0]
        except Exception as exc:  # noqa: BLE001 - 추론 실패 프레임은 색 분할로 대체
            self.last_error = f"{type(exc).__name__}: {exc}"
            return []
        self.last_yolo_ms = (time.perf_counter() - start) * 1000
        boxes = result.boxes
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy()
        return [
            {"xyxy": tuple(float(v) for v in box), "conf": float(conf), "cls": int(cls)}
            for box, conf, cls in zip(xyxy, confs, classes, strict=True)
        ]

    def detect_all(self, bgr: np.ndarray | None) -> list[dict]:
        """대상 검출 결과 전체를 가까운 순서로 반환. 이미지가 None이면 빈 목록."""
        if bgr is None:
            self.last_yolo = []
            return []
        height, width = bgr.shape[:2]
        mask = color_mask(bgr, self.ranges)

        candidates = []
        self.last_yolo = self._yolo_boxes(bgr)
        for box in self.last_yolo:
            x1, y1, x2, y2 = box["xyxy"]
            box["ratio"] = box_color_ratio(mask, x1, y1, x2, y2)
            if box["ratio"] < config.COLOR_RATIO_MIN:
                continue
            candidates.append(
                {
                    "cx": (x1 + x2) / 2,
                    "cy": (y1 + y2) / 2,
                    "w": x2 - x1,
                    "h": y2 - y1,
                    "conf": box["conf"],
                    "cls": box["cls"],
                    "source": "yolo",
                }
            )
        if config.USE_COLOR_FALLBACK:
            yolo_targets = list(candidates)
            for blob in find_color_blobs(mask):
                if any(_center_inside(blob, box) for box in yolo_targets):
                    continue  # YOLO가 이미 찾은 대상
                candidates.append({**blob, "cls": None, "source": "color"})

        detections = []
        for det in candidates:
            if det["cy"] < height / 2 - config.HORIZON_MARGIN:
                continue  # 식탁 위 과일 등 바닥보다 높은 물체
            bearing = bearing_from_cx(det["cx"], width)
            # 화면 가장자리에서 잘린 상자는 한 변만 줄어들므로 긴 변을 지름으로 사용
            dist = distance_from_size(max(det["w"], det["h"]), bearing, width)
            if dist is None:
                continue
            detections.append({**det, "color": self.color, "dist": dist, "bearing": bearing})
        detections.sort(key=lambda d: d["dist"])
        return detections

    def detect(self, bgr: np.ndarray | None) -> dict | None:
        """가장 가까운 대상 1개 반환. 없으면 None.

        반환: {"cx", "cy", "w", "h", "conf", "cls", "color", "dist", "bearing", "source"}
        source는 "yolo" 또는 "color" (YOLO가 놓쳐 색 분할로 찾은 경우).
        """
        detections = self.detect_all(bgr)
        return detections[0] if detections else None

    def to_world(self, det: dict, pose: Sequence[float]) -> tuple[float, float]:
        """검출 결과의 대상 월드 좌표 (x, y) [m]."""
        return to_world(det, pose)


class Confirm:
    """연속 검출 확인과 거리 가중 평균 위치 추정.

    CONFIRM_FRAMES 프레임 연속으로 CONFIRM_MATCH_RADIUS 안에서 검출되면 확정.
    미검출이거나 위치가 크게 바뀌면 관측 기록을 초기화.
    """

    def __init__(
        self,
        frames: int = config.CONFIRM_FRAMES,
        match_radius: float = config.CONFIRM_MATCH_RADIUS,
        min_dist: float = config.CONFIRM_MIN_DIST,
    ):
        self.frames = frames
        self.match_radius = match_radius
        self.min_dist = min_dist
        self._sum_w = 0.0
        self._sum_x = 0.0
        self._sum_y = 0.0
        self._count = 0

    def reset(self) -> None:
        """관측 기록 초기화."""
        self._sum_w = self._sum_x = self._sum_y = 0.0
        self._count = 0

    def estimate(self) -> tuple[float, float] | None:
        """현재 관측의 거리 가중 평균 위치. 관측이 없으면 None."""
        if self._count == 0:
            return None
        return self._sum_x / self._sum_w, self._sum_y / self._sum_w

    def update(
        self, seen: bool, xy: Sequence[float] | None, dist: float | None = None
    ) -> tuple[bool, tuple[float, float] | None]:
        """관측 1회 반영 후 (확정 여부, 위치 추정) 반환.

        가중치는 1 / max(dist, min_dist)². dist가 None이면 거리 1 m로 취급.
        """
        if not seen or xy is None:
            self.reset()
            return False, None
        current = self.estimate()
        if current is not None and math.dist(current, xy) > self.match_radius:
            self.reset()
        d = 1.0 if dist is None else max(dist, self.min_dist)
        weight = 1.0 / d**2
        self._sum_w += weight
        self._sum_x += weight * xy[0]
        self._sum_y += weight * xy[1]
        self._count += 1
        return self._count >= self.frames, self.estimate()


if __name__ == "__main__":
    # Webots 없이 실행하는 단독 확인: 합성 이미지의 빨간 원 검출
    image = np.full((480, 640, 3), 90, np.uint8)
    cv2.circle(image, (400, 300), 12, (0, 0, 255), -1)
    detector = TargetDetector("red", model_path=None)
    found = detector.detect(image)
    print(found)
    if found is not None:
        print("world:", detector.to_world(found, (0.0, 0.0, 0.0)))
