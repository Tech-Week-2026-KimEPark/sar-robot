"""hsv_tuner 측정 CSV 분석.

사과 실제 좌표를 인자로 받아 추정 거리 오차, 역산 사과 지름, YOLO 클래스·신뢰도·색 비율 분포,
YOLO 추론 시간을 Markdown 표로 출력한다. 실제 거리는 CSV의 로봇 pose(엔코더 오도메트리)와
사과 좌표로 계산하므로 오도메트리 오차가 포함된다.

사용 예 (저장소 루트):
    python scripts/analyze_measurements.py controllers/hsv_tuner/output/measurements.csv \\
        --target=-12.02,-3.02 --target=-5.34,-10.54

좌표가 음수로 시작하므로 `--target=X,Y`처럼 등호로 붙여 쓴다.
"""

import argparse
import csv
import math
import os
import statistics
import sys
from collections.abc import Sequence

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers", "sar_main")
)

from sar import config  # noqa: E402
from sar.perception import bearing_from_cx, distance_from_size, to_world  # noqa: E402

IMAGE_WIDTH = 640  # px, 튜닝 컨트롤러 카메라 이미지 폭
MATCH_RADIUS = 1.0  # m, 추정 위치를 실제 사과에 대응시키는 최대 거리
DIST_BANDS = [(0.0, 1.5), (1.5, 2.5), (2.5, float("inf"))]  # m, 거리 오차 요약 구간


def load_rows(path: str) -> list[dict]:
    """측정 CSV 읽기. 숫자 열은 float, 빈 칸은 None."""
    with open(path, encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    numeric = {
        "sim_t", "robot_x", "robot_y", "robot_theta_deg", "conf", "color_ratio",
        "cx", "cy", "w", "h", "dist", "bearing_deg", "world_x", "world_y", "yolo_ms",
    }  # fmt: skip
    for row in rows:
        for key in numeric & row.keys():
            row[key] = float(row[key]) if row[key] not in ("", None) else None
    return rows


def parse_target(text: str) -> tuple[float, float]:
    x, y = text.split(",")
    return float(x), float(y)


def _pose(row: dict) -> tuple[float, float, float]:
    return row["robot_x"], row["robot_y"], math.radians(row["robot_theta_deg"])


def nearest_target(
    xy: Sequence[float], targets: Sequence[Sequence[float]]
) -> tuple[int, float] | None:
    """가장 가까운 실제 사과의 (번호, 거리). 목록이 비면 None."""
    if not targets:
        return None
    dists = [math.dist(xy, t) for t in targets]
    i = min(range(len(dists)), key=dists.__getitem__)
    return i, dists[i]


def analyze_targets(
    rows: list[dict],
    targets: Sequence[Sequence[float]],
    diameter: float = config.TARGET_DIAMETER,
    match_radius: float = MATCH_RADIUS,
) -> list[dict]:
    """target 행별 거리 오차와 역산 지름. 실제 사과와 대응되지 않으면 matched=False."""
    results = []
    for row in rows:
        if row.get("kind") != "target" or row.get("dist") is None:
            continue
        est_xy = (row["world_x"], row["world_y"])
        near = nearest_target(est_xy, targets)
        item = {"row": row, "matched": False}
        if near is not None and near[1] <= match_radius:
            index, world_err = near
            true_dist = math.dist((row["robot_x"], row["robot_y"]), targets[index])
            item.update(
                matched=True,
                target=index + 1,
                true_dist=true_dist,
                err=row["dist"] - true_dist,
                world_err=world_err,
                # 추정 거리는 지름에 비례하므로 실제 거리와 맞추는 지름을 역산
                implied_diameter=diameter * true_dist / row["dist"],
            )
        results.append(item)
    return results


def classify_raw(
    rows: list[dict],
    targets: Sequence[Sequence[float]],
    match_radius: float = MATCH_RADIUS,
) -> list[dict]:
    """yolo_raw 행을 실제 사과 상자(is_target)와 그 밖의 상자로 분류."""
    results = []
    for row in rows:
        if row.get("kind") != "yolo_raw" or row.get("w") is None:
            continue
        bearing = bearing_from_cx(row["cx"], IMAGE_WIDTH)
        dist = distance_from_size(max(row["w"], row["h"]), bearing, IMAGE_WIDTH)
        is_target = False
        if dist is not None:
            near = nearest_target(to_world({"dist": dist, "bearing": bearing}, _pose(row)), targets)
            is_target = near is not None and near[1] <= match_radius
        results.append({"row": row, "is_target": is_target})
    return results


def suggest_ratio_min(raw: list[dict]) -> float | None:
    """사과 상자 색 비율 최솟값과 그 밖의 상자 최댓값의 중간. 두 분포가 겹치면 None."""
    hits = [r["row"]["color_ratio"] for r in raw if r["is_target"]]
    others = [r["row"]["color_ratio"] for r in raw if not r["is_target"]]
    hits = [v for v in hits if v is not None]
    others = [v for v in others if v is not None]
    if not hits:
        return None
    if not others:
        return min(hits) / 2
    if min(hits) <= max(others):
        return None
    return (min(hits) + max(others)) / 2


def _stats(values: list[float]) -> str:
    if not values:
        return "-"
    return f"{min(values):.2f} / {statistics.median(values):.2f} / {max(values):.2f}"


def report(rows: list[dict], targets: Sequence[Sequence[float]]) -> str:
    """분석 결과 Markdown 문자열."""
    out = [f"# 측정 분석 ({len(rows)}행)", ""]

    target_results = analyze_targets(rows, targets)
    matched = [r for r in target_results if r["matched"]]
    out += [
        "## 대상 검출 거리 오차",
        "",
        "| note | source | cls | 사과 | 추정 d [m] | 실제 d [m] | 오차 [m] "
        "| 위치 오차 [m] | 역산 지름 [m] |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in target_results:
        row = r["row"]
        if r["matched"]:
            out.append(
                f"| {row['note']} | {row['source']} | {row['cls_name'] or '-'} | {r['target']} "
                f"| {row['dist']:.2f} | {r['true_dist']:.2f} | {r['err']:+.2f} "
                f"| {r['world_err']:.2f} | {r['implied_diameter']:.3f} |"
            )
        else:
            out.append(
                f"| {row['note']} | {row['source']} | {row['cls_name'] or '-'} | 오검출 "
                f"| {row['dist']:.2f} | - | - | - | - |"
            )
    false_hits = len(target_results) - len(matched)
    out += ["", f"- 대상 검출 {len(target_results)}건, 실제 사과 대응 {len(matched)}건, "
            f"대응 없음(오검출 후보) {false_hits}건"]  # fmt: skip
    for lo, hi in DIST_BANDS:
        band = [abs(r["err"]) for r in matched if lo <= r["true_dist"] < hi]
        if band:
            out.append(
                f"- 실제 거리 {lo:.1f}~{hi:.1f} m: 평균 절대 오차 {statistics.mean(band):.2f} m "
                f"({len(band)}건)"
            )
    diameters = [r["implied_diameter"] for r in matched]
    if diameters:
        out.append(
            f"- 역산 지름 중앙값 {statistics.median(diameters):.3f} m "
            f"(현재 TARGET_DIAMETER {config.TARGET_DIAMETER})"
        )

    raw = classify_raw(rows, targets)
    out += [
        "",
        "## YOLO 원본 상자",
        "",
        "| 구분 | cls | 개수 | 신뢰도 최소/중앙/최대 | 색 비율 최소/중앙/최대 |",
        "|---|---|---|---|---|",
    ]
    groups: dict[tuple[str, str], list[dict]] = {}
    for r in raw:
        label = "실제 사과" if r["is_target"] else "그 밖"
        groups.setdefault((label, r["row"]["cls_name"] or "-"), []).append(r["row"])
    for (label, cls_name), items in sorted(groups.items()):
        confs = [i["conf"] for i in items if i["conf"] is not None]
        ratios = [i["color_ratio"] for i in items if i["color_ratio"] is not None]
        out.append(f"| {label} | {cls_name} | {len(items)} | {_stats(confs)} | {_stats(ratios)} |")
    suggestion = suggest_ratio_min(raw)
    out.append("")
    if suggestion is None:
        out.append(
            "- COLOR_RATIO_MIN 추천값 없음 (표본 부족 또는 분포 겹침). "
            f"현재 {config.COLOR_RATIO_MIN}"
        )
    else:
        out.append(
            f"- COLOR_RATIO_MIN 추천값 {suggestion:.2f} (현재 {config.COLOR_RATIO_MIN}). "
            "실제 사과 상자 최솟값과 그 밖 상자 최댓값의 중간"
        )

    times = sorted({(r["sim_t"], r["yolo_ms"]) for r in rows if r.get("yolo_ms") is not None})
    ms = [t for _, t in times]
    out += ["", "## YOLO 추론 시간", ""]
    out.append(f"- 최소/중앙/최대 [ms]: {_stats(ms)} ({len(ms)}회)" if ms else "- 기록 없음")
    return "\n".join(out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv_path")
    parser.add_argument(
        "--target",
        dest="targets",
        action="append",
        type=parse_target,
        required=True,
        metavar="X,Y",
        help="실제 사과 월드 좌표 [m]. 사과마다 반복. 예: --target=-12.02,-3.02",
    )
    parser.add_argument("--out", help="결과 Markdown 저장 경로. 없으면 화면 출력")
    args = parser.parse_args()
    text = report(load_rows(args.csv_path), args.targets)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print(f"저장: {args.out}")
    else:
        print(text)


if __name__ == "__main__":
    main()
