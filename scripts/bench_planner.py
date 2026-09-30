"""지도·경로 계획 성능 측정 (Webots 불필요, 고정 시드 합성 지도).

측정 방식: 항목마다 워밍업 후 반복 실행해 중앙값·95% 지연·최댓값 [ms]을 기록한다.
경로 결과는 길이, 실제 벽까지 최소 거리, 팽창 영역 진입 점 수를 함께 기록한다.

사용법 (저장소 루트):
    .venv/bin/python scripts/bench_planner.py                 # 현재 코드
    .venv/bin/python scripts/bench_planner.py --code <폴더>   # 다른 버전의 sar 패키지 상위 폴더
    .venv/bin/python scripts/bench_planner.py --json out.json

다른 버전과 비교할 때는 같은 PC에서 연속으로 실행한다. 예:
    git archive origin/main controllers/sar_main/sar | tar -x -C /tmp/base
    .venv/bin/python scripts/bench_planner.py --code /tmp/base/controllers/sar_main
"""

import argparse
import json
import math
import statistics
import sys
import time
from itertools import pairwise
from pathlib import Path

import cv2
import numpy as np

SEED = 20260930
REPEAT = 15
WARMUP = 2


def load(code_dir: str | None):
    root = code_dir or str(Path(__file__).resolve().parents[1] / "controllers" / "sar_main")
    sys.path.insert(0, root)
    from sar import config, grid_map, planner

    return config, grid_map, planner


def invalidate(gm) -> None:
    """지도 배열을 직접 바꾼 뒤 계획용 캐시 무효화 (버전별 이름 차이 흡수)."""
    if hasattr(gm, "invalidate"):
        gm.invalidate()
    else:
        gm._cache.clear()


def cell_rect(gm, x0, y0, x1, y1):
    ra, ca = gm.to_cell(min(x0, x1), min(y0, y1))
    rb, cb = gm.to_cell(max(x0, x1), max(y0, y1))
    return slice(ra, rb + 1), slice(ca, cb + 1)


def apartment(gm, rng) -> np.ndarray:
    """12 m x 12 m, 방 9개, 문 폭 0.7 m, 출입구 없는 방 1개, 무작위 장애물 30개."""
    x0, y0 = gm.to_world(gm.n // 2, gm.n // 2)
    x0, y0 = x0 - 6.0, y0 - 6.0
    wall = np.zeros((gm.n, gm.n), bool)
    for k in range(4):
        wall[cell_rect(gm, x0 + 4.0 * k, y0, x0 + 4.0 * k, y0 + 12.0)] = True
        wall[cell_rect(gm, x0, y0 + 4.0 * k, x0 + 12.0, y0 + 4.0 * k)] = True
    for i in (1, 2):  # 내부 벽마다 방 하나당 문 1개
        for j in range(3):
            c = y0 + 4.0 * j + 2.0
            wall[cell_rect(gm, x0 + 4.0 * i, c - 0.35, x0 + 4.0 * i, c + 0.35)] = False
            c = x0 + 4.0 * j + 2.0
            wall[cell_rect(gm, c - 0.35, y0 + 4.0 * i, c + 0.35, y0 + 4.0 * i)] = False
    # 오른쪽 위 방(8~12, 8~12)의 문 2개를 닫아 도달 불가 영역 생성
    wall[cell_rect(gm, x0 + 8.0, y0 + 8.0, x0 + 8.0, y0 + 12.0)] = True
    wall[cell_rect(gm, x0 + 8.0, y0 + 8.0, x0 + 12.0, y0 + 8.0)] = True
    sr, sc = gm.n // 2, gm.n // 2
    for _ in range(30):
        x, y = x0 + rng.uniform(0.4, 11.3), y0 + rng.uniform(0.4, 11.3)
        rs, cs = cell_rect(gm, x, y, x + 0.3, y + 0.3)
        if abs(rs.start - sr) > 12 or abs(cs.start - sc) > 12:
            wall[rs, cs] = True
    return wall


def two_rooms(gm) -> np.ndarray:
    cx, cy = gm.to_world(gm.n // 2, gm.n // 2)
    wall = np.zeros((gm.n, gm.n), bool)
    wall[cell_rect(gm, cx - 3, cy - 2, cx + 3, cy - 2)] = True
    wall[cell_rect(gm, cx - 3, cy + 2, cx + 3, cy + 2)] = True
    wall[cell_rect(gm, cx - 3, cy - 2, cx - 3, cy + 2)] = True
    wall[cell_rect(gm, cx + 3, cy - 2, cx + 3, cy + 2)] = True
    wall[cell_rect(gm, cx, cy - 2, cx, cy + 2)] = True
    wall[cell_rect(gm, cx, cy - 0.4, cx, cy + 0.4)] = False
    return wall


def set_known(gm, config, wall, known=None) -> None:
    gm.seen[:] = True if known is None else known
    gm.logodds[:] = config.L_MIN
    gm.logodds[wall] = config.L_MAX
    gm.logodds[~gm.seen] = 0.0
    invalidate(gm)


def scan(gm, config, wall, pose, count=360) -> list[float]:
    """벽 격자에 대한 라이다 거리 (numpy 광선 추적). 반사 없으면 inf."""
    x, y, theta = pose
    ang = theta + math.pi - np.arange(count) * (2 * math.pi / count)
    t = np.arange(config.LIDAR_MIN, config.LIDAR_MAX, gm.res / 4)
    rows = np.floor((y + t[None, :] * np.sin(ang)[:, None] - gm.y0) / gm.res).astype(int)
    cols = np.floor((x + t[None, :] * np.cos(ang)[:, None] - gm.x0) / gm.res).astype(int)
    rows, cols = np.clip(rows, 0, gm.n - 1), np.clip(cols, 0, gm.n - 1)
    hit = wall[rows, cols]
    first = hit.argmax(axis=1)
    return np.where(hit.any(axis=1), t[first], np.inf).tolist()


def timed(fn, repeat=REPEAT, warmup=WARMUP, setup=None) -> dict:
    out = None
    samples = []
    for i in range(warmup + repeat):
        if setup is not None:
            setup()
        t0 = time.perf_counter()
        out = fn()
        dt = (time.perf_counter() - t0) * 1000
        if i >= warmup:
            samples.append(dt)
    samples.sort()
    return {
        "median": statistics.median(samples),
        "p95": samples[min(len(samples) - 1, int(math.ceil(0.95 * len(samples))) - 1)],
        "max": samples[-1],
        "result": out,
    }


def path_stats(gm, config, wall, path) -> dict:
    if not path:
        return {"points": 0, "length": None, "min_wall": None, "in_inflation": None}
    dist = cv2.distanceTransform((~wall).astype(np.uint8), cv2.DIST_L2, 5) * gm.res
    cells = [gm.to_cell(x, y) for x, y in path]
    clear = [float(dist[c]) for c in cells]
    return {
        "points": len(path),
        "length": round(sum(math.dist(a, b) for a, b in pairwise(path)), 3),
        "min_wall": round(min(clear), 3),
        "in_inflation": sum(1 for c in clear[1:-1] if c < config.INFLATE - gm.res),
    }


def run(code_dir: str | None) -> dict:
    config, grid_map, planner = load(code_dir)
    rng = np.random.default_rng(SEED)
    rows = {}

    def record(name, stat, gm=None, wall=None, path_of=None):
        entry = {k: round(stat[k], 3) for k in ("median", "p95", "max")}
        if path_of is not None:
            entry["path"] = path_stats(gm, config, wall, path_of(stat["result"]))
        rows[name] = entry

    has_pair = hasattr(planner, "choose_frontier_path")

    # 작은 지도 (8 m, 방 2개)
    small = grid_map.GridMap(0.0, 0.0, size=8.0)
    wall_s = two_rooms(small)
    set_known(small, config, wall_s)
    a, b = (-2.5, -1.5), (2.5, 1.5)
    record(
        "small.plan.cold",
        timed(lambda: planner.plan(small, a, b), setup=lambda: invalidate(small)),
        small,
        wall_s,
        lambda p: p,
    )
    record("small.plan.warm", timed(lambda: planner.plan(small, a, b)), small, wall_s, lambda p: p)

    # 기본 640 x 640 지도 (방 9개)
    gm = grid_map.GridMap()
    wall = apartment(gm, rng)
    set_known(gm, config, wall)
    cx, cy = gm.to_world(gm.n // 2, gm.n // 2)
    start = (cx, cy)
    far = (cx - 5.0, cy + 5.0)
    closed = (cx + 4.0, cy + 4.0)  # 출입구 없는 방 안

    pose = (cx, cy, 0.0)
    ranges = scan(gm, config, wall, pose)
    record("big.update.static", timed(lambda: gm.update(pose, ranges), repeat=60))
    record(
        "big.update+layers.static", timed(lambda: (gm.update(pose, ranges), gm.layers()), repeat=60)
    )
    set_known(gm, config, wall)

    record(
        "big.plan.cold",
        timed(lambda: planner.plan(gm, start, far), setup=lambda: invalidate(gm)),
        gm,
        wall,
        lambda p: p,
    )
    record("big.plan.warm", timed(lambda: planner.plan(gm, start, far)), gm, wall, lambda p: p)
    record(
        "big.plan.unreachable",
        timed(lambda: planner.plan(gm, start, closed)),
        gm,
        wall,
        lambda p: p,
    )

    # 거리 지도: 생성, 생성 포함 전체, 재사용
    record("big.field.build", timed(lambda: planner.DistanceField(gm, far)))
    record(
        "big.field.build+plan",
        timed(lambda: planner.plan(gm, start, far, field=planner.DistanceField(gm, far))),
        gm,
        wall,
        lambda p: p,
    )
    field = planner.DistanceField(gm, far)
    record(
        "big.plan.with_field",
        timed(lambda: planner.plan(gm, start, far, field=field)),
        gm,
        wall,
        lambda p: p,
    )
    home = planner.DistanceField(gm, start)
    record("big.field.path", timed(lambda: home.path(far)), gm, wall, lambda p: p)

    # 장애물 추가·제거 후 재계획 (문 1개 닫기/열기)
    door = cell_rect(gm, cx - 2.0, cy - 0.35, cx - 2.0, cy + 0.35)
    state = {"closed": False}

    def toggle():
        state["closed"] = not state["closed"]
        gm.logodds[door] = config.L_MAX if state["closed"] else config.L_MIN
        invalidate(gm)

    record(
        "big.replan.door_toggle",
        timed(lambda: planner.plan(gm, start, far), setup=toggle, repeat=16),
        gm,
        wall,
        lambda p: p,
    )
    gm.logodds[door] = config.L_MIN
    invalidate(gm)

    # 프론티어 선택부터 경로 반환까지 (지도 왼쪽 절반만 확인)
    known = np.zeros((gm.n, gm.n), bool)
    known[cell_rect(gm, cx - 6.2, cy - 6.2, cx + 1.0, cy + 6.2)] = True
    set_known(gm, config, wall, known)
    fpose = (cx - 1.0, cy - 1.0, 0.0)

    def select_then_plan():
        target = planner.choose_frontier(gm, fpose, [])
        return planner.plan(gm, fpose[:2], target) if target else None

    record("big.frontier.select", timed(lambda: planner.choose_frontier(gm, fpose, [])))
    record("big.frontier.select+plan", timed(select_then_plan), gm, wall, lambda p: p)
    if has_pair:
        record(
            "big.frontier.select_with_path",
            timed(lambda: planner.choose_frontier_path(gm, fpose, [])),
            gm,
            wall,
            lambda r: r[1] if r else None,
        )

    # 탐색 영역 확장: 고정 자세 순서로 스캔을 넣으며 매번 목표 선택과 경로 계산
    route = [
        (cx, cy),
        (cx, cy + 2.0),
        (cx, cy + 4.0),
        (cx, cy + 6.0 - 0.6),
        (cx - 2.0, cy + 2.0),
        (cx - 4.0, cy + 2.0),
        (cx - 4.0, cy),
        (cx - 4.0, cy - 2.0),
        (cx - 2.0, cy - 2.0),
        (cx, cy - 2.0),
        (cx, cy - 4.0),
        (cx + 2.0, cy - 2.0),
        (cx + 4.0, cy - 2.0),
        (cx + 4.0, cy),
        (cx + 2.0, cy + 2.0),
        (cx + 2.0, cy),
    ]
    scans = [(p + (0.0,), scan(gm, config, wall, p + (0.0,))) for p in route]

    def explore():
        m = grid_map.GridMap()
        per_step = []
        found = 0
        for p, r in scans:
            for _ in range(3):
                m.update(p, r)
            t0 = time.perf_counter()
            if has_pair:
                res = planner.choose_frontier_path(m, p, [])
                path = res[1] if res else None
            else:
                target = planner.choose_frontier(m, p, [])
                path = planner.plan(m, p[:2], target) if target else None
            per_step.append((time.perf_counter() - t0) * 1000)
            found += path is not None
        return per_step, found

    totals, steps, found = [], [], 0
    for i in range(WARMUP + 8):
        per_step, found = explore()
        if i >= WARMUP:
            totals.append(sum(per_step))
            steps.extend(per_step)
    steps.sort()
    rows["big.explore.per_selection"] = {
        "median": round(statistics.median(steps), 3),
        "p95": round(steps[int(math.ceil(0.95 * len(steps))) - 1], 3),
        "max": round(steps[-1], 3),
        "paths_found": f"{found}/{len(route)}",
    }
    rows["big.explore.total_16_selections"] = {
        "median": round(statistics.median(totals), 3),
        "p95": round(sorted(totals)[-1], 3),
        "max": round(max(totals), 3),
    }
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "--code", help="sar 패키지가 있는 폴더 (기본: 이 저장소의 controllers/sar_main)"
    )
    parser.add_argument("--json", help="결과 JSON 저장 경로")
    args = parser.parse_args()
    rows = run(args.code)
    print(
        f"{'항목':<34}{'중앙값':>9}{'p95':>9}{'최대':>9}  경로 [길이 m / 벽 최소 m / 팽창 진입 점]"
    )
    for name, r in rows.items():
        extra = ""
        if "path" in r:
            p = r["path"]
            extra = f"{p['length']} / {p['min_wall']} / {p['in_inflation']}"
        elif "paths_found" in r:
            extra = f"경로 {r['paths_found']}"
        print(f"{name:<34}{r['median']:>9.2f}{r['p95']:>9.2f}{r['max']:>9.2f}  {extra}")
    if args.json:
        Path(args.json).write_text(json.dumps(rows, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
