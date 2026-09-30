"""설계 근거 정량 검증. sar_eval 기록과 합성 계산으로 조건별 지표를 계산하고 그래프를 저장한다.

입력: controllers/sar_eval/output/<mode>/ (run.npz, frames/, detections.csv)
- mission: 현재 코드 그대로 실행한 기록
- truth: 실제 pose를 미션에 입력한 기록 (위치 추정 오차 제거 조건)
- truth_explore: truth + 전체 탐색. 기준 지도와 인식 프레임 (없으면 truth 사용)

사용법 (저장소 루트):
    .venv/bin/python scripts/design_eval.py                  # 전체 실험
    .venv/bin/python scripts/design_eval.py --only L,P1,C3   # 일부 실험
결과: <out>/results.json, <out>/design_*.png. 같은 기록에서 같은 값을 출력한다
(계산 시간 항목만 PC 부하에 따라 달라짐).

실험 번호는 docs 저장소 human/explanation/sar-시스템-설계.md의 표 번호와 같다.
"""

import argparse
import concurrent.futures
import contextlib
import json
import math
import re
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "sar_main"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from sar import config, local_control, planner  # noqa: E402
from sar.grid_map import GridMap, simulate_scan  # noqa: E402
from sar.local_control import pure_pursuit, safety_filter  # noqa: E402
from sar.mission import compass_angle, compass_heading, fit_compass  # noqa: E402
from sar.odometry import Odometry, wrap  # noqa: E402
from sar.perception import Confirm  # noqa: E402

DT = 0.064  # s, basicTimeStep
MATCH_RADIUS = 0.5  # m, 검출 월드 좌표와 실제 물체 위치의 일치 반경
VIS_RANGE = 4.0  # m, 라벨 대상 거리 상한
DIST_BINS = [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0)]
FLOOR_OBJECTS = (
    "RedApple",
    "GreenApple",
    "PurpleApple",
    "OrangeApple",
    "FireExtinguisher",
    "FifaSoccerBall",
    "Can",
    "BeerBottle",
    "Cat",
)
# 가시선 라벨을 이미지로 확인한 결과 사과가 가려진 프레임 (실행, step). 33장 중 2장
MANUAL_NOT_VISIBLE = {("truth", 1104), ("truth", 2428)}
COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]


# 공통


@contextlib.contextmanager
def patched(obj=config, **values):
    """모듈 속성을 잠시 바꾼 뒤 복원."""
    old = {k: getattr(obj, k) for k in values}
    for k, v in values.items():
        setattr(obj, k, v)
    try:
        yield
    finally:
        for k, v in old.items():
            setattr(obj, k, v)


def load_run(folder: Path) -> dict | None:
    """sar_eval 기록. 없으면 None."""
    path = folder / "run.npz"
    if not path.exists():
        return None
    data = dict(np.load(path))
    data["folder"] = folder
    frames = sorted(int(p.stem) for p in (folder / "frames").glob("*.jpg"))
    data["frames"] = frames
    return data


def world_objects(path: Path = ROOT / "worlds" / "apartment.wbt") -> list[tuple[str, float, float]]:
    """월드 최상위 노드 중 바닥 물체 (이름, x, y). 식탁 위 물체(z > 0.3 m)는 제외."""
    text = path.read_text(encoding="utf-8")
    objects = []
    pattern = r"^(\w+) \{\n(?:  .*\n)*?  translation (\S+) (\S+) (\S+)"
    for m in re.finditer(pattern, text, re.M):
        if m[1] in FLOOR_OBJECTS and float(m[4]) < 0.3:
            objects.append((m[1], float(m[2]), float(m[3])))
    return objects


def red_apples(objects) -> list[tuple[float, float]]:
    return [(x, y) for name, x, y in objects if name == "RedApple"]


def build_map(run: dict, poses: np.ndarray, stride: int = 1) -> GridMap:
    """기록한 라이다와 주어진 pose 목록으로 점유 격자 작성."""
    grid = GridMap()
    for ranges, pose in zip(run["lidar"][::stride], poses[::stride], strict=True):
        if not np.isnan(ranges[0]):
            grid.update(tuple(float(v) for v in pose), ranges.tolist())
    return grid


def path_clearance(grid: GridMap, path, step: float = 0.02) -> float:
    """경로 위 점에서 가장 가까운 장애물까지 거리 최솟값 [m]."""
    clear = grid.clearance()
    best = math.inf
    for (x0, y0), (x1, y1) in zip(path, path[1:], strict=False):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        for k in range(n + 1):
            r, c = grid.to_cell(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
            if grid.inside(r, c):
                best = min(best, float(clear[r, c]))
    return best


def turn_count(path, min_angle: float = math.radians(10)) -> int:
    """경로에서 min_angle보다 큰 방향 전환 수."""
    count = 0
    for a, b, c in zip(path, path[1:], path[2:], strict=False):
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        count += abs(wrap(h2 - h1)) > min_angle
    return count


def summary(values) -> dict:
    vals = [v for v in values if v is not None and not math.isnan(v)]
    if not vals:
        return {"n": 0}
    arr = np.asarray(vals)
    return {
        "n": len(vals),
        "median": float(np.median(arr)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(arr.max()),
    }


def save(fig, out: Path, name: str) -> None:
    fig.tight_layout()
    fig.savefig(out / name, dpi=120)
    plt.close(fig)


# L: 위치 추정


def replay_odometry(run: dict, variant: str) -> np.ndarray:
    """기록한 엔코더·나침반으로 위치 추정을 다시 계산한 pose 배열.

    variant: enc (엔코더만), enc_scale (+ 회전 스케일), kf (+ 나침반 KF), kf_scale (현재),
    compass (나침반 방향 직접 사용). 나침반 보정·스케일은 미션과 같이 INIT_SPIN 종료 다음
    step부터 적용한다.
    """
    enc, vec, state = run["enc"], run["compass"], run["state"]
    k_end = int(np.argmax(state != 0))  # INIT_SPIN이 끝난 tick
    odom = Odometry(config.START_X, config.START_Y, config.START_THETA)
    samples, fit, poses = [], None, []
    use_compass = variant in ("kf", "kf_scale", "compass")
    r_value = 1e-12 if variant == "compass" else config.HEADING_R
    with patched(HEADING_R=r_value):
        for i in range(len(enc)):
            heading = None
            if use_compass and fit is not None and not np.isnan(vec[i][0]):
                heading = compass_heading(vec[i], fit[0], fit[1])
            odom.update(float(enc[i][0]), float(enc[i][1]), compass=heading)
            if i <= k_end and not np.isnan(vec[i][0]):
                samples.append((odom.pose()[2], compass_angle(vec[i])))
            if i == k_end:
                fit = fit_compass(samples)
                if fit is not None and variant in ("enc_scale", "kf_scale"):
                    odom.set_wheel_separation_scale(fit[2])
            poses.append(odom.pose())
    return np.asarray(poses)


LOC_VARIANTS = {
    "enc": "encoder only",
    "enc_scale": "encoder + scale (#16)",
    "kf": "encoder + compass KF (#13)",
    "kf_scale": "KF + scale (current)",
    "compass": "compass heading direct",
}


def loc_errors(run: dict, poses: np.ndarray) -> dict:
    gt = run["gt"]
    err = np.hypot(poses[:, 0] - gt[:, 0], poses[:, 1] - gt[:, 1])
    head = np.abs(np.arctan2(np.sin(poses[:, 2] - gt[:, 2]), np.cos(poses[:, 2] - gt[:, 2])))
    t = run["t"]
    at = {f"t{int(s)}": float(err[min(np.searchsorted(t, s), len(t) - 1)]) for s in (100, 165, 185)}
    return {
        "final": float(err[-1]),
        "max": float(err.max()),
        "rms": float(np.sqrt((err**2).mean())),
        "heading_rms": float(np.sqrt((head**2).mean())),
        **at,
        "err": err,
    }


def slip_episodes(run: dict) -> dict:
    """엔코더 직진 이동량 대비 실제 이동량이 절반 미만인 step (바퀴 공회전). 제자리 회전 제외."""
    enc, gt, t = run["enc"], run["gt"], run["t"]
    denc = np.abs(np.diff(enc, axis=0).mean(axis=1)) * config.WHEEL_RADIUS
    dgt = np.hypot(*np.diff(gt[:, :2], axis=0).T)
    slip = (denc > 0.005) & (dgt < 0.5 * denc)
    idx = np.nonzero(slip)[0]
    if len(idx) == 0:
        return {"steps": 0}
    return {
        "steps": int(slip.sum()),
        "seconds": float(slip.sum() * DT),
        "lost_m": float((denc[slip] - dgt[slip]).sum()),
        "t_first": float(t[idx[0]]),
        "t_last": float(t[idx[-1]]),
        "x": float(gt[idx[0], 0]),
        "y": float(gt[idx[0], 1]),
    }


def carpet_rect(
    path: Path = ROOT / "worlds" / "apartment.wbt",
) -> tuple[float, float, float, float]:
    """Carpet 노드의 바닥 영역 (xmin, ymin, xmax, ymax). PROTO 충돌 상자 2.4 x 1.6 m."""
    text = path.read_text(encoding="utf-8")
    m = re.search(r"^Carpet \{\n  translation (\S+) (\S+) \S+\n  rotation 0 0 1 (\S+)", text, re.M)
    cx, cy, rot = float(m[1]), float(m[2]), float(m[3])
    hx, hy = (1.2, 0.8) if abs(math.sin(rot)) < 0.5 else (0.8, 1.2)
    return cx - hx, cy - hy, cx + hx, cy + hy


def edge_error(run: dict, poses: np.ndarray, band: float = 0.2) -> dict:
    """step별 이동 증분 오차 |Δ실제 − Δ추정|를 카펫 경계 ±band 구간과 나머지로 분리."""
    gt = run["gt"][:, :2]
    inc = np.hypot(*(np.diff(gt, axis=0) - np.diff(poses[:, :2], axis=0)).T)
    x0, y0, x1, y1 = carpet_rect()
    x, y = gt[:-1, 0], gt[:-1, 1]
    dx = np.maximum.reduce([x0 - x, x - x1, np.zeros_like(x)])
    dy = np.maximum.reduce([y0 - y, y - y1, np.zeros_like(y)])
    outside = np.hypot(dx, dy)
    inside = np.minimum.reduce([x - x0, x1 - x, y - y0, y1 - y])
    near = np.where(outside > 0, outside, inside) < band
    return {
        "total_m": float(inc.sum()),
        "carpet_edge_m": float(inc[near].sum()),
        "carpet_edge_share": float(inc[near].sum() / max(inc.sum(), 1e-9)),
        "carpet_edge_time_s": float(near.sum() * DT),
        "carpet_rect": [x0, y0, x1, y1],
    }


def exp_localization(runs: dict, out: Path) -> dict:
    result = {}
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
    for ax, key in zip(axes, ("mission", "truth"), strict=True):
        run = runs.get(key)
        if run is None:
            continue
        res = {"slip": slip_episodes(run)}
        for i, (name, label) in enumerate(LOC_VARIANTS.items()):
            poses = replay_odometry(run, name)
            errs = loc_errors(run, poses)
            ax.plot(run["t"], errs.pop("err"), color=COLORS[i], lw=1.2, label=label)
            res[name] = errs
        res["edge_error"] = edge_error(run, replay_odometry(run, "kf_scale"))
        if key == "mission":
            res["replay_matches_log"] = float(
                np.abs(replay_odometry(run, "kf_scale") - run["est"]).max()
            )
        ax.axhline(0.3, color="k", ls="--", lw=0.8)
        ax.set_yscale("log")
        ax.set_ylim(0.005, 20)
        ax.set_title(f"run: {key} (driven by {'odometry' if key == 'mission' else 'true pose'})")
        ax.set_xlabel("sim time [s]")
        result[key] = res
    axes[0].set_ylabel("position error [m] (log, dashed: 0.3 m)")
    axes[0].legend(fontsize=7, loc="upper left")
    save(fig, out, "design_localization_error.png")
    return result


def f1_occupied(ref: GridMap, test: GridMap) -> dict:
    """장애물 칸 F1 (1칸 허용 오차)."""
    kernel = np.ones((3, 3), np.uint8)
    a, b = ref.occupied(), test.occupied()
    a_d = cv2.dilate(a.astype(np.uint8), kernel).astype(bool)
    b_d = cv2.dilate(b.astype(np.uint8), kernel).astype(bool)
    precision = float((b & a_d).sum() / max(1, b.sum()))
    recall = float((a & b_d).sum() / max(1, a.sum()))
    f1 = 2 * precision * recall / max(1e-9, precision + recall)
    return {"precision": precision, "recall": recall, "f1": f1, "cells": int(b.sum())}


def exp_map(runs: dict, out: Path) -> dict:
    run = runs["mission"]
    ref = build_map(run, run["gt"])
    result = {}
    maps = {"true pose": ref}
    for name in LOC_VARIANTS:
        grid = build_map(run, replay_odometry(run, name))
        result[name] = f1_occupied(ref, grid)
        if name in ("enc", "kf_scale"):
            maps[LOC_VARIANTS[name]] = grid
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.8))
    seen = np.logical_or.reduce([g.seen for g in maps.values()])
    rows, cols = np.nonzero(seen)
    box = (rows.min() - 10, rows.max() + 10, cols.min() - 10, cols.max() + 10)
    for ax, (title, grid) in zip(axes, maps.items(), strict=True):
        img = np.full(grid.logodds.shape, 0.6)
        img[grid.seen] = 1.0
        img[grid.occupied()] = 0.0
        ax.imshow(
            img[box[0] : box[1], box[2] : box[3]], cmap="gray", origin="lower", vmin=0, vmax=1
        )
        f1 = "" if grid is ref else f"  F1={f1_occupied(ref, grid)['f1']:.2f}"
        ax.set_title(title + f1, fontsize=9)
        ax.set_xticks([])
        ax.set_yticks([])
    save(fig, out, "design_map_compare.png")
    return result


# P: 지도·경로·탐색


def clear_steps(hits: int, clamp: bool) -> int:
    """칸이 hits회 연속 장애물로 관측된 뒤 빈칸으로 돌아가는 데 필요한 빈칸 관측 수."""
    values = {} if clamp else {"L_MIN": -1e9, "L_MAX": 1e9}
    with patched(**values):
        grid = GridMap(0.0, 0.0, size=4.0)
        hit = [math.inf] * 360
        hit[180] = 1.0
        for _ in range(hits):
            grid.update((0.0, 0.0, 0.0), hit)
        cell = grid.to_cell(1.0 + grid.res / 2, 0.0)
        free = [math.inf] * 360
        for n in range(1, 10000):
            grid.update((0.0, 0.0, 0.0), free)
            if grid.logodds[cell] <= config.OCC_THRESHOLD:
                return n
    return -1


def exp_logodds(out: Path) -> dict:
    hits = list(range(1, 161))
    clamp = [clear_steps(k, True) for k in hits]
    free = [clear_steps(k, False) for k in hits]
    fig, ax = plt.subplots(figsize=(6, 3.4))
    occ_t = np.asarray(hits) * DT
    ax.plot(occ_t, np.asarray(free) * DT, label="no clamp", color=COLORS[3])
    ax.plot(occ_t, np.asarray(clamp) * DT, label=f"clamp L_MAX={config.L_MAX}", color=COLORS[0])
    ax.axvline(0.3 / 0.2, color="k", ls=":", lw=0.8)
    ax.text(0.3 / 0.2 + 0.1, max(free) * DT * 0.9, "pedestrian 0.3 m @ 0.2 m/s", fontsize=7)
    ax.set_xlabel("time cell observed occupied [s]")
    ax.set_ylabel("time to clear [s]")
    ax.legend(fontsize=8)
    save(fig, out, "design_logodds_clear.png")
    pick = {
        f"{k * DT:.1f}s": {"clamp": clamp[k - 1] * DT, "no_clamp": free[k - 1] * DT}
        for k in (23, 78, 156)
    }
    return {"clear_time_s": pick, "clamp_max_s": max(clamp) * DT}


def reference(runs: dict) -> tuple[dict, GridMap]:
    run = runs.get("truth_explore") or runs["truth"]
    return run, build_map(run, run["gt"])


def count_search():
    """planner._search 호출마다 도달한 칸 수를 기록하는 래퍼 설치."""
    counts: list[int] = []
    original = planner._search

    def wrapper(win, source, goal=None, heuristic=None, on_pop=None):
        dist, parent = original(win, source, goal, heuristic, on_pop)
        counts.append(sum(1 for d in dist if d < math.inf))
        return dist, parent

    planner._search = wrapper
    return counts, original


def timed(fn, repeat: int = 5):
    fn()  # 캐시 준비
    times = []
    result = None
    for _ in range(repeat):
        start = time.perf_counter()
        result = fn()
        times.append((time.perf_counter() - start) * 1000)
    return result, statistics.median(times)


def exp_planner(runs: dict, out: Path) -> dict:
    _, grid = reference(runs)
    start = (config.START_X, config.START_Y)
    goals = red_apples(world_objects())
    result = {}
    counts, original = count_search()
    try:
        for gi, goal in enumerate(goals, start=1):
            res = {}
            path, ms = timed(lambda g=goal: planner.plan(grid, start, g))
            counts.clear()
            planner.plan(grid, start, goal)
            res["astar"] = {"ms": ms, "cells": counts[-1], "length": planner.path_length(path)}

            def dijkstra(g=goal):
                return planner.DistanceField(grid, g).path(start)

            path, ms = timed(dijkstra)
            counts.clear()
            dijkstra()
            res["dijkstra"] = {"ms": ms, "cells": counts[-1], "length": planner.path_length(path)}
            field, build_ms = timed(lambda g=goal: planner.DistanceField(grid, g))
            path, ms = timed(lambda g=goal, f=field: planner.plan(grid, start, g, field=f))
            counts.clear()
            planner.plan(grid, start, goal, field=field)
            res["astar_field"] = {
                "ms": ms,
                "build_ms": build_ms,
                "cells": counts[-1],
                "length": planner.path_length(path),
            }
            result[f"apple{gi}"] = res
    finally:
        planner._search = original

    # P3 벽 비용, P4 경로 다듬기, P5 팽창 반경
    variants = {
        "WALL_COST=0": {"WALL_COST": 0.0},
        "WALL_COST=2 (current)": {},
        "WALL_COST=4": {"WALL_COST": 4.0},
        "PATH_SMOOTH=False": {"PATH_SMOOTH": False},
        "INFLATE=0.105": {"INFLATE": config.ROBOT_RADIUS},
        "INFLATE=0.25": {"INFLATE": 0.25},
    }
    walls = sim_walls(grid)
    for name, values in variants.items():
        with patched(**values):
            grid.invalidate()
            occ, blocked, _, unknown = grid.layers()
            passable = (~blocked & ~unknown).astype(np.uint8)
            _, labels = cv2.connectedComponents(passable, connectivity=4)
            label = labels[grid.to_cell(*start)]
            area = float((labels == label).sum() * grid.res**2) if label else 0.0
            rows = {"reachable_m2": area}
            for gi, goal in enumerate(goals, start=1):
                path = planner.plan(grid, start, goal)
                if path is None:
                    rows[f"apple{gi}"] = None
                    continue
                drive = follow_path(walls, grid, path, (*start, config.START_THETA))
                rows[f"apple{gi}"] = {
                    "length": planner.path_length(path),
                    "clearance": path_clearance(grid, path),
                    "turns": turn_count(path),
                    **drive,
                }
            result[name] = rows
    grid.invalidate()

    fig, axes = plt.subplots(1, 2, figsize=(9, 3.2))
    names = ["astar", "dijkstra", "astar_field"]
    labels_ = ["A*", "Dijkstra", "A* + field"]
    x = np.arange(len(goals))
    for i, (n, lab) in enumerate(zip(names, labels_, strict=True)):
        ms = [result[f"apple{g}"][n]["ms"] for g in range(1, len(goals) + 1)]
        cells = [result[f"apple{g}"][n]["cells"] for g in range(1, len(goals) + 1)]
        axes[0].bar(x + i * 0.27, ms, 0.27, label=lab, color=COLORS[i])
        axes[1].bar(x + i * 0.27, cells, 0.27, label=lab, color=COLORS[i])
    for ax, ylab in zip(axes, ("compute time [ms]", "reached cells"), strict=True):
        ax.set_xticks(x + 0.27, [f"start -> apple {g}" for g in range(1, len(goals) + 1)])
        ax.set_ylabel(ylab)
    axes[0].legend(fontsize=8)
    save(fig, out, "design_planner_compare.png")
    return result


# 운동학 시뮬레이션 (C1, C2, P3~P6)


def sim_walls(ref: GridMap) -> np.ndarray:
    """주행 시뮬레이션의 벽 격자. 기준 지도의 장애물 칸 + 확인하지 않은 영역."""
    return ref.occupied() | ~ref.seen


class Sim:
    """오차 없는 차동 구동 로봇. 벽까지 거리는 벽 격자 거리 변환(0.05 m 격자)으로 계산."""

    def __init__(self, walls: np.ndarray, grid: GridMap, pose) -> None:
        self.walls, self.grid = walls, grid
        self.pose = tuple(pose)
        self.t = 0.0
        self.dist = 0.0
        self.min_gap = math.inf
        self.collisions = 0
        self.blocked = 0
        clear = cv2.distanceTransform((~walls).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        self.clear = clear * grid.res - grid.res / 2  # 벽 칸 중심 → 벽 칸 경계

    def scan(self) -> list[float]:
        return simulate_scan(self.walls, self.grid.to_cell, self.pose)

    def step(self, v: float, w: float) -> None:
        x, y, th = self.pose
        mid = th + w * DT / 2
        self.pose = (x + v * math.cos(mid) * DT, y + v * math.sin(mid) * DT, wrap(th + w * DT))
        self.t += DT
        self.dist += abs(v) * DT
        r, c = self.grid.to_cell(*self.pose[:2])
        gap = float(self.clear[r, c]) - config.ROBOT_RADIUS
        self.min_gap = min(self.min_gap, gap)
        self.collisions += gap < 0


def cross_track(path, xy) -> float:
    best = math.inf
    for (x0, y0), (x1, y1) in zip(path, path[1:], strict=False):
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((xy[0] - x0) * dx + (xy[1] - y0) * dy) / L2))
        best = min(best, math.hypot(x0 + u * dx - xy[0], y0 + u * dy - xy[1]))
    return best


def follow_path(walls, grid, path, pose, lookahead=config.LOOKAHEAD, timeout=240.0) -> dict:
    """고정 경로 추종. 도착 시간, 경로 이탈, 최소 거리, 차단·충돌 step 수."""
    sim = Sim(walls, grid, pose)
    errors = []
    reached = False
    while sim.t < timeout:
        ranges = sim.scan()
        v, w, reached = pure_pursuit(sim.pose, path, lookahead)
        if reached:
            break
        v, w, blocked = safety_filter(v, w, ranges)
        sim.blocked += blocked
        sim.step(v, w)
        errors.append(cross_track(path, sim.pose[:2]))
    err = np.asarray(errors) if errors else np.zeros(1)
    return {
        "reached": reached,
        "time": sim.t,
        "track_rms": float(np.sqrt((err**2).mean())),
        "track_max": float(err.max()),
        "min_gap": sim.min_gap,
        "blocked_steps": sim.blocked,
        "collision_steps": sim.collisions,
    }


def exp_control(runs: dict, out: Path) -> dict:
    _, grid = reference(runs)
    walls = sim_walls(grid)
    start = (config.START_X, config.START_Y, config.START_THETA)
    paths = [planner.plan(grid, start[:2], g) for g in red_apples(world_objects())]
    result = {}
    # C2는 경로와 반대 방향(동쪽)을 보고 출발해 큰 방향 전환이 필요한 조건
    reverse = (config.START_X, config.START_Y, 0.0)
    cases = {
        "lookahead=0.20": ({}, 0.20, start),
        "lookahead=0.35 (current)": ({}, 0.35, start),
        "lookahead=0.50": ({}, 0.50, start),
        "reverse start, pivot 55 deg (current)": ({}, 0.35, reverse),
        "reverse start, pivot off": ({"_PIVOT_ANGLE": math.pi}, 0.35, reverse),
    }
    for name, (values, lookahead, pose) in cases.items():
        with patched(local_control, **values):
            result[name] = {
                f"apple{i}": follow_path(walls, grid, p, pose, lookahead)
                for i, p in enumerate(paths, start=1)
                if p is not None
            }
    return result


def safety_run(offset: float, half_angle: float) -> float:
    """정면 1 m, 측면 offset에 5 cm 기둥. 직진하며 안전 필터 적용. 몸체-기둥 최소 거리 [m]."""
    grid = GridMap(0.0, 0.0, size=4.0)
    walls = np.zeros((grid.n, grid.n), bool)
    r, c = grid.to_cell(1.0, offset)
    walls[r, c] = True
    cx, cy = grid.to_world(r, c)
    pose = (0.0, 0.0, 0.0)
    best = math.inf
    with patched(local_control, _FRONT_HALF_ANGLE=half_angle):
        for _ in range(int(10.0 / DT)):
            ranges = simulate_scan(walls, grid.to_cell, pose)
            v, w, _ = safety_filter(config.V_MAX, 0.0, ranges)
            pose = (pose[0] + v * DT, pose[1], 0.0)
            dx = max(abs(cx - pose[0]) - grid.res / 2, 0.0)
            dy = max(abs(cy - pose[1]) - grid.res / 2, 0.0)
            best = min(best, math.hypot(dx, dy) - config.ROBOT_RADIUS)
            if pose[0] > 1.3:
                break
    return best


def exp_safety(out: Path) -> dict:
    old, new = math.radians(25), local_control._FRONT_HALF_ANGLE
    offsets = np.round(np.arange(0.0, 0.2001, 0.005), 3)
    gaps = {
        name: [safety_run(float(o), a) for o in offsets]
        for name, a in (("25deg", old), ("current", new))
    }
    fig, ax = plt.subplots(figsize=(6, 3.2))
    ax.plot(
        offsets, gaps["25deg"], "o-", ms=3, color=COLORS[3], label="half angle 25 deg (before #11)"
    )
    ax.plot(
        offsets,
        gaps["current"],
        "o-",
        ms=3,
        color=COLORS[0],
        label=f"half angle {math.degrees(new):.1f} deg (current)",
    )
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xlabel("obstacle lateral offset [m]")
    ax.set_ylabel("min body gap [m] (<0: collision)")
    ax.legend(fontsize=8)
    save(fig, out, "design_safety_sweep.png")
    return {
        "half_width_at_stop_m": {
            "25deg": config.STOP_DIST * math.tan(old),
            "current": config.STOP_DIST * math.tan(new),
            "robot_radius": config.ROBOT_RADIUS,
        },
        "collisions": {k: int(sum(g < 0 for g in v)) for k, v in gaps.items()},
        "collision_offsets": {
            k: [float(o) for o, g in zip(offsets, v, strict=True) if g < 0] for k, v in gaps.items()
        },
        "offsets_tested": len(offsets),
        "step_move_m": config.V_MAX * DT,
        "margin_steps": (config.STOP_DIST - config.LIDAR_MIN) / (config.V_MAX * DT),
    }


def explore_run(args: tuple) -> dict:
    """프론티어 탐색 운동학 시뮬레이션 1회 (프로세스 풀에서 실행)."""
    ref_path, power, hold, period, t_max, dtheta = args
    data = np.load(ref_path)
    walls, ref_free = data["walls"], data["free"]
    grid = GridMap()
    sizes = grid.frontier_sizes
    grid.frontier_sizes = lambda: np.where(sizes() > 0, sizes().astype(np.int64) ** power, 0)
    sim = Sim(walls, grid, (config.START_X, config.START_Y, config.START_THETA + dtheta))
    target, path = None, None
    goal_t, plan_t, blocked_t = 0.0, -math.inf, None
    blacklist, selections, plan_ms = [], 0, []
    curve, done = [], False
    spin_end = 2 * math.pi / config.INIT_SPIN_W
    next_rec = 0.0
    while sim.t < t_max and not done:
        ranges = sim.scan()
        grid.update(sim.pose, ranges)
        if sim.t >= next_rec:
            curve.append((sim.t, float((grid.seen & ref_free).sum() / ref_free.sum())))
            next_rec += 1.0
        if sim.t < spin_end:
            sim.step(0.0, config.INIT_SPIN_W)
            continue
        if path is None or (not hold and sim.t - plan_t >= period):
            start = time.perf_counter()
            found = planner.choose_frontier_path(grid, sim.pose, blacklist)
            plan_ms.append((time.perf_counter() - start) * 1000)
            if found is None:
                done = True
                break
            new, path = found
            if target is None or math.dist(new, target) > config.BLACKLIST_RADIUS:
                goal_t, selections = sim.t, selections + 1
            target, plan_t = new, sim.t
        elif sim.t - plan_t >= period:
            start = time.perf_counter()
            path = planner.plan(grid, sim.pose[:2], target)
            plan_ms.append((time.perf_counter() - start) * 1000)
            plan_t = sim.t
            if path is None:
                blacklist.append(target)
                target = None
                continue
        if sim.t - goal_t > config.FRONTIER_TIMEOUT:
            blacklist.append(target)
            target, path = None, None
            continue
        v, w, reached = pure_pursuit(sim.pose, path)
        if reached:
            target, path = None, None
        v, w, blocked = safety_filter(v, w, ranges)
        sim.blocked += blocked
        if blocked:
            blocked_t = sim.t if blocked_t is None else blocked_t
            if sim.t - blocked_t > config.BLOCKED_TIME:
                path, blocked_t = None, None
        else:
            blocked_t = None
        sim.step(v, w)
    cov = [c for _, c in curve]
    t90 = next((t for t, c in curve if c >= 0.9), None)
    return {
        "power": power,
        "hold": hold,
        "period": period,
        "dtheta": dtheta,
        "finished": done,
        "time": sim.t,
        "t90": t90,
        "coverage_final": cov[-1] if cov else 0.0,
        "distance": sim.dist,
        "selections": selections,
        "blacklisted": len(blacklist),
        "collision_steps": sim.collisions,
        "blocked_steps": sim.blocked,
        "plan_ms_total": float(sum(plan_ms)),
        "plan_ms_median": float(np.median(plan_ms)) if plan_ms else 0.0,
        "curve": curve,
    }


def exp_explore(runs: dict, out: Path, t_max: float = 600.0) -> dict:
    _, grid = reference(runs)
    ref_path = out / "reference_map.npz"
    np.savez_compressed(ref_path, walls=sim_walls(grid), free=grid.seen & ~grid.occupied())
    cases = [(p, False, 2.0) for p in (0, 1, 2)] + [(1, True, 2.0)]
    cases += [(1, False, period) for period in (0.5, 1.0, 5.0)]
    offsets = (-0.2, 0.0, 0.2)  # rad, 시작 방향 변화로 초기 조건 민감도 확인
    jobs = [(str(ref_path), p, h, per, t_max, d) for p, h, per in cases for d in offsets]
    with concurrent.futures.ProcessPoolExecutor() as pool:
        runs_ = list(pool.map(explore_run, jobs))
    names = {0: "N^0/d (nearest)", 1: "N/d (current)", 2: "N^2/d (size first)"}
    fig, ax = plt.subplots(figsize=(6.5, 3.4))
    for i, (p, h, per) in enumerate(cases[:4]):
        label = names[p] + (", hold target" if h else ", reselect 2 s")
        for j, r in enumerate(
            x for x in runs_ if (x["power"], x["hold"], x["period"]) == (p, h, per)
        ):
            t, c = zip(*r["curve"], strict=True)
            ax.plot(
                t,
                np.asarray(c) * 100,
                color=COLORS[i],
                lw=1,
                alpha=0.8,
                label=label if j == 0 else None,
            )
    ax.axhline(90, color="k", ls=":", lw=0.8)
    ax.set_xlabel("sim time [s]")
    ax.set_ylabel("known free area [%]")
    ax.legend(fontsize=7, loc="lower right")
    save(fig, out, "design_frontier_coverage.png")
    summary_rows = []
    for p, h, per in cases:
        group = [x for x in runs_ if (x["power"], x["hold"], x["period"]) == (p, h, per)]
        t90 = [x["t90"] for x in group]
        summary_rows.append(
            {
                "power": p,
                "hold": h,
                "period": per,
                "t90": t90,
                "t90_median": float(np.median([v if v is not None else t_max for v in t90])),
                "reached_90": sum(v is not None for v in t90),
                "coverage_final_mean": float(np.mean([x["coverage_final"] for x in group])),
                "distance_mean": float(np.mean([x["distance"] for x in group])),
                "selections_mean": float(np.mean([x["selections"] for x in group])),
                "blocked_steps_mean": float(np.mean([x["blocked_steps"] for x in group])),
                "collision_steps": int(sum(x["collision_steps"] for x in group)),
                "plan_ms_total_mean": float(np.mean([x["plan_ms_total"] for x in group])),
            }
        )
    for r in runs_:
        r.pop("curve")
    return {
        "cases": summary_rows,
        "runs": runs_,
        "reference_free_m2": float((grid.seen & ~grid.occupied()).sum() * grid.res**2),
    }


# V: 인식


def visible(grid: GridMap, pose, xy, max_range=VIS_RANGE) -> tuple[bool, float, float]:
    """카메라 화각 안, max_range 이내, 기준 지도 가시선 확보 여부와 (거리, 방위각)."""
    x, y, th = pose
    d = math.hypot(xy[0] - x, xy[1] - y)
    b = wrap(math.atan2(xy[1] - y, xy[0] - x) - th)
    if d > max_range or abs(b) > config.CAMERA_FOV / 2 - 0.03:
        return False, d, b
    occ = grid.occupied()
    n = int(max(0.0, d - 0.15) / 0.025)
    for k in range(1, n):
        r, c = grid.to_cell(x + (xy[0] - x) * k * 0.025 / d, y + (xy[1] - y) * k * 0.025 / d)
        if grid.inside(r, c) and occ[r, c]:
            return False, d, b
    return True, d, b


def load_old_perception():
    """#17 이전 perception.py (30f3271)를 임시 모듈로 불러옴."""
    import importlib.util
    import subprocess

    src = subprocess.run(
        ["git", "show", "30f3271:controllers/sar_main/sar/perception.py"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    path = ROOT / "controllers" / "sar_eval" / "output" / "perception_30f3271.py"
    path.write_text(src, encoding="utf-8")
    spec = importlib.util.spec_from_file_location("perception_30f3271", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def detect_variants(frames: list[np.ndarray]) -> dict[str, list[list[dict]]]:
    """프레임별 검출 결과. 변형: current, color_only, yolo_only, yolo_no_color, before_17."""
    from sar.perception import TargetDetector

    det = TargetDetector(config.TARGET_COLOR, str(ROOT / "models" / "YOLO" / "yolo11n.pt"))
    old_mod = load_old_perception()
    with patched(MIN_CIRCULARITY=0.6):
        old = old_mod.TargetDetector(
            config.TARGET_COLOR, str(ROOT / "models" / "YOLO" / "yolo11n.pt")
        )
    real_boxes = det._yolo_boxes
    out: dict[str, list] = {
        k: [] for k in ("current", "color_only", "yolo_only", "yolo_no_color", "before_17")
    }
    for bgr in frames:
        boxes = real_boxes(bgr)

        def cached(_bgr, b=boxes):
            return [dict(x) for x in b]

        det._yolo_boxes = cached
        out["current"].append(det.detect_all(bgr))
        with patched(USE_COLOR_FALLBACK=False):
            out["yolo_only"].append(det.detect_all(bgr))
            with patched(COLOR_RATIO_MIN=0.0):
                out["yolo_no_color"].append(det.detect_all(bgr))
        det._yolo_boxes = lambda _bgr: []
        out["color_only"].append(det.detect_all(bgr))
        with patched(MIN_CIRCULARITY=0.6):
            out["before_17"].append(old.detect_all(bgr))
    det._yolo_boxes = real_boxes
    return out


def match_radius(dist: float) -> float:
    """검출 일치 반경 [m]. 거리 추정 오차(약 10%)를 고려해 거리의 15%와 MATCH_RADIUS 중 큰 값."""
    return max(MATCH_RADIUS, 0.15 * dist)


def det_world(det: dict, pose) -> tuple[float, float]:
    x, y, th = pose
    return x + det["dist"] * math.cos(th + det["bearing"]), y + det["dist"] * math.sin(
        th + det["bearing"]
    )


def classify(det: dict, pose, objects) -> tuple[str, int | None, float]:
    """검출을 가장 가까운 실제 물체로 분류. (이름, 빨간 사과 번호, 부호 있는 거리 오차 [m])."""
    wx, wy = det_world(det, pose)
    best, name, index = match_radius(det["dist"]), "background", None
    apple_i = 0
    for obj, ox, oy in objects:
        apple_i += obj == "RedApple"
        d = math.hypot(wx - ox, wy - oy)
        if d < best:
            best, name = d, obj
            index = apple_i if obj == "RedApple" else None
    err = math.nan
    if index is not None:
        ox, oy = red_apples(objects)[index - 1]
        err = det["dist"] - math.hypot(ox - pose[0], oy - pose[1])
    return name, index, err


def exp_perception(runs: dict, out: Path) -> dict:
    """truth·mission 실행 프레임 전체. 라벨과 검출 판정은 실제 pose 기준."""
    _, grid = reference(runs)
    objects = world_objects()
    apples = red_apples(objects)
    keys = [k for k in ("truth", "mission") if k in runs]
    items = [(k, s) for k in keys for s in runs[k]["frames"]]
    frames = [cv2.imread(str(runs[k]["folder"] / "frames" / f"{s:06d}.jpg")) for k, s in items]
    variants = detect_variants(frames)
    poses = [runs[k]["gt"][s] for k, s in items]
    labels = []  # 프레임별 [(사과 번호, 거리)] 보이는 빨간 사과
    for item, pose in zip(items, poses, strict=True):
        vis = []
        for i, a in enumerate(apples, start=1):
            ok, d, _ = visible(grid, pose, a)
            if ok and item not in MANUAL_NOT_VISIBLE:
                vis.append((i, d))
        labels.append(vis)
    n_frames = len(items)
    result = {
        "manual_label_exclusions": len(MANUAL_NOT_VISIBLE),
        "frames": n_frames,
        "frames_by_run": {k: len(runs[k]["frames"]) for k in keys},
        "visible_frames": sum(1 for v in labels if v),
    }
    for name, dets in variants.items():
        hit = {b: [0, 0] for b in DIST_BINS}
        fp_by = {}
        for pose, vis, frame_dets in zip(poses, labels, dets, strict=True):
            found = set()
            for d in frame_dets:
                obj, idx, _ = classify(d, pose, objects)
                if idx is None:
                    fp_by[obj] = fp_by.get(obj, 0) + 1
                else:
                    found.add(idx)
            for idx, dist in vis:
                for b in DIST_BINS:
                    if b[0] <= dist < b[1]:
                        hit[b][1] += 1
                        hit[b][0] += idx in found
        seen = sum(n for _, n in hit.values())
        result[name] = {
            "recall": {
                f"{a:.0f}-{b:.0f}m": (h / n if n else None) for (a, b), (h, n) in hit.items()
            },
            "recall_all": sum(h for h, _ in hit.values()) / max(1, seen),
            "visible_by_bin": {f"{a:.0f}-{b:.0f}m": n for (a, b), (_, n) in hit.items()},
            "fp_per_100": 100 * sum(fp_by.values()) / n_frames,
            "fp_by_object": fp_by,
        }
    # JPG 손실 영향: 실행 중 원본 화면 검출(detections.csv) 수와 비교
    live = {}
    for k in keys:
        for line in (runs[k]["folder"] / "detections.csv").read_text().splitlines()[1:]:
            key = (k, int(line.split(",")[0]))
            live[key] = live.get(key, 0) + 1
    same = sum(live.get(it, 0) == len(d) for it, d in zip(items, variants["current"], strict=True))
    result["jpg_agreement"] = same / n_frames
    result["confirm"] = {}
    for k in keys:
        sel = [i for i, it in enumerate(items) if it[0] == k]
        result["confirm"][k] = confirm_replay(
            [variants["current"][i] for i in sel],
            [poses[i] for i in sel],
            objects,
            [float(runs[k]["t"][items[i][1]]) for i in sel],
        )

    names = ["current", "before_17", "yolo_only", "color_only", "yolo_no_color"]
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.4), gridspec_kw={"width_ratios": [3, 2]})
    x = np.arange(len(DIST_BINS))
    for i, n in enumerate(names):
        rec = [v if v is not None else 0 for v in result[n]["recall"].values()]
        axes[0].bar(x + i * 0.16, np.asarray(rec) * 100, 0.16, color=COLORS[i], label=n)
    counts = result["current"]["visible_by_bin"].values()
    axes[0].set_xticks(
        x + 0.32,
        [f"{a:.0f}-{b:.0f} m\n(n={c})" for (a, b), c in zip(DIST_BINS, counts, strict=True)],
    )
    axes[0].set_ylabel("recall [% visible frames]")
    axes[0].legend(fontsize=7)
    axes[1].bar(names, [result[n]["fp_per_100"] for n in names], color=COLORS[: len(names)])
    axes[1].set_ylabel(f"false detections / 100 frames (n={n_frames})")
    axes[1].tick_params(axis="x", rotation=30, labelsize=7)
    save(fig, out, "design_perception_recall.png")

    true_d, est_d, src = [], [], []
    for pose, frame_dets in zip(poses, variants["current"], strict=True):
        for d in frame_dets:
            _, idx, e = classify(d, pose, objects)
            if idx is not None:
                true_d.append(d["dist"] - e)
                est_d.append(d["dist"])
                src.append(d["source"])
    err = np.asarray(est_d) - np.asarray(true_d)
    result["current"]["dist_err"] = {
        "n": len(err),
        "median": float(np.median(err)),
        "min": float(err.min()),
        "max": float(err.max()),
        "median_ratio": float(np.median(err / np.asarray(true_d))),
    }
    fig, ax = plt.subplots(figsize=(4.4, 3.6))
    for s_, c in (("yolo", COLORS[0]), ("color", COLORS[1])):
        m = [i for i, v in enumerate(src) if v == s_]
        ax.scatter(np.asarray(true_d)[m], np.asarray(est_d)[m], s=8, color=c, label=s_)
    lim = max([4.5] + est_d)
    ax.plot([0, lim], [0, lim], "k--", lw=0.8)
    ax.set_xlabel("true distance [m]")
    ax.set_ylabel("estimated distance [m]")
    ax.legend(fontsize=8)
    save(fig, out, "design_distance_error.png")
    return result


CONFIRM_NS = (1, 2, 3, 4, 6)


def confirm_replay(dets, poses, objects, t) -> dict:
    """프레임별 가장 가까운 검출을 Confirm에 넣어 확정 결과를 재현. 확정 후 초기화."""
    apples = red_apples(objects)
    result = {}
    for frames_n in CONFIRM_NS:
        conf = Confirm(frames=frames_n)
        true_ok, false_pos, first_time = set(), [], {}
        for pose, frame_dets, now in zip(poses, dets, t, strict=True):
            if not frame_dets:
                conf.update(False, None)
                continue
            d = frame_dets[0]
            ok, est = conf.update(True, det_world(d, pose), d["dist"])
            if not ok:
                continue
            conf.reset()
            radius = match_radius(d["dist"])
            near = [i for i, a in enumerate(apples, 1) if math.dist(a, est) <= radius]
            if near:
                true_ok.add(near[0])
                first_time.setdefault(near[0], now)
            else:
                false_pos.append(est)
        unique = []
        for p in false_pos:
            if all(math.dist(p, q) > MATCH_RADIUS for q in unique):
                unique.append(p)
        result[f"N={frames_n}"] = {
            "true_apples": len(true_ok),
            "false_events": len(false_pos),
            "false_unique": len(unique),
            "false_positions": [[round(v, 2) for v in p] for p in unique],
            "first_confirm_t": first_time,
        }
    # 매 step 호출: 추론하지 않은 step에 seen=False를 넣으면 연속 조건이 끊김
    conf = Confirm()
    confirmed = 0
    for frame_dets in dets:
        for _ in range(config.YOLO_EVERY - 1):
            conf.update(False, None)
        if frame_dets:
            ok, _ = conf.update(True, (0.0, 0.0), frame_dets[0]["dist"])
            confirmed += ok
    result["every_step_calls"] = {"confirmations": confirmed}
    return result


def plot_confirm(result: dict, out: Path) -> None:
    runs_ = result["confirm"]
    ns = CONFIRM_NS
    false = [sum(r[f"N={n}"]["false_events"] for r in runs_.values()) for n in ns]
    true = [sum(r[f"N={n}"]["true_apples"] for r in runs_.values()) for n in ns]
    fig, ax = plt.subplots(figsize=(5.5, 3.2))
    x = np.arange(len(ns))
    ax.bar(x - 0.2, false, 0.4, color=COLORS[3], label="false confirmations")
    ax.bar(x + 0.2, true, 0.4, color=COLORS[0], label="red apples confirmed (sum of runs)")
    ax.set_xticks(x, [f"N={n}" for n in ns])
    ax.set_xlabel("CONFIRM_FRAMES")
    ax.set_ylabel("count")
    ax.legend(fontsize=8)
    save(fig, out, "design_confirm_frames.png")


# M: 통합


def fit_compass_rms(samples, max_rms: float = 0.1):
    """기존 나침반 보정: 부호별 원형 평균 오프셋, 잔차 RMS가 작은 쪽. RMS > max_rms면 실패."""
    best = None
    for sign in (1, -1):
        diffs = [wrap(th - sign * raw) for th, raw in samples]
        off = math.atan2(sum(map(math.sin, diffs)), sum(map(math.cos, diffs)))
        rms = math.sqrt(sum(wrap(d - off) ** 2 for d in diffs) / len(diffs))
        if best is None or rms < best[2]:
            best = (sign, off, rms)
    return best if best[2] <= max_rms else None


def exp_compass(runs: dict, out: Path) -> dict:
    rng = np.random.default_rng(20260930)
    scales = np.round(np.arange(0.80, 1.3001, 0.01), 2)
    sigmas = (0.0, 0.02, 0.05)
    trials = 30
    n = int(round(2 * math.pi / (config.INIT_SPIN_W * DT)))
    theta0 = config.START_THETA
    result = {}
    for sigma in sigmas:
        rate = {"rms": [], "current": []}
        for s in scales:
            ok = {"rms": 0, "current": 0}
            for _ in range(trials if sigma else 1):
                true = theta0 + config.INIT_SPIN_W * DT * np.arange(n)
                odom = theta0 + s * (true - theta0)
                raw = math.pi / 2 - true + rng.normal(0, sigma, n)
                samples = [(wrap(a), wrap(b)) for a, b in zip(odom, raw, strict=True)]
                for name, fn in (("rms", fit_compass_rms), ("current", fit_compass)):
                    fit = fn(samples)
                    ok[name] += (
                        fit is not None and fit[0] == -1 and abs(wrap(fit[1] - math.pi / 2)) < 0.05
                    )
            for name in rate:
                rate[name].append(ok[name] / (trials if sigma else 1))
        result[f"sigma={sigma}"] = {
            name: {
                "success_range": success_range(scales, v),
                "rate_at_1.104": float(np.interp(1.104, scales, v)),
            }
            for name, v in rate.items()
        }
        result[f"sigma={sigma}"]["_curves"] = rate
    measured = {k: float(r["compass_fit"][2]) for k, r in runs.items() if r is not None}
    result["measured_scale"] = measured
    run = runs.get("truth") or runs["mission"]
    ok = ~np.isnan(run["compass"][:, 0])
    heading = np.asarray([compass_heading(v, -1, math.pi / 2) for v in run["compass"][ok]])
    resid = np.arctan2(np.sin(heading - run["gt"][ok, 2]), np.cos(heading - run["gt"][ok, 2]))
    result["compass_noise_rad"] = {"mean": float(resid.mean()), "std": float(resid.std())}
    fig, ax = plt.subplots(figsize=(6.2, 3.3))
    for i, sigma in enumerate(sigmas):
        curves = result[f"sigma={sigma}"].pop("_curves")
        ax.plot(
            scales,
            np.asarray(curves["rms"]) * 100,
            ls="--",
            color=COLORS[i],
            label=f"old (RMS), sigma={sigma}",
        )
        ax.plot(
            scales,
            np.asarray(curves["current"]) * 100,
            color=COLORS[i],
            label=f"current, sigma={sigma}",
        )
    ax.axvline(measured.get("mission", 1.104), color="k", ls=":", lw=0.8)
    ax.set_xlabel("odometry / true rotation ratio")
    ax.set_ylabel("fit success [%]")
    ax.legend(fontsize=6, ncol=2)
    save(fig, out, "design_compass_fit.png")
    return result


def success_range(scales, rates) -> list[float] | None:
    ok = [float(s) for s, r in zip(scales, rates, strict=True) if r >= 0.95]
    return [min(ok), max(ok)] if ok else None


def mission_summary(run: dict, objects) -> dict:
    apples = red_apples(objects)
    t, gt, state = run["t"], run["gt"], run["state"]
    rescued = []
    for x, y, rt in run["rescued"]:
        i = int(np.searchsorted(t, rt))
        name, dist = "background", math.inf
        for obj, ox, oy in objects:
            d = math.hypot(x - ox, y - oy)
            if d < dist:
                name, dist = obj, d
        true_xy = gt[min(i, len(gt) - 1), :2]
        rescued.append(
            {
                "t": float(rt),
                "xy": [float(x), float(y)],
                "nearest": name,
                "error_m": float(dist),
                "robot_true": true_xy.tolist(),
            }
        )
    done = np.nonzero(state == 6)[0]
    end = int(done[0]) if len(done) else len(t) - 1
    wall = run.get("wall")
    rt_factor = None
    if wall is not None and len(wall) > 1:
        rt_factor = float((t[-1] - t[0]) / (wall[-1] - wall[0]))
    yolo = run["tick_ms"][~np.isnan(run["yolo_ms"])]
    base = run["tick_ms"][np.isnan(run["yolo_ms"])]
    return {
        "rescued": rescued,
        "red_apples_rescued": len(
            {
                min(range(len(apples)), key=lambda i, r=r: math.dist(apples[i], r["xy"]))
                for r in rescued
                if r["nearest"] == "RedApple" and r["error_m"] < MATCH_RADIUS
            }
        ),
        "done": bool(len(done)),
        "time": float(t[end]),
        "return_error_true_m": float(
            math.hypot(gt[end, 0] - config.START_X, gt[end, 1] - config.START_Y)
        ),
        "return_error_est_m": float(
            math.hypot(run["est"][end, 0] - config.START_X, run["est"][end, 1] - config.START_Y)
        ),
        "contact_steps": int((run["contact"] > 0).sum()),
        "tick_ms_yolo": summary(yolo),
        "tick_ms_other": summary(base),
        "yolo_ms": summary(run["yolo_ms"]),
        "plan_ms": summary(run["plan_calls"][:, 0]) if len(run["plan_calls"]) else {"n": 0},
        "rt_factor": rt_factor,
        "path_m": float(np.hypot(*np.diff(gt[:, :2], axis=0).T).sum()),
    }


def exp_runs(runs: dict, out: Path) -> dict:
    objects = world_objects()
    result = {k: mission_summary(r, objects) for k, r in runs.items() if r is not None}
    run = runs.get("truth") or runs["mission"]
    yolo = float(np.nanmedian(run["yolo_ms"]))
    base = float(np.median(run["tick_ms"][np.isnan(run["yolo_ms"])]))
    budget = {}
    for n in (1, 2, 4, 8):
        per_step = base + yolo / n
        budget[f"N={n}"] = {
            "ms_per_step": per_step,
            "max_rt_factor": DT * 1000 / per_step,
            "move_between_m": config.V_MAX * DT * n,
            "spin_between_deg": math.degrees(config.INIT_SPIN_W * DT * n),
        }
    result["yolo_every"] = {"base_ms": base, "yolo_ms": yolo, **budget}
    calls = run["plan_calls"][:, 0] if len(run["plan_calls"]) else np.zeros(1)
    result["replan_period"] = {
        f"{p}s": {"ms_per_sim_s": 2 * float(np.median(calls)) / p} for p in (0.5, 1.0, 2.0, 5.0)
    }

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.3))
    tick = run["tick_ms"]
    is_yolo = ~np.isnan(run["yolo_ms"])
    bins = np.logspace(0, 3, 50)
    axes[0].hist(tick[~is_yolo], bins=bins, alpha=0.7, label="steps without YOLO", color=COLORS[0])
    axes[0].hist(tick[is_yolo], bins=bins, alpha=0.7, label="YOLO steps", color=COLORS[1])
    axes[0].axvline(DT * 1000, color="k", ls="--", lw=0.8)
    axes[0].set_xscale("log")
    axes[0].set_xlabel("tick() compute time [ms] (dashed: 64 ms step)")
    axes[0].set_ylabel("steps")
    axes[0].legend(fontsize=8)
    ns = [1, 2, 4, 8]
    axes[1].bar(
        [str(n) for n in ns], [budget[f"N={n}"]["ms_per_step"] for n in ns], color=COLORS[2]
    )
    axes[1].axhline(DT * 1000, color="k", ls="--", lw=0.8)
    axes[1].set_xlabel("YOLO_EVERY")
    axes[1].set_ylabel("mean compute per step [ms]")
    save(fig, out, "design_timing.png")

    _, grid = reference(runs)
    order = ["mission", "truth", "mission__CONFIRM_FRAMES-3", "truth__CONFIRM_FRAMES-3"]
    shown = [k for k in order if k in runs]
    ncols = 2
    nrows = math.ceil(len(shown) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(10, 4.2 * nrows), squeeze=False)
    extent = [grid.x0, grid.x0 + grid.n * grid.res, grid.y0, grid.y0 + grid.n * grid.res]
    img = np.full(grid.logodds.shape, 0.75)
    img[grid.seen] = 1.0
    img[grid.occupied()] = 0.2
    for ax, key in zip(axes.ravel(), shown, strict=False):
        r = runs[key]
        ax.imshow(img, cmap="gray", origin="lower", extent=extent, vmin=0, vmax=1)
        ax.plot(r["gt"][:, 0], r["gt"][:, 1], color=COLORS[0], lw=1, label="true path")
        if not key.startswith("truth"):
            ax.plot(
                r["est"][:, 0],
                r["est"][:, 1],
                color=COLORS[3],
                lw=1,
                ls="--",
                label="estimated path",
            )
        for i, (ax_, ay) in enumerate(red_apples(objects), start=1):
            ax.plot(ax_, ay, "o", color="red", ms=7)
            ax.annotate(
                f"apple {i}", (ax_, ay), fontsize=7, xytext=(4, 4), textcoords="offset points"
            )
        for x, y, _ in r["rescued"]:
            ax.plot(x, y, "x", color="k", ms=8, mew=2)
        ax.plot(config.START_X, config.START_Y, "s", color="g", ms=6)
        ax.set_xlim(-13, 1)
        ax.set_ylim(-13.5, 0.5)
        pose_src = "true pose" if key.startswith("truth") else "odometry"
        n3 = ", CONFIRM_FRAMES=3" if "CONFIRM_FRAMES-3" in key else ", CONFIRM_FRAMES=4"
        ax.set_title(f"{pose_src}{n3}: rescued {len(r['rescued'])}", fontsize=9)
        ax.legend(fontsize=7, loc="lower left")
    for ax in axes.ravel()[len(shown) :]:
        ax.axis("off")
    save(fig, out, "design_mission_runs.png")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--runs", type=Path, default=ROOT / "controllers" / "sar_eval" / "output")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--only", default="", help="쉼표 구분 실험: L,L2,P1,P2,P6,C,C3,V,M1,M5")
    args = parser.parse_args()
    out = args.out or args.runs / "design"
    out.mkdir(parents=True, exist_ok=True)
    runs = {p.name: load_run(p) for p in sorted(args.runs.iterdir()) if (p / "run.npz").exists()}
    only = set(filter(None, args.only.split(",")))
    results_path = out / "results.json"
    results = json.loads(results_path.read_text()) if results_path.exists() else {}
    experiments = {
        "L": lambda: exp_localization(runs, out),
        "L2": lambda: exp_map(runs, out),
        "P1": lambda: exp_logodds(out),
        "P2": lambda: exp_planner(runs, out),
        "P6": lambda: exp_explore(runs, out),
        "C": lambda: exp_control(runs, out),
        "C3": lambda: exp_safety(out),
        "V": lambda: exp_perception(runs, out),
        "M1": lambda: exp_compass(runs, out),
        "M5": lambda: exp_runs(runs, out),
    }
    for name, fn in experiments.items():
        if only and name not in only:
            continue
        start = time.perf_counter()
        results[name] = fn()
        if name == "V":
            plot_confirm(results[name], out)
        print(f"{name}: {time.perf_counter() - start:.1f} s")
        results_path.write_text(json.dumps(results, indent=1, default=float, ensure_ascii=False))
    print(f"결과: {results_path}")


if __name__ == "__main__":
    main()
