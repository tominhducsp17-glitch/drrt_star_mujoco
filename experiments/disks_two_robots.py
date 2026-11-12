"""Tái hiện Mục 8.1, Hình 10 và Bảng 1. Hai đĩa đổi chỗ (0,0) ↔ (9,9) trong môi trường Hình 9.

Cấu trúc thí nghiệm theo chú thích Hình 10: với mỗi n, 10 cặp roadmap; dRRT* và ao-dRRT chạy 5 lần trên mỗi cặp;
Implicit A* chạy trên cả 10 tổ hợp; N²-PRM* dựng 10 lần (3 lần ở n=200 vì rất tốn).
"""
from __future__ import annotations

import argparse
import csv
import functools
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from drrt.checkers.disk2d import CompositeDiskSpace, DiskPairChecker, DiskSpace
from drrt.planners import AODRRT, DRRTStar, implicit_astar
from drrt.planners.prm_star import composite_sum_graph, shortest_path_cost
from drrt.roadmap import Roadmap, r_star
from drrt.tensor_graph import TensorGraph

SWAP = ((0.0, 0.0), (9.0, 9.0))
DISK_RADIUS = 0.2
CHECKPOINTS = list(range(5000, 100001, 5000))


def build_connected(space: DiskSpace, n: int, rng: np.random.Generator, required) -> tuple[Roadmap, int]:
    """Bài không nói cách xử lý roadmap không nối được s–t; ở đây lấy mẫu lại và ghi số lần thử."""
    radius = r_star(n, 2, space.free_area)
    for attempt in range(1, 10000):
        roadmap = Roadmap.build(space, n, rng, radius=radius, required=required)
        if roadmap.connected(0, 1):
            return roadmap, attempt
    raise RuntimeError("không dựng được roadmap liên thông")


@functools.lru_cache(maxsize=2)
def roadmap_pair(n: int, pair: int):
    rng = np.random.default_rng([n, pair])
    space = DiskSpace()
    t0 = time.perf_counter()
    a, tries_a = build_connected(space, n, rng, SWAP)
    build_a = (time.perf_counter() - t0) / tries_a
    t0 = time.perf_counter()
    b, tries_b = build_connected(space, n, rng, SWAP[::-1])
    build_b = (time.perf_counter() - t0) / tries_b
    return a, b, build_a + build_b, tries_a + tries_b


def graph_for(n: int, pair: int) -> TensorGraph:
    a, b, _, _ = roadmap_pair(n, pair)
    return TensorGraph([a, b], DiskPairChecker(DISK_RADIUS), (0, 0), (1, 1), cost="sum")


def run_tree(kind: str, n: int, pair: int, run: int, iters: int) -> list[dict]:
    g = graph_for(n, pair)
    cls = {"dRRT*": DRRTStar, "ao-dRRT": AODRRT}[kind]
    r = cls(g, np.random.default_rng([n, pair, run, 7])).solve(max_iters=iters)
    base = {"algo": kind, "n": n, "pair": pair, "run": run, "first_time": r.first_time, "first_iter": r.first_iter,
            "first_cost": r.first_cost, "final_cost": r.cost if r.solved else None, "elapsed": r.elapsed,
            "build_time": roadmap_pair(n, pair)[2], "build_tries": roadmap_pair(n, pair)[3],
            "tree_size": r.tree_size, "collision_checks": r.collision_checks}
    rows = [base]
    history = r.history
    for cp in CHECKPOINTS:
        if cp > iters:
            break
        cost = None
        for it, _, c in history:
            if it > cp:
                break
            cost = c
        rows.append({"algo": kind, "n": n, "pair": pair, "run": run, "checkpoint": cp, "cost": cost})
    return rows


def run_astar(n: int, pair: int, limit: float) -> list[dict]:
    r = implicit_astar(graph_for(n, pair), time_limit=limit)
    return [{"algo": "Implicit A*", "n": n, "pair": pair, "run": 0, "first_time": r.first_time,
             "first_cost": r.first_cost, "final_cost": r.cost if r.solved else None, "elapsed": r.elapsed,
             "collision_checks": r.collision_checks}]


def run_prm(n: int, run: int) -> list[dict]:
    """N²-PRM* trong không gian hợp 4 chiều: cùng số đỉnh với Ĝ nhưng phải dựng tường minh."""
    rng = np.random.default_rng([n, run, 99])
    base = DiskSpace()
    space = CompositeDiskSpace(base, 2, DISK_RADIUS)
    N = n * n
    start = np.array([*SWAP[0], *SWAP[1]])
    goal = np.array([*SWAP[1], *SWAP[0]])
    t0 = time.perf_counter()
    roadmap = Roadmap.build(space, N, rng, radius=r_star(N, 4, base.free_area ** 2), required=[start, goal])
    graph = composite_sum_graph(roadmap, 2)
    build = time.perf_counter() - t0
    t0 = time.perf_counter()
    cost = shortest_path_cost(graph, 0, 1)
    query = time.perf_counter() - t0
    return [{"algo": "N2-PRM*", "n": n, "pair": -1, "run": run, "first_time": query,
             "final_cost": cost if np.isfinite(cost) else None, "build_time": build, "n_edges": roadmap.n_edges}]


FIELDS = ["algo", "n", "pair", "run", "first_time", "first_iter", "first_cost", "final_cost", "elapsed", "build_time",
          "build_tries", "tree_size", "collision_checks", "n_edges"]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("results/disks_two_robots"))
    ap.add_argument("--sizes", type=int, nargs="+", default=[50, 100, 200])
    ap.add_argument("--pairs", type=int, default=10)
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--iters", type=int, default=100000)
    ap.add_argument("--workers", type=int, default=14)
    ap.add_argument("--astar-limit", type=float, default=1800.0)
    ap.add_argument("--no-prm", action="store_true")
    ap.add_argument("--only-prm", action="store_true", help="chỉ chạy lại N²-PRM*, thay các dòng cũ trong summary.csv")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    if args.only_prm:
        path = args.out / "summary.csv"
        with open(path, newline="") as f:
            kept = [r for r in csv.DictReader(f) if r["algo"] != "N2-PRM*"]
        with ProcessPoolExecutor(args.workers) as pool:
            jobs = [pool.submit(run_prm, n, run) for n in args.sizes for run in range(3 if n >= 200 else args.pairs)]
            for job in as_completed(jobs):
                kept.extend(job.result())
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, FIELDS)
            writer.writeheader()
            writer.writerows(kept)
        return

    jobs = []
    with ProcessPoolExecutor(args.workers) as pool:
        for n in args.sizes:
            for pair in range(args.pairs):
                jobs.append(pool.submit(run_astar, n, pair, args.astar_limit))
                for run in range(args.runs):
                    jobs.append(pool.submit(run_tree, "dRRT*", n, pair, run, args.iters))
                    jobs.append(pool.submit(run_tree, "ao-dRRT", n, pair, run, args.iters))
            if not args.no_prm:
                for run in range(3 if n >= 200 else args.pairs):
                    jobs.append(pool.submit(run_prm, n, run))
        with open(args.out / "summary.csv", "w", newline="") as fs, open(args.out / "convergence.csv", "w", newline="") as fc:
            summary = csv.DictWriter(fs, FIELDS)
            summary.writeheader()
            conv = csv.DictWriter(fc, ["algo", "n", "pair", "run", "checkpoint", "cost"])
            conv.writeheader()
            for i, job in enumerate(as_completed(jobs), 1):
                for row in job.result():
                    (conv if "checkpoint" in row else summary).writerow(row)
                fs.flush()
                fc.flush()
                if i % 20 == 0 or i == len(jobs):
                    print(f"{i}/{len(jobs)} {time.strftime('%H:%M:%S')}", flush=True)


if __name__ == "__main__":
    main()
