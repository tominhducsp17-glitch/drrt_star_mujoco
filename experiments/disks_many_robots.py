"""Tái hiện Hình 11. R = 3…10 đĩa từ biên sang phía đối diện, roadmap n = 50 cho mỗi robot, 100k vòng, 10 lần chạy.

Mỗi lần chạy dùng cùng một bộ roadmap và cùng truy vấn cho mọi thuật toán. Bài không giới hạn thời gian, nhưng ở đây mỗi lần
chạy dừng sau --time-limit giây; số vòng lặp thực tế được ghi lại để biết thuật toán nào không chạy đủ 100k vòng.
"""
from __future__ import annotations

import argparse
import csv
import functools
import resource
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from drrt.checkers.disk2d import CompositeDiskSpace, DiskPairChecker, DiskSpace, perimeter_swap_query
from drrt.planners import AODRRT, CompositeRRTStar, DRRT, DRRTStar, implicit_astar
from drrt.roadmap import Roadmap, r_star
from drrt.tensor_graph import TensorGraph

DISK_RADIUS = 0.2


def offset_for(R: int, run: int) -> float:
    """Xoay vị trí xuất phát theo từng lần chạy để truy vấn không trùng nhau."""
    return float(np.random.default_rng([R, run, 123]).uniform(0, 1 / R))


@functools.lru_cache(maxsize=2)
def roadmaps_for(R: int, n: int, run: int):
    rng = np.random.default_rng([R, n, run])
    space = DiskSpace()
    radius = r_star(n, 2, space.free_area)
    out = []
    tries = 0
    for s, t in perimeter_swap_query(R, offset_for(R, run)):
        while True:
            tries += 1
            rm = Roadmap.build(space, n, rng, radius=radius, required=[s, t])
            if rm.connected(0, 1):
                out.append(rm)
                break
    return tuple(out), tries


def graph_for(R: int, n: int, run: int) -> TensorGraph:
    roadmaps, _ = roadmaps_for(R, n, run)
    return TensorGraph(roadmaps, DiskPairChecker(DISK_RADIUS), (0,) * R, (1,) * R)


def row(algo, R, n, run, r, lower_bound, tries=None) -> dict:
    return {"algo": algo, "R": R, "n": n, "run": run, "solved": int(r.solved), "first_time": r.first_time,
            "first_iter": r.first_iter, "first_cost": r.first_cost, "final_cost": r.cost if r.solved else None,
            "lower_bound": lower_bound, "iterations": r.iterations, "elapsed": r.elapsed, "tree_size": r.tree_size,
            "collision_checks": r.collision_checks, "build_tries": tries}


def limit_memory(gigabytes: float) -> None:
    """A* trên đồ thị 50^R đỉnh có thể dùng hết RAM; giới hạn theo từng tiến trình worker."""
    size = int(gigabytes * 2 ** 30)
    resource.setrlimit(resource.RLIMIT_DATA, (size, size))


def run_job(algo: str, R: int, n: int, run: int, iters: int, limit: float) -> dict:
    t0 = time.perf_counter()
    try:
        return _run_job(algo, R, n, run, iters, limit)
    except MemoryError:
        roadmaps_for.cache_clear()
        return {"algo": algo, "R": R, "n": n, "run": run, "solved": 0, "elapsed": time.perf_counter() - t0, "oom": 1}


def _run_job(algo: str, R: int, n: int, run: int, iters: int, limit: float) -> dict:
    roadmaps, tries = roadmaps_for(R, n, run)
    g = graph_for(R, n, run)
    bound = float(sum(table[0] for table in g.h_i))
    rng = np.random.default_rng([R, n, run, sum(map(ord, algo))])
    if algo == "dRRT*":
        r = DRRTStar(g, rng).solve(max_iters=iters, time_limit=limit)
    elif algo == "ao-dRRT":
        r = AODRRT(g, rng).solve(max_iters=iters, time_limit=limit)
    elif algo == "dRRT":
        r = DRRT(g, rng).solve(max_iters=iters, time_limit=limit, stop_on_first=True)
    elif algo == "Implicit A*":
        r = implicit_astar(g, time_limit=limit)
    elif algo == "RRT*":
        base = DiskSpace()
        space = CompositeDiskSpace(base, R, DISK_RADIUS)
        query = perimeter_swap_query(R, offset_for(R, run))
        start = np.array([c for s, _ in query for c in s])
        goal = np.array([c for _, t in query for c in t])
        r = CompositeRRTStar(space, start, goal, rng, base.free_area ** R).solve(max_iters=iters, time_limit=limit)
    else:
        raise ValueError(algo)
    return row(algo, R, n, run, r, bound, tries)


FIELDS = ["algo", "R", "n", "run", "solved", "first_time", "first_iter", "first_cost", "final_cost", "lower_bound",
          "iterations", "elapsed", "tree_size", "collision_checks", "build_tries", "oom"]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("results/disks_many_robots"))
    ap.add_argument("--robots", type=int, nargs="+", default=list(range(3, 11)))
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--runs", type=int, default=10)
    ap.add_argument("--iters", type=int, default=100000)
    ap.add_argument("--time-limit", type=float, default=300.0)
    ap.add_argument("--astar-max-robots", type=int, default=6)
    ap.add_argument("--workers", type=int, default=14)
    ap.add_argument("--memory-gb", type=float, default=1.4, help="giới hạn bộ nhớ của mỗi worker")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    with ProcessPoolExecutor(args.workers, initializer=limit_memory, initargs=(args.memory_gb,)) as pool:
        jobs = []
        for R in args.robots:
            for run in range(args.runs):
                for algo in ("dRRT*", "ao-dRRT", "dRRT", "RRT*"):
                    jobs.append(pool.submit(run_job, algo, R, args.n, run, args.iters, args.time_limit))
                if R <= args.astar_max_robots:
                    jobs.append(pool.submit(run_job, "Implicit A*", R, args.n, run, args.iters, args.time_limit))
        with open(args.out / "summary.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, FIELDS)
            writer.writeheader()
            for i, job in enumerate(as_completed(jobs), 1):
                writer.writerow(job.result())
                f.flush()
                if i % 20 == 0 or i == len(jobs):
                    print(f"{i}/{len(jobs)} {time.strftime('%H:%M:%S')}", flush=True)


if __name__ == "__main__":
    main()
