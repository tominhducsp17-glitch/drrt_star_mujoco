"""P4: cell 2 tay UR10e, tác vụ đi chéo qua cột (cell_tasks.crossing_two_arms).

Benchmark gọn theo tinh thần Hình 13: dRRT* so với Implicit A* (tối ưu trên cùng Ĝ) và RRT* trong không gian hợp 12 chiều,
với vài cỡ roadmap. Sau đó chạy nghiệm dRRT* trong mj_step đúng như planner trả về: mỗi cạnh là một bước di chuyển đồng thời, tay nào không đổi
đỉnh trên cạnh đó thì đứng yên chờ (Adj(v) chứa chính v, như bài). Ghi video. Bản rút ngắn đường (ngoài bài) chỉ để tham khảo.
"""
from __future__ import annotations

import argparse
import csv
import json
import resource
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from drrt.cell_tasks import TASKS, CompositeSpace, free_volume, make_problem, roadmaps, tensor_graph
from drrt.execute import execute, save_video, time_parameterize
from drrt.planners import CompositeRRTStar, DRRTStar, implicit_astar
from drrt.shortcut import path_length, shortcut

CACHE = Path("outputs/cache")


def limit_memory(gigabytes: float) -> None:
    size = int(gigabytes * 2 ** 30)
    resource.setrlimit(resource.RLIMIT_DATA, (size, size))


def uncoordinated_free(g) -> bool:
    """Mọi tay cùng lúc đi theo đường ngắn nhất riêng lẻ trên roadmap của mình: có va nhau không."""
    hops = [rm.next_hop(t) for rm, t in zip(g.roadmaps, g.target)]
    v = g.start
    while v != g.target:
        u = tuple(int(h[a]) if a != t else a for h, a, t in zip(hops, v, g.target))
        if not g.edge_free(v, u):
            return False
        v = u
    return True


def run(task: str, algo: str, n: int, seed: int, budget: float) -> dict:
    problem = make_problem(TASKS[task]())
    row = {"algo": algo, "n": n, "seed": seed}
    if algo == "RRT*":
        space = CompositeSpace(problem)
        volume = float(np.prod([free_volume(a) for a in problem.arms]))
        planner = CompositeRRTStar(space, np.concatenate(problem.start), np.concatenate(problem.goal),
                                   np.random.default_rng([seed, 5]), volume, step=1.0)
        r = planner.solve(time_limit=budget)
    else:
        g = tensor_graph(problem, roadmaps(problem, n, seed, CACHE))
        row["lower_bound"] = g.h(g.start)
        row["uncoordinated_free"] = int(uncoordinated_free(g))
        try:
            r = (implicit_astar(g, time_limit=budget) if algo == "Implicit A*"
                 else DRRTStar(g, np.random.default_rng([seed, 1])).solve(time_limit=budget))
        except MemoryError:
            return {**row, "solved": 0, "oom": 1}
        row["history"] = json.dumps([(round(t, 3), round(c, 4)) for _, t, c in r.history])
    row.update({"solved": int(r.solved), "first_time": r.first_time, "first_cost": r.first_cost,
                "final_cost": r.cost if r.solved else None, "iterations": r.iterations, "elapsed": r.elapsed,
                "collision_checks": r.collision_checks})
    return row


FIELDS = ["algo", "n", "seed", "solved", "first_time", "first_cost", "final_cost", "lower_bound", "uncoordinated_free",
          "iterations", "elapsed", "collision_checks", "oom", "history"]


def benchmark(args) -> None:
    jobs = [(a, n, s) for n in args.sizes for s in range(args.seeds) for a in ("dRRT*", "Implicit A*")]
    jobs += [("RRT*", 0, s) for s in range(args.seeds)]
    # Dựng và cache roadmap trước để các tiến trình không dựng trùng.
    problem = make_problem(TASKS[args.task]())
    for n in args.sizes:
        for s in range(args.seeds):
            roadmaps(problem, n, s, CACHE)
    with ProcessPoolExecutor(args.workers, initializer=limit_memory, initargs=(args.memory_gb,)) as pool:
        futures = [pool.submit(run, args.task, a, n, s, args.budget) for a, n, s in jobs]
        with open(args.out / "summary.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, FIELDS)
            writer.writeheader()
            for i, fut in enumerate(as_completed(futures), 1):
                writer.writerow(fut.result())
                f.flush()
                print(f"{i}/{len(futures)} {time.strftime('%H:%M:%S')}", flush=True)


def demo(args) -> None:
    problem = make_problem(TASKS[args.task]())
    g = tensor_graph(problem, roadmaps(problem, args.demo_n, 0, CACHE))
    r = DRRTStar(g, np.random.default_rng(0)).solve(time_limit=args.budget)
    raw = [g.flat(v) for v in r.path]
    space = CompositeSpace(problem)
    short = shortcut(raw, space.edge_free, problem.R, np.random.default_rng(0))
    info = {"n": args.demo_n, "budget_s": args.budget, "first_time": r.first_time, "first_cost": r.first_cost,
            "drrt_cost": r.cost, "lower_bound": g.h(g.start), "waypoints": len(raw),
            "shortcut_cost": path_length(short, problem.R), "shortcut_waypoints": len(short),
            "straight_line_free": bool(space.edge_free(raw[0], raw[-1])),
            "straight_line_cost": path_length([raw[0], raw[-1]], problem.R)}
    # Các (cạnh, tay) mà tay đó đứng yên trong khi tay khác di chuyển.
    waits = [(e, k) for e, (u, v) in enumerate(zip(r.path, r.path[1:])) for k in range(problem.R) if u[k] == v[k]]
    info["edges"] = len(r.path) - 1
    info["waits"] = waits
    runs = [("raw", time_parameterize(raw)), ("shortcut", time_parameterize(short))]
    for label, traj in runs:
        ex = execute(problem.task.cell, traj, render_fps=30 if label == "raw" else 0)
        info[f"{label}_duration_s"] = traj.duration
        info[f"{label}_max_tracking_error_rad"] = ex.max_tracking_error
        info[f"{label}_collision_steps"] = ex.collision_steps
        info[f"{label}_collision_pairs"] = sorted(map(list, ex.collision_pairs))
        if ex.frames:
            save_video(ex.frames, args.out / f"{args.task}_demo.mp4", 30)
            info["video"] = label
    (args.out / "demo.json").write_text(json.dumps(info, indent=2, ensure_ascii=False))
    print(json.dumps(info, indent=2, ensure_ascii=False))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="crossing2", choices=sorted(TASKS))
    ap.add_argument("--out", type=Path, default=Path("results/p4"))
    ap.add_argument("--sizes", type=int, nargs="+", default=[300, 1000, 3000])
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--budget", type=float, default=60.0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--memory-gb", type=float, default=3.0)
    ap.add_argument("--demo-n", type=int, default=1000)
    ap.add_argument("--skip-benchmark", action="store_true")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if not args.skip_benchmark:
        benchmark(args)
    demo(args)


if __name__ == "__main__":
    main()
