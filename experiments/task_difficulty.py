"""Đo độ khó của các biến thể tác vụ 2 tay: đường riêng lẻ có va nhau không, A* mất bao lâu, dRRT* tìm nghiệm đầu bao lâu."""
from __future__ import annotations

import argparse
import itertools
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from p4_two_arm import limit_memory, uncoordinated_free  # noqa: E402

from drrt.cell_tasks import crossing_two_arms, make_problem, roadmaps, swap_two_arms, tensor_graph  # noqa: E402
from drrt.planners import DRRTStar, implicit_astar  # noqa: E402

VARIANTS = {
    "crossing (hiện tại)": lambda: crossing_two_arms(),
    "swap, không vật cản": lambda: swap_two_arms(bar=False),
    "swap, thanh 1,40 m": lambda: swap_two_arms(bar=True, bar_height=1.40),
    "swap, thanh 1,40 m + cột": lambda: swap_two_arms(pole=True, bar=True, bar_height=1.40),
    "swap, thanh 1,30 m": lambda: swap_two_arms(bar=True, bar_height=1.30),
}


def job(args):
    name, seed, n, budget = args
    task = VARIANTS[name]()
    task.name = f"var{list(VARIANTS).index(name)}"
    try:
        p = make_problem(task)
    except ValueError as e:
        return name, seed, f"không có query: {e}"
    g = tensor_graph(p, roadmaps(p, n, seed, Path("outputs/cache")))
    free = uncoordinated_free(g)
    try:
        a = implicit_astar(tensor_graph(p, g.roadmaps), time_limit=budget)
        astar = f"A* {a.elapsed:.1f}s mở {a.iterations} đỉnh" + (f" cost {a.cost:.2f}" if a.solved else " KHÔNG GIẢI")
    except MemoryError:
        astar = "A* hết RAM"
    r = DRRTStar(g, np.random.default_rng(seed)).solve(time_limit=budget)
    d = f"dRRT* đầu {r.first_time:.3f}s cost {r.first_cost:.2f} → {r.cost:.2f}" if r.solved else "dRRT* KHÔNG GIẢI"
    return name, seed, f"riêng lẻ {'tự do' if free else 'va nhau'} | mốc {g.h(g.start):.2f} | {astar} | {d}"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--only", nargs="*", help="chỉ chạy các biến thể có tên chứa chuỗi này")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--budget", type=float, default=60)
    a = ap.parse_args()
    names = [v for v in VARIANTS if not a.only or any(o in v for o in a.only)]
    jobs = [(v, s, a.n, a.budget) for v, s in itertools.product(names, range(a.seeds))]
    with ProcessPoolExecutor(8, initializer=limit_memory, initargs=(3.0,)) as pool:
        for name, seed, line in pool.map(job, jobs):
            print(f"{name:28s} seed {seed}: {line}", flush=True)
