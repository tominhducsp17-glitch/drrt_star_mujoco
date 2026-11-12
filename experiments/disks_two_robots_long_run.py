"""Chạy dài dRRT* ở n=200 để xác nhận hội tụ đúng tối ưu của Ĝ (Theorem 2), không chỉ tới lân cận."""
from __future__ import annotations

import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from disks_two_robots import graph_for  # noqa: E402

from drrt.planners import DRRTStar, implicit_astar  # noqa: E402

ITERS = 1000000


def job(pair: int) -> tuple:
    optimum = implicit_astar(graph_for(200, pair)).cost
    r = DRRTStar(graph_for(200, pair), np.random.default_rng([200, pair, 0, 7])).solve(max_iters=ITERS)
    return pair, optimum, r.cost, r.history[-1][0], r.elapsed


if __name__ == "__main__":
    with ProcessPoolExecutor(10) as pool:
        for pair, opt, cost, last, t in pool.map(job, range(10)):
            print(f"pair {pair}: tối ưu {opt:.4f}, dRRT* {cost:.4f}, chênh {(cost / opt - 1) * 100:.4f}%, "
                  f"cải thiện cuối ở vòng {last}, {t:.0f}s", flush=True)
