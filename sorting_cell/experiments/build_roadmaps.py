"""Dựng và cache roadmap của 4 tay (song song), in kích thước và thời gian."""
import argparse
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from sortcell.planning.models import ArmSpace
from sortcell.planning.roadmaps import ArmRoadmap, build_roadmap
from sortcell.scene.builder import Layout


def one(args):
    k, nt, nu, seed = args
    t = time.perf_counter()
    rm = build_roadmap(Layout(), k, nt, nu, seed)
    built = time.perf_counter() - t
    ar = ArmRoadmap(rm, ArmSpace(Layout(), k))
    return k, rm.n, rm.n_edges, ar.n, len(ar.edges), built


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-task", type=int, default=1500)
    ap.add_argument("--n-uniform", type=int, default=1500)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    with ProcessPoolExecutor(4) as ex:
        for k, n, e, nc, ec, t in ex.map(one, [(k, a.n_task, a.n_uniform, a.seed) for k in range(4)]):
            print(f"tay {k}: {n} đỉnh, {e} cạnh; thành phần chứa HOME {nc} đỉnh, {ec} cạnh; dựng {t:.1f} s")
