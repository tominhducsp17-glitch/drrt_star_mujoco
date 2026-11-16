"""Chạy bộ cảnh đánh giá (mục 2) với cell M1. Mỗi tiến trình nạp roadmap và dựng công cụ một lần.

    .venv/bin/python -m sortcell.eval.run_cell --scenes 50 --workers 12 --out results/m1
    MUJOCO_GL=egl .venv/bin/python -m sortcell.eval.run_cell --only 1000 --video results/m1/scene1000.mp4
"""
from __future__ import annotations

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

from ..control.arm_fsm import Cell
from ..planning.service import RoundPlanner
from ..scene.builder import Layout
from ..scene.sampling import sample_scene
from ..skills.scripted import ArmTools
from .metrics import summarize

_cell: Cell | None = None


def _init(budget: float):
    global _cell
    L = Layout()
    planner = RoundPlanner(L, budget=budget)
    _cell = Cell(L, planner, [ArmTools(L, k, planner.spaces[k]) for k in range(L.n_arms)])


def _run(seed: int, video: str | None = None) -> dict:
    rec = None
    if video:
        from ..control.video import Recorder
        rec = Recorder(video, size=(960, 720))
    t = time.perf_counter()
    try:
        r = _cell.run(sample_scene(seed), seed=seed, recorder=rec)
    finally:
        if rec is not None:
            rec.close()
    d = asdict(r)
    d["wall_time"] = time.perf_counter() - t
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, default=50)
    ap.add_argument("--first-seed", type=int, default=1000)
    ap.add_argument("--only", type=int, nargs="*", default=None)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--budget", type=float, default=0.5)
    ap.add_argument("--out", default="results/m1")
    ap.add_argument("--video", default=None)
    a = ap.parse_args()
    seeds = a.only if a.only else list(range(a.first_seed, a.first_seed + a.scenes))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    results = []
    if a.video or len(seeds) == 1:
        _init(a.budget)
        results = [_run(s, a.video) for s in seeds]
    else:
        with ProcessPoolExecutor(a.workers, initializer=_init, initargs=(a.budget,)) as ex:
            for r in ex.map(_run, seeds):
                results.append(r)
                print(f"seed {r['seed']}: đúng {r['placed_correct']}/{r['n_cubes']}, va chạm {r['collision_steps']}, "
                      f"gắp {r['pick_success']}/{r['picks']}, makespan {r['t_last_place']:.1f} s, "
                      f"lập kế hoạch {r['planning_wall']:.1f} s, {r['rounds']} lượt, hoãn {r['deferrals']}, "
                      f"deadlock {r['deadlock']}", flush=True)
    for r in results:
        (out / f"scene_{r['seed']}.json").write_text(json.dumps(r, indent=1, default=str))
    if len(results) > 1 or not a.video:
        s = summarize(results)
        (out / "summary.json").write_text(json.dumps(s, indent=1))
        print(json.dumps(s, indent=1))
    else:
        r = results[0]
        print({k: v for k, v in r.items() if k not in ("events",)})


if __name__ == "__main__":
    main()
