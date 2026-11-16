"""Chạy bộ cảnh đánh giá với cell M2 (lệch giờ, lập kế hoạch lại theo sự kiện).

    .venv/bin/python -m sortcell.eval.run_m2 --alloc level1 --scenes 50 --out results/m2_level1
"""
from __future__ import annotations

import argparse
import gzip
import json
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path

import numpy as np

from ..allocation.costs import build_cost_table
from ..control.staggered import StaggeredCell
from ..planning.service import PlannerService
from ..scene.builder import Layout
from ..scene.sampling import sample_scene
from ..skills.scripted import ArmTools
from .metrics import perception_summary, summarize

_cell: StaggeredCell | None = None


def make_cell(alloc: str, budget: float, penalty: float | None = None, max_total: float = 1.0,
              perception: bool = False) -> StaggeredCell:
    L = Layout()
    planner = PlannerService(L, max_total=max_total)
    tools = [ArmTools(L, k, planner.spaces[k]) for k in range(L.n_arms)]
    tables = [build_cost_table(L, k, planner.rms[k], tools[k]) for k in range(L.n_arms)]
    return StaggeredCell(L, planner, tools, tables, alloc=alloc, budget=budget, penalty=penalty, perception=perception)


def _init(alloc, budget, penalty, max_total=1.0, perception=False):
    global _cell
    _cell = make_cell(alloc, budget, penalty, max_total, perception)


def _run(seed: int, video: str | None = None, view: float | None = None) -> dict:
    rec = None
    if view:
        from ..control.video import LiveViewer
        rec = LiveViewer(speed=view)
    elif video:
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
    d["rounds"] = d["replans"]
    d["plan_times"] = d["replan_times"]
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alloc", default="level1", choices=["level0", "level1"])
    ap.add_argument("--scenes", type=int, default=50)
    ap.add_argument("--first-seed", type=int, default=1000)
    ap.add_argument("--only", type=int, nargs="*", default=None)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--budget", type=float, default=0.3)
    ap.add_argument("--penalty", type=float, default=None)
    ap.add_argument("--perception", action="store_true", help="pose từ camera trên cao (M3)")
    ap.add_argument("--max-total", type=float, default=1.0, help="tổng ngân sách một lần lập kế hoạch (s)")
    ap.add_argument("--out", default=None, help="thư mục ghi kết quả; bỏ trống thì chỉ in ra, không ghi")
    ap.add_argument("--video", default=None)
    ap.add_argument("--view", type=float, nargs="?", const=1.0, default=None,
                    help="xem trực tiếp bằng cửa sổ MuJoCo; số = tốc độ so với thời gian thực (mặc định 1)")
    a = ap.parse_args()
    seeds = a.only if a.only else list(range(a.first_seed, a.first_seed + a.scenes))
    out = Path(a.out) if a.out else None
    if out is not None:
        out.mkdir(parents=True, exist_ok=True)
    if a.video or a.view or len(seeds) == 1:
        _init(a.alloc, a.budget, a.penalty, a.max_total, a.perception)
        results = []
        for s in seeds:
            try:
                r = _run(s, a.video, a.view)
            except KeyboardInterrupt:
                print("dừng (đóng cửa sổ hoặc Ctrl+C)")
                return
            results.append(r)
            print(f"seed {r['seed']}: đúng {r['placed_correct']}/{r['n_cubes']}, va chạm {r['collision_steps']}, "
                  f"gắp {r['pick_success']}/{r['picks']}, makespan {r['t_last_place']:.1f} s", flush=True)
    else:
        _init(a.alloc, a.budget, a.penalty, a.max_total, a.perception)  # dựng cache (bảng chi phí) trước khi chia tiến trình
        results = []
        with ProcessPoolExecutor(a.workers, initializer=_init, initargs=(a.alloc, a.budget, a.penalty, a.max_total, a.perception)) as ex:
            for r in ex.map(_run, seeds):
                results.append(r)
                print(f"seed {r['seed']}: đúng {r['placed_correct']}/{r['n_cubes']}, va chạm {r['collision_steps']}, "
                      f"gắp {r['pick_success']}/{r['picks']}, makespan {r['t_last_place']:.1f} s, "
                      f"{r['replans']} lần lập kế hoạch, thất bại {r['plan_failures']}, deadlock {r['deadlock']}",
                      flush=True)
    s = summarize(results)
    if a.perception:
        s["perception"] = perception_summary(results)
    for r in results:
        perc = r.pop("perception", None)
        if out is None:
            continue
        if perc is not None:  # khung hình đánh giá perception: lớn, lưu nén riêng
            with gzip.open(out / f"perception_{r['seed']}.json.gz", "wt") as f:
                json.dump(perc, f)
        (out / f"scene_{r['seed']}.json").write_text(json.dumps(r, indent=1, default=str))
    rt = np.concatenate([r["replan_times"] for r in results])
    s["replan_latency_ms"] = {"median": float(np.median(rt) * 1000), "p95": float(np.percentile(rt, 95) * 1000),
                              "p99": float(np.percentile(rt, 99) * 1000), "max": float(rt.max() * 1000),
                              "n": int(rt.size), "failures": int(sum(r["plan_failures"] for r in results))}
    s["wait_local_s_per_scene"] = float(np.mean([r["wait_local_time"] for r in results]))
    if out is not None:
        (out / "summary.json").write_text(json.dumps(s, indent=1))
    print(json.dumps(s, indent=1))


if __name__ == "__main__":
    main()
