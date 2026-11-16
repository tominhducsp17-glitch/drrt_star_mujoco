"""Quét hằng số phạt tranh chấp của mức 1 trong mô hình lập lịch (không mô phỏng), so với CP-SAT."""
import json
from pathlib import Path

import numpy as np

from sortcell.allocation.costs import build_cost_table
from sortcell.allocation.milp import make_instance, simulate_rule
from sortcell.planning.service import PlannerService
from sortcell.scene.builder import Layout
from sortcell.scene.sampling import eval_scenes
from sortcell.skills.scripted import ArmTools

ROOT = Path(__file__).resolve().parents[1]
L = Layout()
pl = PlannerService(L)
tools = [ArmTools(L, k, pl.spaces[k]) for k in range(4)]
tables = [build_cost_table(L, k, pl.rms[k], tools[k]) for k in range(4)]
ref = {r["seed"]: r["cpsat"] for r in json.loads((ROOT / "results/allocation/reference.json").read_text())["scenes"]}
insts = [(1000 + n, make_instance(L, tables, c)) for n, c in enumerate(eval_scenes(50))]
out = {}
for pen in (0.0, 1.0, 3.0, 6.0, 10.0, 20.0, 40.0):
    gaps = [simulate_rule(inst, L, "level1", penalty=pen)["makespan"] / ref[s] - 1 for s, inst in insts]
    out[pen] = {"gap_mean_pct": float(np.mean(gaps) * 100), "gap_median_pct": float(np.median(gaps) * 100)}
    print(pen, out[pen], flush=True)
(ROOT / "results/allocation/penalty_sweep.json").write_text(json.dumps(out, indent=1))
