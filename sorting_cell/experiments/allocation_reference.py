"""So sánh phân việc trong mô hình lập lịch của Oberthür et al.: CP-SAT (tối ưu theo mô hình) với luật mức 0 và mức 1.

Ra results/allocation/: makespan theo mô hình của từng luật trên 50 cảnh, khoảng cách tới tối ưu.
"""
import json
from pathlib import Path

import numpy as np

from sortcell.allocation.costs import build_cost_table
from sortcell.allocation.milp import make_instance, simulate_rule, solve_cpsat
from sortcell.planning.service import PlannerService
from sortcell.scene.builder import Layout
from sortcell.scene.sampling import eval_scenes
from sortcell.skills.scripted import ArmTools

OUT = Path(__file__).resolve().parents[1] / "results/allocation"

L = Layout()
pl = PlannerService(L)
tools = [ArmTools(L, k, pl.spaces[k]) for k in range(4)]
tables = [build_cost_table(L, k, pl.rms[k], tools[k]) for k in range(4)]
rows = []
for n, cubes in enumerate(eval_scenes(50)):
    inst = make_instance(L, tables, cubes)
    opt = solve_cpsat(inst)
    l0 = simulate_rule(inst, L, "level0")
    l1 = simulate_rule(inst, L, "level1")
    rows.append({"seed": 1000 + n, "cpsat": opt["makespan"], "bound": opt["bound"], "status": opt["status"],
                 "solve_s": opt["time"], "level0": l0["makespan"], "level1": l1["makespan"],
                 "conflict_pairs": int(inst.conflict.sum() // 2)})
    print(rows[-1], flush=True)
OUT.mkdir(parents=True, exist_ok=True)
opt = np.array([r["cpsat"] for r in rows])
s = {"n": len(rows), "optimal": sum(r["status"] == "OPTIMAL" for r in rows),
     "cpsat_mean": float(opt.mean()), "solve_s_mean": float(np.mean([r["solve_s"] for r in rows])),
     "solve_s_max": float(np.max([r["solve_s"] for r in rows]))}
for rule in ("level0", "level1"):
    v = np.array([r[rule] for r in rows])
    gap = v / opt - 1
    s[rule] = {"makespan_mean": float(v.mean()), "gap_mean_pct": float(gap.mean() * 100),
               "gap_median_pct": float(np.median(gap) * 100), "gap_max_pct": float(gap.max() * 100)}
(OUT / "reference.json").write_text(json.dumps({"summary": s, "scenes": rows}, indent=1))
print(json.dumps(s, indent=1))
