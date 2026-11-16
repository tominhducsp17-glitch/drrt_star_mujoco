"""Độ chịu sai số pose của script gắp (điểm quyết định M3): tỉ lệ gắp thành công theo sai số vị trí và góc của pose
ước lượng.

Mỗi lần thử: một khối ở vị trí và góc ngẫu nhiên trong vùng đặt vật, tay gần nhất gắp bằng đúng kỹ năng của cell
(ArmTools.pick, kiểm tra đoạn đi xuống, LineSeg, thời gian T_DOWN/T_CLOSE/T_UP, GRIP_OPEN/GRIP_CLOSE) nhưng theo pose bị
lệch: vị trí lệch đúng `e` mm theo hướng ngẫu nhiên, góc lệch đúng `a` độ với dấu ngẫu nhiên. Tay bắt đầu ở tư thế chuẩn
bị gắp (bỏ qua transit). Thành công: khối nằm trong gripper và cao hơn mặt bàn ≥ 5 cm sau khi nhấc và sau 1 s giữ thêm.
Cùng seed cho mọi ô lưới nên các ô so sánh được với nhau.

    MUJOCO_GL=egl .venv/bin/python experiments/grasp_tolerance.py --trials 40
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from sortcell.control.arm_fsm import LIFT_OK, T_CLOSE, T_DOWN, T_UP
from sortcell.control.executor import GRIP_CLOSE, GRIP_OPEN, CellSim, LineSeg, Step
from sortcell.planning.models import ArmSpace
from sortcell.scene.builder import CUBE, Layout
from sortcell.scene.sampling import CubeSpec
from sortcell.skills.scripted import ArmTools

POS_MM = [0, 5, 10, 15, 20, 25, 30]
YAW_DEG = [0, 10, 20, 30, 45]

_tools = None


def _init():
    global _tools
    L = Layout()
    _tools = [ArmTools(L, k, ArmSpace(L, k)) for k in range(L.n_arms)]


def trial(args) -> dict:
    e_mm, a_deg, n = args
    L = _tools[0].L
    rng = np.random.default_rng([n, 7])
    lim = L.zone_half - CUBE / 2 * math.sqrt(2)
    xy = rng.uniform(-lim, lim, 2)
    yaw = float(rng.uniform(0, math.pi / 2))
    k = int(np.argmin([np.hypot(*(xy - np.array(L.base(j)[:2]))) for j in range(L.n_arms)]))
    th = rng.uniform(0, 2 * math.pi)  # hướng lệch và dấu góc: theo seed của lần thử, không theo ô lưới
    sgn = 1 if rng.random() < 0.5 else -1
    est_xy = xy + e_mm / 1000 * np.array([math.cos(th), math.sin(th)])
    est_yaw = yaw + sgn * math.radians(a_deg)
    cube = CubeSpec("cube0", "red", float(xy[0]), float(xy[1]), yaw)
    sim = CellSim(L, [cube])
    mv = _tools[k].pick(est_xy, est_yaw)
    if mv is None:
        return {"e": e_mm, "a": a_deg, "n": n, "ok": False, "reason": "no_ik"}
    sim.set_arm(k, mv.q_top)
    sim.d.ctrl[sim.grip[k]] = GRIP_OPEN
    import mujoco as mj
    mj.mj_forward(sim.m, sim.d)
    sim.wait(0.3)
    sim.allowed[0] = k
    steps = [Step(LineSeg(mv.down, T_DOWN), GRIP_OPEN), Step(None, GRIP_CLOSE, T_CLOSE),
             Step(LineSeg(mv.down[::-1], T_UP), GRIP_CLOSE)]
    sim.run_scripts({k: steps})
    lifted = sim.lifted_cube(k, LIFT_OK) == 0
    sim.wait(1.0)
    kept = sim.lifted_cube(k, LIFT_OK) == 0
    p, yaw_end = sim.cube_pose(0)
    return {"e": e_mm, "a": a_deg, "n": n, "ok": bool(lifted and kept), "lifted": bool(lifted), "kept": bool(kept),
            "coll": dict(sim.coll.steps), "arm": k, "cube_z": float(p[2])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--out", default="results/m3/grasp_tolerance.json")
    a = ap.parse_args()
    jobs = [(e, y, n) for e, y in itertools.product(POS_MM, YAW_DEG) for n in range(a.trials)]
    t = time.perf_counter()
    with ProcessPoolExecutor(a.workers, initializer=_init) as ex:
        res = list(ex.map(trial, jobs, chunksize=4))
    grid = {}
    for e, y in itertools.product(POS_MM, YAW_DEG):
        sel = [r for r in res if r["e"] == e and r["a"] == y]
        grid[f"{e}_{y}"] = {"e_mm": e, "yaw_deg": y, "n": len(sel), "success": sum(r["ok"] for r in sel),
                            "rate": sum(r["ok"] for r in sel) / len(sel),
                            "collisions": sum(bool(r.get("coll")) for r in sel),
                            "no_ik": sum(r.get("reason") == "no_ik" for r in sel)}
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"trials_per_cell": a.trials, "wall_s": time.perf_counter() - t, "grid": grid,
                               "raw": res}, indent=1))
    print("e\\yaw " + " ".join(f"{y:>6}" for y in YAW_DEG))
    for e in POS_MM:
        print(f"{e:>5} " + " ".join(f"{grid[f'{e}_{y}']['rate']:>6.2f}" for y in YAW_DEG))
    print("va chạm:", {k: v["collisions"] for k, v in grid.items() if v["collisions"]})


if __name__ == "__main__":
    main()
