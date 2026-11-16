"""Khối nào gắp được về nguyên tắc (M4 việc 1): dùng pose THẬT, không perception, không chuyển động.

Lặp: một khối là gắp được nếu có ít nhất một tay và một hướng kẹp mà đoạn đi xuống của gripper (mở ở GRIP_OPEN) không
tới gần khối khác hơn CUBE_MARGIN, với các khối còn lại trên bàn. Bỏ mọi khối gắp được rồi xét lại. Khối còn lại khi
không bỏ được thêm là khối không thể gắp bằng gripper hai ngón nếu không đẩy hay gạt (ví dụ cụm 2 × 2). Đây là chặn
trên cho số khối mà cell có thể phân loại; so với số khối cell báo "bị vây".

Chú ý: giữ nguyên chỗ các khối khác (không mô phỏng việc khối bị xê dịch khi gắp khối bên cạnh).

    ../.venv/bin/python experiments/graspability.py --out results/m4/graspability.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco as mj
import numpy as np

from sortcell.control.executor import CellSim
from sortcell.scene.builder import Layout
from sortcell.scene.sampling import sample_cluster_scene
from sortcell.skills.scripted import ArmTools

FAR = np.array([10.0, 10.0, -1.0])


def analyse(seed: int, L: Layout, tools: list[ArmTools]) -> dict:
    cubes = sample_cluster_scene(seed, L)
    sim = CellSim(L, cubes)
    sim.wait(0.3)
    remaining = set(range(len(cubes)))
    order = []
    while True:
        free = []
        for i in sorted(remaining):
            pos, yaw = sim.cube_pose(i)
            ok = False
            for k in range(L.n_arms):
                for mv in tools[k].pick_options(pos[:2], yaw):
                    if not sim.descent_touches(k, mv.down, i):
                        ok = True
                        break
                if ok:
                    break
            if ok:
                free.append(i)
        if not free:
            break
        order.append(free)
        for i in free:  # đưa khối đã gắp ra khỏi bàn
            jnt = sim.m.body_jntadr[sim.cube_body[i]]
            adr = sim.m.jnt_qposadr[jnt]
            sim.d.qpos[adr:adr + 3] = FAR + [i, 0, 0]
        mj.mj_forward(sim.m, sim.d)
        remaining -= set(free)
    return {"seed": seed, "rounds": order, "infeasible": sorted(remaining)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, default=50)
    ap.add_argument("--first-seed", type=int, default=2000)
    ap.add_argument("--out", default="results/m4/graspability.json")
    a = ap.parse_args()
    L = Layout()
    tools = [ArmTools(L, k) for k in range(L.n_arms)]
    res = [analyse(s, L, tools) for s in range(a.first_seed, a.first_seed + a.scenes)]
    n_inf = sum(len(r["infeasible"]) for r in res)
    out = {"scenes": len(res), "cubes": 8 * len(res), "infeasible": n_inf,
           "scenes_with_infeasible": sum(bool(r["infeasible"]) for r in res),
           "need_order": sum(len(r["rounds"]) > 1 for r in res), "per_scene": res}
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k != "per_scene"}, indent=1))


if __name__ == "__main__":
    main()
