"""P0: đo tốc độ collision và tỉ lệ va chạm của cell UR10e theo khoảng cách đế."""
from __future__ import annotations

import argparse
import collections
import time
from pathlib import Path

import mujoco as mj
import numpy as np

UR10E = Path(__file__).resolve().parents[1] / "third_party/universal_robots_ur10e/ur10e.xml"


def build_cell(spacing: float, n_arms: int = 2) -> mj.MjModel:
    """n_arms tay UR10e cách đều trên đường tròn đường kính `spacing`, đế quay vào tâm, thêm sàn."""
    spec = mj.MjSpec()
    spec.worldbody.add_geom(name="floor", type=mj.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1])
    for k in range(n_arms):
        angle = 2 * np.pi * k / n_arms
        frame = spec.worldbody.add_frame(
            pos=[spacing / 2 * np.cos(angle), spacing / 2 * np.sin(angle), 0],
            quat=[np.cos(angle / 2), 0, 0, np.sin(angle / 2)],
        )
        spec.attach(mj.MjSpec.from_file(str(UR10E)), frame=frame, prefix=f"a{k}_")
    return spec.compile()


def probe(spacing: float, n_arms: int, samples: int, limit: float, seed: int = 0) -> dict:
    m = build_cell(spacing, n_arms)
    d = mj.MjData(m)
    rng = np.random.default_rng(seed)
    q = rng.uniform(-limit, limit, (samples, m.nq))

    t0 = time.perf_counter()
    for row in q:
        d.qpos[:] = row
        mj.mj_kinematics(m, d)
        mj.mj_collision(m, d)
    us = (time.perf_counter() - t0) / samples * 1e6

    def owner(geom: int) -> str:
        name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, m.geom_bodyid[geom]) or "world"
        return name.split("_")[0] if name[0] == "a" else "world"

    hits = collections.Counter()
    for row in q:
        d.qpos[:] = row
        mj.mj_kinematics(m, d)
        mj.mj_collision(m, d)
        kinds = set()
        for i in range(d.ncon):
            if d.contact[i].dist < 0:
                a, b = owner(d.contact[i].geom1), owner(d.contact[i].geom2)
                kinds.add("world" if "world" in (a, b) else ("self" if a == b else "inter"))
        hits["free" if not kinds else "any"] += 1
        hits.update(kinds)
    collidable = sum(1 for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g])
    return {"us": us, "collidable": collidable, **{k: v / samples for k, v in hits.items()}}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=20000)
    args = ap.parse_args()
    print("arms spacing limit   us/check geoms   free   self  floor  inter")
    for n_arms, spacing, limit in [(1, 0.0, np.pi), (2, 2.4, np.pi), (2, 1.8, np.pi), (2, 1.4, np.pi), (2, 1.0, np.pi),
                                   (2, 1.4, 2 * np.pi), (4, 2.0, np.pi), (4, 1.4, np.pi)]:
        r = probe(spacing, n_arms, args.samples, limit)
        print(f"{n_arms:4d} {spacing:7.1f} {limit:5.2f} {r['us']:10.1f} {r['collidable']:5d} "
              f"{r.get('free', 0):6.3f} {r.get('self', 0):6.3f} {r.get('world', 0):6.3f} {r.get('inter', 0):6.3f}")


if __name__ == "__main__":
    main()
