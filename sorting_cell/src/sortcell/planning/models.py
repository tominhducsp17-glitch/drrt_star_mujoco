"""Kiểm tra va chạm cho planner, tách như drrt.mujoco_cell nhưng trên tay có gripper và khối ảo (vật đang mang).

- offline (roadmap của tay k): tay k + môi trường. Lề an toàn đặt trên geom môi trường. Khối ảo (contype=2) chỉ va với
  môi trường (conaffinity của môi trường thêm bit 2), không va với chính gripper của nó.
- online: mọi tay, môi trường tắt; tay k (cả khối ảo) mang bit 1<<k, mỗi geom nửa lề.
- full: model đầy đủ cùng lề, chỉ dùng để kiểm chứng phép tách không bỏ sót va chạm.
Gripper giữ ở tư thế mở hẳn (qpos = 0), là tư thế chiếm chỗ nhiều nhất.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import mujoco as mj
import numpy as np

from drrt.mujoco_cell import DOF, _bisection_order, _qpos_indices, steps_between

from ..scene.builder import Layout, arm_of_geom, build_spec

CLEARANCE = 0.02
RESOLUTION = 0.05


def _is_phantom(m, g) -> bool:
    return (mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, g) or "").endswith("held_phantom")


def offline_model(L: Layout, arm: int, clearance: float = CLEARANCE) -> mj.MjModel:
    m = build_spec(L, "plan", arms=[arm], cameras=False).compile()
    for g in range(m.ngeom):
        if not (m.geom_contype[g] or m.geom_conaffinity[g]):
            continue
        if arm_of_geom(m, g) < 0:
            m.geom_margin[g] = clearance
            m.geom_conaffinity[g] |= 2
        else:
            m.geom_margin[g] = 0.0
    return m


def online_model(L: Layout, clearance: float = CLEARANCE) -> mj.MjModel:
    m = build_spec(L, "plan", environment=False, cameras=False).compile()
    everyone = (1 << L.n_arms) - 1
    for g in range(m.ngeom):
        if not (m.geom_contype[g] or m.geom_conaffinity[g]):
            continue
        k = arm_of_geom(m, g)
        if k < 0:
            m.geom_contype[g] = m.geom_conaffinity[g] = 0
            continue
        m.geom_contype[g] = 1 << k
        m.geom_conaffinity[g] = everyone & ~(1 << k)
        m.geom_margin[g] = clearance / 2
    return m


class ArmSpace:
    """RobotSpace của drrt.roadmap cho một tay: giới hạn khớp, va chạm với môi trường, tự va chạm."""

    def __init__(self, L: Layout, arm: int, clearance: float = CLEARANCE, resolution: float = RESOLUTION):
        self.L, self.arm = L, arm
        self.model = offline_model(L, arm, clearance)
        self.data = mj.MjData(self.model)
        self.idx = _qpos_indices(self.model, arm)
        self.lower, self.upper = np.full(DOF, -np.pi), np.full(DOF, np.pi)
        self.resolution = resolution
        self.checks = 0

    def config_free(self, q) -> bool:
        self.checks += 1
        self.data.qpos[self.idx] = q
        mj.mj_kinematics(self.model, self.data)
        mj.mj_collision(self.model, self.data)
        return self.data.ncon == 0

    def edge_free(self, a, b) -> bool:
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = steps_between(a, b, self.resolution)
        return all(self.config_free(a + (b - a) * (k / n)) for k in _bisection_order(n))

    def path_free(self, qs) -> bool:
        return all(self.edge_free(a, b) for a, b in zip(qs, qs[1:]))


class PairChecker:
    """PairwiseChecker của drrt.tensor_graph: chỉ va chạm giữa các tay."""

    def __init__(self, L: Layout, clearance: float = CLEARANCE, resolution: float = RESOLUTION):
        self.model = online_model(L, clearance)
        self.data = mj.MjData(self.model)
        self.n = L.n_arms
        self.idx = np.concatenate([_qpos_indices(self.model, k) for k in range(L.n_arms)])
        self.resolution = resolution
        self.checks = 0

    def config_free(self, q) -> bool:
        self.checks += 1
        self.data.qpos[self.idx] = np.asarray(q, float).reshape(-1)
        mj.mj_kinematics(self.model, self.data)
        mj.mj_collision(self.model, self.data)
        return self.data.ncon == 0

    def edge_free(self, a, b) -> bool:
        a = np.concatenate([np.asarray(x, float) for x in a])
        b = np.concatenate([np.asarray(x, float) for x in b])
        n = steps_between(a, b, self.resolution)
        return all(self.config_free(a + (b - a) * (k / n)) for k in _bisection_order(n))


@dataclass
class FullChecker:
    """Model đầy đủ (mọi tay, môi trường, khối ảo) với cùng lề, để kiểm chứng phép tách."""

    L: Layout
    clearance: float = CLEARANCE
    model: mj.MjModel = field(init=False)

    def __post_init__(self):
        self.model = build_spec(self.L, "plan", cameras=False).compile()
        m = self.model
        for g in range(m.ngeom):
            if m.geom_contype[g] or m.geom_conaffinity[g]:
                m.geom_margin[g] = self.clearance
                if arm_of_geom(m, g) < 0 or _is_phantom(m, g):
                    m.geom_conaffinity[g] |= 2  # để khối ảo va với môi trường và với tay khác
                if arm_of_geom(m, g) >= 0 and not _is_phantom(m, g):
                    m.geom_conaffinity[g] |= 2
        self.data = mj.MjData(m)
        self.idx = np.concatenate([_qpos_indices(m, k) for k in range(self.L.n_arms)])

    def contacts(self, q) -> list[tuple[int, int]]:
        m, d = self.model, self.data
        d.qpos[self.idx] = q
        mj.mj_kinematics(m, d)
        mj.mj_collision(m, d)
        out = []
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = arm_of_geom(m, c.geom1), arm_of_geom(m, c.geom2)
            if a == b and a >= 0:
                if _is_phantom(m, c.geom1) or _is_phantom(m, c.geom2):
                    continue  # khối ảo với chính gripper của nó
                if c.dist >= 0.0:
                    continue
            elif c.dist >= self.clearance:
                continue
            out.append((a, b))
        return out
