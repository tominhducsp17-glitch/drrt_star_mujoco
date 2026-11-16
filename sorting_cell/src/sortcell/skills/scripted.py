"""Tư thế và đoạn chuyển động cục bộ cho gắp và thả bằng script (gripper chúc xuống).

Gắp: tư thế chuẩn bị gắp q_pre (TCP cao hơn tâm khối D_H) → đi thẳng xuống tới TCP ở tâm khối → đóng → đi thẳng lên q_pre.
Thả: tư thế tiếp cận q_app (TCP cao hơn đáy khay APPROACH_H) → xuống tới PLACE_H → mở → lên lại q_app.

Điều kiện chấp nhận:
- q_pre, q_app: tự do trong model offline (lề CLEARANCE với môi trường, có khối ảo) để làm đỉnh roadmap;
- mọi điểm của đoạn đi xuống: không chạm môi trường trong model không có khối ảo, lề 0 (má kẹp phải xuống sát bàn).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco as mj
import numpy as np

from ..planning.models import ArmSpace
from ..scene.builder import CUBE, Layout, arm_of_geom, build_spec
from .ik import ArmKin, down_rot, seeds

D_H = 0.16          # khoảng bàn giao (mục 5.4: thử 8 và 16 cm); M1 dùng 16 cm
GRASP_DZ = 0.0      # TCP so với tâm khối khi kẹp
PLACE_H = 0.045     # TCP trên đáy khay khi thả: đáy khối cao hơn đáy khay 2 cm
APPROACH_H = 0.30


@dataclass
class LocalMove:
    q_top: np.ndarray        # đỉnh roadmap (q_pre hoặc q_app)
    down: np.ndarray         # (n+1, 6) từ q_top xuống điểm thấp nhất
    yaw: float               # góc kẹp (phương đóng mở má kẹp)


class ArmTools:
    def __init__(self, L: Layout, arm: int, space: ArmSpace | None = None):
        self.L, self.arm = L, arm
        self.space = space or ArmSpace(L, arm)
        self.kin = ArmKin(self.space.model, arm)
        # model không khối ảo, không lề: kiểm tra đoạn đi xuống
        self._env = build_spec(L, "exec", arms=[arm], cameras=False).compile()
        self._d = mj.MjData(self._env)
        self._idx = ArmKin(self._env, arm).idx
        self.rng = np.random.default_rng(arm)

    def touches_env(self, q) -> bool:
        d = self._d
        d.qpos[self._idx] = q
        mj.mj_kinematics(self._env, d)
        mj.mj_collision(self._env, d)
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist < 0 and (arm_of_geom(self._env, c.geom1) >= 0 or arm_of_geom(self._env, c.geom2) >= 0):
                return True
        return False

    def _top(self, pos, R, tries: int = 8):
        for s in seeds(self.L, self.arm, pos) + [self.rng.uniform(-np.pi, np.pi, 6) for _ in range(tries)]:
            q = self.kin.solve(pos, R, s)
            if q is not None and self.space.config_free(q):
                return q
        return None

    def _local(self, top, bottom, yaws, tries: int = 8) -> LocalMove | None:
        for yaw in yaws:
            R = down_rot(yaw)
            q = self._top(top, R, tries)
            if q is None:
                continue
            qs = self.kin.line(q, top, bottom, R)
            if qs is not None and not any(self.touches_env(x) for x in qs[1:]):
                return LocalMove(q, qs, yaw)
        return None

    def grasp_yaws(self, xy, cube_yaw: float) -> list[float]:
        """4 hướng kẹp tương đương (đối xứng 90°), hướng gần với 'tự nhiên' nhất (vuông góc với tia đế → khối) trước,
        để cổ tay ít phải xoay."""
        x, y, _, _ = self.L.base(self.arm)
        ref = math.atan2(xy[1] - y, xy[0] - x) + math.pi / 2
        c = [cube_yaw + j * math.pi / 2 for j in range(4)]
        return sorted(c, key=lambda a: abs((a - ref + math.pi) % (2 * math.pi) - math.pi))

    def pick_options(self, xy, cube_yaw: float, d_h: float = D_H):
        """Các cách gắp (mỗi hướng kẹp một cách), theo thứ tự ưu tiên; sinh lười."""
        zc = self.L.table_top + CUBE / 2 + GRASP_DZ
        bottom = np.array([xy[0], xy[1], zc])
        for yaw in self.grasp_yaws(xy, cube_yaw):
            mv = self._local(bottom + [0, 0, d_h], bottom, [yaw])
            if mv is not None:
                yield mv

    def pick(self, xy, cube_yaw: float, d_h: float = D_H) -> LocalMove | None:
        return next(self.pick_options(xy, cube_yaw, d_h), None)

    def place_options(self, slot):
        slot = np.asarray(slot, float)
        yaw = self.L.base(self.arm)[3]
        for a in np.radians([0, 90, 45, 135]):
            mv = self._local(slot + [0, 0, APPROACH_H], slot + [0, 0, PLACE_H], [yaw + a], tries=16)
            if mv is not None:
                yield mv

    def place(self, slot) -> LocalMove | None:
        return next(self.place_options(slot), None)
