"""Kiểm tra va chạm cho một truy vấn dRRT* mà chỉ các tay đang di chuyển nằm trong Ĝ.

Tay không di chuyển là vật cản cố định, mỗi tay cho bởi một TẬP cấu hình:
- tay đứng yên: một cấu hình;
- tay đang ở pha gắp/thả (vùng dành riêng, mục 5.3): mọi cấu hình trên đoạn quét cục bộ của nó, nên tay khác tránh được
  cả thân tay lẫn hình trụ quanh vật, bất kể tay đó đang ở điểm nào của pha gắp.

Va chạm của cạnh (a, b) trong Ĝ = (mỗi tay di chuyển i: đoạn a_i→b_i không va các tập cố định) và (các tay di chuyển không
va nhau). Phần đầu chỉ phụ thuộc cạnh của một tay nên được cache theo cạnh: đó là chỗ tiết kiệm chính.
Mặt nạ bit contype/conaffinity trên model online (planning.models) chọn cặp nào được xét.
"""
from __future__ import annotations

import mujoco as mj
import numpy as np

from drrt.mujoco_cell import _bisection_order, steps_between

from .models import RESOLUTION, PairChecker


class QueryChecker:
    def __init__(self, pair: PairChecker, moving: list[int], fixed: dict[int, list[np.ndarray]],
                 resolution: float = RESOLUTION):
        self.pair = pair
        self.m, self.d = pair.model, pair.data
        self.moving = list(moving)
        self.fixed = {k: [np.asarray(q, float) for q in v] for k, v in fixed.items()}
        self.K = max((len(v) for v in self.fixed.values()), default=0)
        self.res = resolution
        n = pair.n
        self.qidx = [pair.idx[6 * k:6 * k + 6] for k in range(n)]
        self._orig = (self.m.geom_contype.copy(), self.m.geom_conaffinity.copy())
        arm_of = np.full(self.m.ngeom, -1)
        for g in range(self.m.ngeom):
            ct = self._orig[0][g]
            if ct:
                arm_of[g] = int(ct).bit_length() - 1
        self.arm_geoms = [np.flatnonzero(arm_of == k) for k in range(n)]
        # mặt nạ "giữa các tay di chuyển"
        ct, ca = np.zeros_like(self._orig[0]), np.zeros_like(self._orig[1])
        mv_bits = sum(1 << k for k in self.moving)
        for k in self.moving:
            ct[self.arm_geoms[k]] = 1 << k
            ca[self.arm_geoms[k]] = mv_bits & ~(1 << k)
        self.mask_mm = (ct, ca)
        # mặt nạ "tay i với các tay cố định" (các tay cố định không xét với nhau)
        self.mask_fixed = {}
        fx_bits = sum(1 << k for k in self.fixed)
        for i in self.moving:
            ct, ca = np.zeros_like(self._orig[0]), np.zeros_like(self._orig[1])
            ct[self.arm_geoms[i]] = 1 << i
            ca[self.arm_geoms[i]] = fx_bits
            for f in self.fixed:
                ct[self.arm_geoms[f]] = 1 << f
                ca[self.arm_geoms[f]] = 1 << i
            self.mask_fixed[i] = (ct, ca)
        self._cache: dict[tuple, bool] = {}
        self.checks = 0

    def restore(self) -> None:
        self.m.geom_contype[:], self.m.geom_conaffinity[:] = self._orig

    def _collide(self, mask) -> bool:
        self.m.geom_contype[:], self.m.geom_conaffinity[:] = mask
        mj.mj_kinematics(self.m, self.d)
        mj.mj_collision(self.m, self.d)
        self.checks += 1
        return self.d.ncon > 0

    # ------------------------------------------------------------ tay i với các tay cố định
    def arm_config_free(self, i: int, q) -> bool:
        if not self.fixed:
            return True
        self.d.qpos[self.qidx[i]] = q
        for j in range(self.K):
            for f, qs in self.fixed.items():
                self.d.qpos[self.qidx[f]] = qs[min(j, len(qs) - 1)]
            if self._collide(self.mask_fixed[i]):
                return False
        return True

    def arm_edge_free(self, i: int, a, b) -> bool:
        key = (i, tuple(a), tuple(b))
        hit = self._cache.get(key)
        if hit is None:
            a, b = np.asarray(a, float), np.asarray(b, float)
            n = steps_between(a, b, self.res)
            hit = all(self.arm_config_free(i, a + (b - a) * (k / n)) for k in _bisection_order(n))
            self._cache[key] = hit
            self._cache[(i, tuple(b), tuple(a))] = hit
        return hit

    # ------------------------------------------------------------ giữa các tay di chuyển
    def moving_config_free(self, qs) -> bool:
        if len(self.moving) < 2:
            return True
        for k, q in zip(self.moving, qs):
            self.d.qpos[self.qidx[k]] = q
        return not self._collide(self.mask_mm)

    # ------------------------------------------------------------ PairwiseChecker của drrt.tensor_graph
    def edge_free(self, a, b) -> bool:
        for i, x, y in zip(self.moving, a, b):
            if not self.arm_edge_free(i, x, y):
                return False
        if len(self.moving) < 2:
            return True
        A = np.concatenate([np.asarray(x, float) for x in a])
        B = np.concatenate([np.asarray(x, float) for x in b])
        n = steps_between(A, B, self.res)
        for k in _bisection_order(n):
            q = A + (B - A) * (k / n)
            if not self.moving_config_free([q[6 * j:6 * j + 6] for j in range(len(self.moving))]):
                return False
        return True

    def config_free(self, qs) -> bool:
        return all(self.arm_config_free(i, q) for i, q in zip(self.moving, qs)) and self.moving_config_free(qs)
