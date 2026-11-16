"""Lấy mẫu cảnh: N khối 5 cm, 3 màu, trong vùng đặt vật, tâm cách nhau tối thiểu MIN_DIST. Cố định theo seed."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .builder import COLOR_NAMES, CUBE, Layout

MIN_DIST = 0.08


@dataclass(frozen=True)
class CubeSpec:
    name: str
    color: str
    x: float
    y: float
    yaw: float  # [0, π/2): khối đối xứng 90°

    def as_tuple(self, L: Layout):
        return (self.name, (self.x, self.y, L.table_top + CUBE / 2), self.yaw, self.color)


def sample_scene(seed: int, L: Layout | None = None, n: int = 8, min_dist: float = MIN_DIST,
                 max_tries: int = 10000) -> list[CubeSpec]:
    L = L or Layout()
    rng = np.random.default_rng(seed)
    lim = L.zone_half - CUBE / 2 * math.sqrt(2)  # cả khối (kể cả khi xoay 45°) nằm trong vùng
    pts: list[np.ndarray] = []
    for _ in range(max_tries):
        p = rng.uniform(-lim, lim, 2)
        if all(np.linalg.norm(p - q) >= min_dist for q in pts):
            pts.append(p)
            if len(pts) == n:
                break
    else:
        raise RuntimeError("không đặt đủ khối")
    colors = rng.integers(0, len(COLOR_NAMES), n)
    yaws = rng.uniform(0, math.pi / 2, n)
    return [CubeSpec(f"cube{i}", COLOR_NAMES[c], float(p[0]), float(p[1]), float(w))
            for i, (p, c, w) in enumerate(zip(pts, colors, yaws))]


def eval_scenes(count: int = 50, L: Layout | None = None) -> list[list[CubeSpec]]:
    """Bộ cảnh đánh giá mục 2: seed 1000..1000+count-1."""
    return [sample_scene(1000 + i, L) for i in range(count)]
