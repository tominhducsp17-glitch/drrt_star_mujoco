"""Bảng chi phí tính trước cho phân việc (mục 5.2), mỗi tay một bảng.

Ô lưới (bước STEP) trên vùng đặt vật → tư thế chuẩn bị gắp (hướng kẹp 'tự nhiên', khối góc 0). Các tư thế này, HOME và tư
thế tiếp cận chỗ thả giữa của 3 khay được chèn vào roadmap của tay; Dijkstra từ HOME và từ từng khay cho độ dài đường ngắn
nhất H trên roadmap. Thời gian ước lượng = H / VMAX. Ô không có nghiệm IK không va chạm hoặc không nối được: inf.
"""
from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from ..planning.roadmaps import CACHE, ArmRoadmap
from ..scene.builder import CUBE, Layout
from ..skills.ik import HOME
from ..skills.scripted import ArmTools

STEP = 0.05
VMAX = 1.0


@dataclass
class CostTable:
    arm: int
    xs: np.ndarray                 # toạ độ lưới (dùng chung cho x và y)
    from_home: np.ndarray          # (n, n) độ dài trên roadmap HOME → ô
    from_tray: np.ndarray          # (3, n, n) khay j → ô (đối xứng: ô → khay j)

    def cell(self, xy) -> tuple[int, int]:
        i = int(np.clip(np.round((xy[0] - self.xs[0]) / STEP), 0, len(self.xs) - 1))
        j = int(np.clip(np.round((xy[1] - self.xs[0]) / STEP), 0, len(self.xs) - 1))
        return i, j

    def t_home_to(self, xy) -> float:
        return float(self.from_home[self.cell(xy)]) / VMAX

    def t_tray_to(self, j: int, xy) -> float:
        return float(self.from_tray[j][self.cell(xy)]) / VMAX


def _graph(rm) -> csr_matrix:
    rows = [u for u in range(rm.n) for _ in rm.nbrs[u]]
    cols = [int(v) for u in range(rm.n) for v in rm.nbrs[u]]
    data = [rm.dist[u][v] for u, v in zip(rows, cols)]
    return csr_matrix((data, (rows, cols)), shape=(rm.n, rm.n))


def build_cost_table(L: Layout, arm: int, aroad: ArmRoadmap, tools: ArmTools, cache: Path | None = CACHE) -> CostTable:
    lim = L.zone_half
    xs = np.round(np.arange(-lim, lim + 1e-9, STEP), 4)
    key = hashlib.sha1(repr((L, STEP, aroad.n, len(aroad.edges))).encode()).hexdigest()[:10]
    path = cache / f"cost_{key}_a{arm}.npz" if cache else None
    if path is not None and path.exists():
        z = np.load(path)
        return CostTable(arm, z["xs"], z["from_home"], z["from_tray"])
    n = len(xs)
    cells, qs = [], []
    for i, x in enumerate(xs):
        for j, y in enumerate(xs):
            mv = tools.pick((x, y), 0.0)
            if mv is not None:
                cells.append((i, j))
                qs.append(mv.q_top)
    trays = []
    for j in range(3):
        mv = tools.place(L.tray_slots(arm, j)[0])
        trays.append(mv.q_top)
    out = aroad.with_extra([HOME] + trays + qs)
    if out is None:
        raise RuntimeError("không nối được đỉnh của bảng chi phí")
    rm, ids = out
    g = _graph(rm)
    dist = dijkstra(g, directed=False, indices=ids[:4])
    from_home = np.full((n, n), np.inf)
    from_tray = np.full((3, n, n), np.inf)
    for (i, j), v in zip(cells, ids[4:]):
        from_home[i, j] = dist[0, v]
        for t in range(3):
            from_tray[t, i, j] = dist[1 + t, v]
    if path is not None:
        np.savez_compressed(path, xs=xs, from_home=from_home, from_tray=from_tray)
    return CostTable(arm, xs, from_home, from_tray)
