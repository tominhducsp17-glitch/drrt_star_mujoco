"""Roadmap của từng tay, dựng một lần cho cảnh cố định rồi cache; chèn đỉnh tạm (cấu hình hiện tại, đích) khi lập kế hoạch.

Mẫu của roadmap gồm hai phần:
- mẫu "tác vụ": nghiệm IK ở tư thế chuẩn bị gắp trên bàn (vị trí, góc, độ cao ngẫu nhiên) và tư thế tiếp cận khay,
  để roadmap dày ở đúng vùng cell dùng; PRM đều trong 6 chiều rất thưa ở đó;
- mẫu đều trong hộp khớp, để có đường vòng.
Nối k láng giềng gần nhất (k của k-PRM*), cạnh kiểm tra bằng ArmSpace (tay với môi trường, khối ảo, lề an toàn).
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from drrt.roadmap import Roadmap, k_prm_star

from ..scene.builder import CUBE, Layout
from ..skills.ik import HOME, down_rot
from ..skills.scripted import APPROACH_H, ArmTools
from .models import CLEARANCE, ArmSpace

ROOT = Path(__file__).resolve().parents[3]
CACHE = ROOT / "cache"
VERSION = 1  # tăng khi đổi cách dựng để cache cũ không còn dùng


def task_samples(tools: ArmTools, n: int, rng: np.random.Generator, tray_frac: float = 0.25) -> list[np.ndarray]:
    L, k = tools.L, tools.arm
    out = [HOME.copy()]
    tries = 0
    while len(out) < n and tries < 50 * n:
        tries += 1
        if rng.random() < tray_frac:
            j = rng.integers(3)
            slot = L.tray_slots(k, j)[rng.integers(5)]
            pos = slot + [0, 0, APPROACH_H + rng.uniform(-0.05, 0.10)]
        else:
            lim = L.table_half - CUBE / 2
            pos = np.array([*rng.uniform(-lim, lim, 2), L.table_top + CUBE / 2 + rng.uniform(0.08, 0.25)])
        q = tools._top(pos, down_rot(rng.uniform(-math.pi, math.pi)), tries=2)
        if q is not None:
            out.append(q)
    return out


def _key(L: Layout, n_task: int, n_uniform: int, seed: int, arm: int) -> str:
    h = hashlib.sha1(repr((VERSION, L, CLEARANCE, n_task, n_uniform, seed)).encode()).hexdigest()[:10]
    return f"rm_{h}_a{arm}_t{n_task}_u{n_uniform}_s{seed}"


def build_roadmap(L: Layout, arm: int, n_task: int = 1500, n_uniform: int = 1500, seed: int = 0,
                  cache: Path | None = CACHE) -> Roadmap:
    path = cache / f"{_key(L, n_task, n_uniform, seed, arm)}.npz" if cache else None
    if path is not None and path.exists():
        return Roadmap.load(path)
    rng = np.random.default_rng([seed, arm])
    tools = ArmTools(L, arm)
    req = task_samples(tools, n_task, rng)
    n = len(req) + n_uniform
    rm = Roadmap.build(tools.space, n, rng, k=k_prm_star(n, 6), required=req)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        rm.save(path)
    return rm


def _component(rm: Roadmap, v: int) -> np.ndarray:
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import connected_components
    rows = [u for u in range(rm.n) for _ in rm.nbrs[u]]
    cols = [int(w) for u in range(rm.n) for w in rm.nbrs[u]]
    g = csr_matrix((np.ones(len(rows)), (rows, cols)), shape=(rm.n, rm.n))
    _, labels = connected_components(g, directed=False)
    return labels == labels[v]


class ArmRoadmap:
    """Roadmap gốc của một tay + chèn đỉnh tạm. Chỉ giữ thành phần liên thông chứa HOME (đỉnh 0)."""

    def __init__(self, base: Roadmap, space: ArmSpace, k: int | None = None):
        keep = _component(base, 0)
        idx = np.flatnonzero(keep)
        remap = -np.ones(base.n, int)
        remap[idx] = np.arange(len(idx))
        edges = [(int(remap[u]), int(remap[v])) for u, v in base.edges() if keep[u] and keep[v]]
        self.q = base.q[idx]
        self.edges = edges
        self.space = space
        self.k = k or k_prm_star(len(idx), 6)
        self.kd = cKDTree(self.q)
        self.base = Roadmap(self.q, edges)  # dựng một lần; with_extra chỉ vá thêm đỉnh tạm

    @property
    def n(self) -> int:
        return len(self.q)

    def connect(self, q: np.ndarray) -> list[tuple[int, float]]:
        """Các đỉnh của roadmap gốc nối thẳng được tới q (đã kiểm tra va chạm), gần trước."""
        dist, idx = self.kd.query(q, k=min(3 * self.k, self.n))
        out = []
        for dd, i in zip(dist, idx):
            if self.space.edge_free(q, self.q[i]):
                out.append((int(i), float(dd)))
                if len(out) >= self.k:
                    break
        return out

    def with_extra(self, qs: list[np.ndarray]) -> tuple[Roadmap, list[int]] | None:
        """Roadmap mới có thêm các đỉnh qs (trùng nhau thì gộp). None nếu có đỉnh không nối được vào roadmap."""
        extra: list[np.ndarray] = []
        ids = []
        for q in qs:
            for j, e in enumerate(extra):
                if np.allclose(e, q, atol=1e-9):
                    ids.append(self.n + j)
                    break
            else:
                extra.append(np.asarray(q, float))
                ids.append(self.n + len(extra) - 1)
        new_edges = []
        for j, q in enumerate(extra):
            links = self.connect(q)
            if not links:
                return None
            new_edges += [(i, self.n + j) for i, _ in links]
        for a in range(len(extra)):
            for b in range(a + 1, len(extra)):
                if self.space.edge_free(extra[a], extra[b]):
                    new_edges.append((self.n + a, self.n + b))
        return _patched(self.base, extra, new_edges), ids


def _patched(base: Roadmap, extra: list[np.ndarray], new_edges: list[tuple[int, int]]) -> Roadmap:
    """Roadmap = base + đỉnh extra + cạnh new_edges, cho ra đúng các trường như Roadmap(q, edges) nhưng không dựng lại
    phần của base (Roadmap.__init__ tính độ dài từng cạnh trong vòng lặp Python: ~0,17 s cho 46 nghìn cạnh)."""
    rm = Roadmap.__new__(Roadmap)
    n0 = base.n
    rm.q = np.vstack([base.q] + extra) if extra else base.q
    rm.n, rm.d = rm.q.shape
    rm.qt = base.qt + [tuple(map(float, row)) for row in extra]
    rm.dist = list(base.dist) + [{v: 0.0} for v in range(n0, rm.n)]
    nbr_add: dict[int, list[int]] = {}
    touched = set()
    for u, v in new_edges:
        w = float(np.linalg.norm(rm.q[u] - rm.q[v]))
        for x, y in ((u, v), (v, u)):
            if x < n0 and x not in touched:
                rm.dist[x] = dict(rm.dist[x])  # chép trước khi sửa để không đụng tới base
                touched.add(x)
            rm.dist[x][y] = w
            nbr_add.setdefault(x, []).append(y)
    rm.nbrs = list(base.nbrs) + [np.zeros(0, np.int32) for _ in extra]
    for x, ys in nbr_add.items():
        rm.nbrs[x] = np.array(sorted(set(rm.nbrs[x].tolist()) | set(ys)), dtype=np.int32)
    rm.adj = list(base.adj) + [None] * len(extra)
    for x in set(nbr_add) | set(range(n0, rm.n)):
        rm.adj[x] = np.append(rm.nbrs[x], x).astype(np.int32)
    rm.n_edges = base.n_edges + len(new_edges)
    rm.mask = np.zeros(rm.n, dtype=bool)
    rm._unit = {}
    rm._h = {}
    return rm
