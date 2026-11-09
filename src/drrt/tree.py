"""Cây tìm kiếm trên Ĝ: thêm đỉnh, rewire có lan truyền chi phí, láng giềng gần nhất, Adj(V) ∩ T."""
from __future__ import annotations

from typing import Sequence

import numpy as np

from .roadmap import Roadmap
from .tensor_graph import Vertex


class Tree:
    def __init__(self, root: Vertex, roadmaps: Sequence[Roadmap], capacity: int = 1024):
        self.roadmaps = list(roadmaps)
        self.R = len(self.roadmaps)
        self.flat_dim = sum(rm.d for rm in self.roadmaps)
        self.V: list[Vertex] = []
        self.ids: dict[Vertex, int] = {}
        self.parent: list[int] = []
        self.cost: list[float] = []
        self.children: list[list[int]] = []
        self._idx = np.empty((capacity, self.R), dtype=np.int32)
        self._q = np.empty((capacity, self.flat_dim))
        self.add(root, -1, 0.0)

    def __len__(self) -> int:
        return len(self.V)

    def __contains__(self, v: Vertex) -> bool:
        return v in self.ids

    def add(self, v: Vertex, parent: int, cost: float) -> int:
        i = len(self.V)
        if i == len(self._idx):
            self._idx = np.concatenate([self._idx, np.empty_like(self._idx)])
            self._q = np.concatenate([self._q, np.empty_like(self._q)])
        self._idx[i] = v
        self._q[i] = np.concatenate([rm.q[a] for rm, a in zip(self.roadmaps, v)])
        self.V.append(v)
        self.ids[v] = i
        self.parent.append(parent)
        self.cost.append(cost)
        self.children.append([])
        if parent >= 0:
            self.children[parent].append(i)
        return i

    def rewire(self, i: int, new_parent: int, new_cost: float) -> None:
        """Đổi cha của đỉnh i và cập nhật chi phí cho cả nhánh con (bài không nói rõ, ghi chú 6)."""
        old = self.parent[i]
        if old >= 0:
            self.children[old].remove(i)
        self.parent[i] = new_parent
        self.children[new_parent].append(i)
        delta = new_cost - self.cost[i]
        stack = [i]
        while stack:
            j = stack.pop()
            self.cost[j] += delta
            stack.extend(self.children[j])

    def nearest(self, q: np.ndarray) -> int:
        n = len(self.V)
        return int(((self._q[:n] - q) ** 2).sum(axis=1).argmin())

    def neighbours(self, v: Vertex) -> np.ndarray:
        """Adj(v, Ĝ) ∩ T, tính từ phía cây nên không phải liệt kê tích ∏|Adj(v_i)| (ghi chú 2)."""
        for rm, a in zip(self.roadmaps, v):
            rm.mask[rm.adj[a]] = True
        idx = self._idx
        n = len(self.V)
        found = np.flatnonzero(self.roadmaps[0].mask[idx[:n, 0]])
        for i in range(1, self.R):
            found = found[self.roadmaps[i].mask[idx[found, i]]]
        for rm, a in zip(self.roadmaps, v):
            rm.mask[rm.adj[a]] = False
        return found

    def path_to(self, i: int) -> list[Vertex]:
        path = []
        while i >= 0:
            path.append(self.V[i])
            i = self.parent[i]
        path.reverse()
        return path
