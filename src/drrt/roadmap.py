"""PRM của một robot (Mục 4.1) và heuristic H trên roadmap đó."""
from __future__ import annotations

import math
from typing import Protocol, Sequence

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components, dijkstra
from scipy.spatial import cKDTree


class RobotSpace(Protocol):
    """Không gian cấu hình của một robot, chỉ xét robot với môi trường."""

    lower: np.ndarray
    upper: np.ndarray

    def config_free(self, q: Sequence[float]) -> bool: ...

    def edge_free(self, a: Sequence[float], b: Sequence[float]) -> bool: ...


def r_star(n: int, d: int, free_volume: float, eta: float = 0.0) -> float:
    """Bán kính kết nối r*(n) của Theorem 1, với d là số chiều của một robot."""
    zeta = math.pi ** (d / 2) / math.gamma(d / 2 + 1)
    gamma = (1 + eta) * 2 * (1 / d) ** (1 / d) * (free_volume / zeta) ** (1 / d)
    return gamma * (math.log(n) / n) ** (1 / d)


def k_prm_star(n: int, d: int) -> int:
    """k của k-PRM* (Karaman & Frazzoli 2011). Không nằm trong chứng minh của Theorem 1."""
    return math.ceil(math.e * (1 + 1 / d) * math.log(n))


class Roadmap:
    """Đồ thị vô hướng. `nbrs[v]` không chứa v; `adj[v]` là Adj(v) của bài nên có chứa v (robot được đứng yên)."""

    def __init__(self, q: np.ndarray, edges: Sequence[tuple[int, int]]):
        self.q = q
        self.n, self.d = q.shape
        # Tuple số thực để vòng lặp nóng của checker giải tích không phải đi qua numpy.
        self.qt: list[tuple[float, ...]] = [tuple(map(float, row)) for row in q]
        neighbours: list[list[int]] = [[] for _ in range(self.n)]
        self.dist: list[dict[int, float]] = [{v: 0.0} for v in range(self.n)]
        for u, v in edges:
            w = float(np.linalg.norm(q[u] - q[v]))
            neighbours[u].append(v)
            neighbours[v].append(u)
            self.dist[u][v] = w
            self.dist[v][u] = w
        self.nbrs = [np.array(sorted(x), dtype=np.int32) for x in neighbours]
        self.adj = [np.append(x, v).astype(np.int32) for v, x in enumerate(self.nbrs)]
        self.n_edges = len(edges)
        self.mask = np.zeros(self.n, dtype=bool)  # bộ đệm dùng lại cho phép tính Adj ∩ T
        self._unit: dict[int, np.ndarray] = {}
        self._h: dict[int, tuple[np.ndarray, np.ndarray]] = {}

    @classmethod
    def build(cls, space: RobotSpace, n: int, rng: np.random.Generator, radius: float | None = None,
              k: int | None = None, required: Sequence[Sequence[float]] = ()) -> "Roadmap":
        """Lấy n mẫu tự do rồi nối theo `radius` hoặc `k`. `required` (s_i, t_i) nằm ở các chỉ số đầu tiên."""
        if (radius is None) == (k is None):
            raise ValueError("chọn đúng một trong radius, k")
        points = [np.asarray(p, dtype=float) for p in required]
        for p in points:
            if not space.config_free(p):
                raise ValueError(f"cấu hình bắt buộc không tự do: {p}")
        while len(points) < n:
            p = rng.uniform(space.lower, space.upper)
            if space.config_free(p):
                points.append(p)
        q = np.array(points)
        kd = cKDTree(q)
        if radius is not None:
            pairs = kd.query_pairs(radius, output_type="set")
        else:
            _, idx = kd.query(q, k=min(k + 1, len(q)))
            pairs = {(min(int(u), int(v)), max(int(u), int(v))) for u, row in enumerate(idx) for v in row if u != v}
        edges = [(u, v) for u, v in sorted(pairs) if space.edge_free(q[u], q[v])]
        return cls(q, edges)

    def unit_vectors(self, v: int) -> np.ndarray:
        """Vector đơn vị từ v tới từng phần tử của nbrs[v], dùng cho oracle góc O_d."""
        cached = self._unit.get(v)
        if cached is None:
            delta = self.q[self.nbrs[v]] - self.q[v]
            cached = delta / np.linalg.norm(delta, axis=1, keepdims=True)
            self._unit[v] = cached
        return cached

    def _dijkstra(self, target: int) -> tuple[np.ndarray, np.ndarray]:
        result = self._h.get(target)
        if result is None:
            rows = [u for u in range(self.n) for _ in self.nbrs[u]]
            cols = [int(v) for u in range(self.n) for v in self.nbrs[u]]
            data = [self.dist[u][v] for u, v in zip(rows, cols)]
            graph = csr_matrix((data, (rows, cols)), shape=(self.n, self.n))
            dist, pred = dijkstra(graph, directed=False, indices=target, return_predecessors=True)
            result = (dist, pred)
            self._h[target] = result
        return result

    def heuristic(self, target: int) -> np.ndarray:
        """H(v, T_i): khoảng cách ngắn nhất trên roadmap tới target. Mục 5.2 dùng Johnson all-pairs; mục tiêu cố định
        thì một lần Dijkstra là đủ."""
        return self._dijkstra(target)[0]

    def next_hop(self, target: int) -> np.ndarray:
        """Đỉnh kế tiếp trên đường ngắn nhất tới target (-9999 nếu không có)."""
        return self._dijkstra(target)[1]

    def connected(self, a: int, b: int) -> bool:
        rows = [u for u in range(self.n) for _ in self.nbrs[u]]
        cols = [int(v) for u in range(self.n) for v in self.nbrs[u]]
        graph = csr_matrix((np.ones(len(rows)), (rows, cols)), shape=(self.n, self.n))
        _, labels = connected_components(graph, directed=False)
        return bool(labels[a] == labels[b])
