"""Roadmap tích tensor Ĝ biểu diễn ngầm (Mục 4.1) và hàm chi phí."""
from __future__ import annotations

import math
from typing import Protocol, Sequence

import numpy as np

from .roadmap import Roadmap

Vertex = tuple[int, ...]
Config = Sequence[Sequence[float]]


class PairwiseChecker(Protocol):
    """Va chạm giữa các robot. Va chạm với môi trường đã được xử lý trong từng roadmap."""

    def edge_free(self, a: Config, b: Config) -> bool:
        """Chuyển động thẳng đồng thời trong không gian hợp từ a tới b có tự do không."""
        ...


class TensorGraph:
    """Một truy vấn (S, T) trên Ĝ. Đỉnh là tuple chỉ số đỉnh của từng roadmap.

    cost="sum": tổng độ dài cung. cost="euclid": độ dài cung trong không gian hợp. Cả hai cộng dồn được theo cạnh.
    """

    def __init__(self, roadmaps: Sequence[Roadmap], checker: PairwiseChecker, start: Vertex, target: Vertex,
                 cost: str = "sum"):
        if cost not in ("sum", "euclid"):
            raise ValueError(f"cost không hỗ trợ: {cost}")
        self.roadmaps = list(roadmaps)
        self.R = len(self.roadmaps)
        self.d = self.roadmaps[0].d
        self.checker = checker
        self.start = tuple(start)
        self.target = tuple(target)
        self.cost_kind = cost
        self.h_i = [rm.heuristic(t).tolist() for rm, t in zip(self.roadmaps, self.target)]
        self.target_q = np.concatenate([rm.q[t] for rm, t in zip(self.roadmaps, self.target)])
        self.lower = np.concatenate([np.tile(rm.q.min(axis=0), 1) for rm in self.roadmaps])
        self.upper = np.concatenate([np.tile(rm.q.max(axis=0), 1) for rm in self.roadmaps])
        self._edge_cache: dict[tuple[Vertex, Vertex], bool] = {}
        self.collision_checks = 0

    def config(self, v: Vertex) -> list[tuple[float, ...]]:
        return [rm.qt[i] for rm, i in zip(self.roadmaps, v)]

    def flat(self, v: Vertex) -> np.ndarray:
        return np.concatenate([rm.q[i] for rm, i in zip(self.roadmaps, v)])

    def edge_cost(self, u: Vertex, v: Vertex) -> float:
        if self.cost_kind == "sum":
            return sum(rm.dist[a][b] for rm, a, b in zip(self.roadmaps, u, v))
        return math.sqrt(sum(rm.dist[a][b] ** 2 for rm, a, b in zip(self.roadmaps, u, v)))

    def h(self, v: Vertex) -> float:
        """H trong không gian hợp, không vượt quá chi phí thật (ghi chú 7 trong docs/paper_notes.md)."""
        if self.cost_kind == "sum":
            return sum(table[i] for table, i in zip(self.h_i, v))
        return math.sqrt(sum(table[i] ** 2 for table, i in zip(self.h_i, v)))

    def adjacent(self, u: Vertex, v: Vertex) -> bool:
        return all(b in rm.dist[a] for rm, a, b in zip(self.roadmaps, u, v))

    def edge_free(self, u: Vertex, v: Vertex) -> bool:
        key = (u, v) if u <= v else (v, u)
        cached = self._edge_cache.get(key)
        if cached is None:
            self.collision_checks += 1
            cached = self.checker.edge_free(self.config(u), self.config(v))
            self._edge_cache[key] = cached
        return cached

    def sample(self, rng: np.random.Generator, goal_bias: float) -> tuple[np.ndarray, np.ndarray]:
        """Q_rand phẳng và mặt nạ các robot được lấy mẫu trúng đích (focused sampling, Mục 5.2)."""
        q = np.empty(self.R * self.d)
        goal = rng.random(self.R) < goal_bias
        for i, rm in enumerate(self.roadmaps):
            slot = slice(i * self.d, (i + 1) * self.d)
            q[slot] = rm.q[self.target[i]] if goal[i] else rng.uniform(rm.q.min(axis=0), rm.q.max(axis=0))
        return q, goal

    def path_cost(self, path: Sequence[Vertex]) -> float:
        return sum(self.edge_cost(a, b) for a, b in zip(path, path[1:]))
