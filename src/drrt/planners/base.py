"""Phần dùng chung của các planner trên Ĝ."""
from __future__ import annotations

import abc
import time
from dataclasses import dataclass, field

import numpy as np

from ..tensor_graph import TensorGraph, Vertex
from ..tree import Tree


@dataclass
class Result:
    path: list[Vertex] | None = None
    cost: float = float("inf")
    first_time: float | None = None
    first_iter: int | None = None
    first_cost: float | None = None
    iterations: int = 0
    elapsed: float = 0.0
    tree_size: int = 0
    collision_checks: int = 0
    history: list[tuple[int, float, float]] = field(default_factory=list)  # (vòng lặp, giây, cost)

    @property
    def solved(self) -> bool:
        return self.path is not None


class TreePlanner(abc.ABC):
    """Khung Alg. 1, 3, 6: lặp mở rộng, sau mỗi n_it vòng kiểm tra nghiệm tới T."""

    def __init__(self, graph: TensorGraph, rng: np.random.Generator, n_it: int = 1):
        self.g = graph
        self.rng = rng
        self.n_it = n_it
        self.tree = Tree(graph.start, graph.roadmaps)

    @abc.abstractmethod
    def expand(self) -> None: ...

    def connect_to_target(self) -> None:
        """Bài không định nghĩa (ghi chú 1). Mặc định chỉ kiểm tra T đã có trong cây chưa."""

    def best_cost(self) -> float:
        i = self.tree.ids.get(self.g.target)
        return float("inf") if i is None else self.tree.cost[i]

    def solve(self, max_iters: int | None = None, time_limit: float | None = None,
              stop_on_first: bool = False) -> Result:
        if max_iters is None and time_limit is None:
            raise ValueError("cần max_iters hoặc time_limit")
        result = Result()
        t0 = time.perf_counter()
        it = 0
        best = float("inf")
        while (max_iters is None or it < max_iters) and (time_limit is None or time.perf_counter() - t0 < time_limit):
            self.expand()
            it += 1
            if it % self.n_it == 0:
                self.connect_to_target()
                c = self.best_cost()
                if c < best - 1e-12:
                    best = c
                    now = time.perf_counter() - t0
                    if result.first_time is None:
                        result.first_time, result.first_iter, result.first_cost = now, it, c
                    result.history.append((it, now, c))
                    if stop_on_first:
                        break
        result.iterations = it
        result.elapsed = time.perf_counter() - t0
        result.tree_size = len(self.tree)
        result.collision_checks = self.g.collision_checks
        i = self.tree.ids.get(self.g.target)
        if i is not None:
            result.path = self.tree.path_to(i)
            result.cost = self.tree.cost[i]
        return result
