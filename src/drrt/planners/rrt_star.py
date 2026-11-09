"""RRT* tường minh trong không gian hợp (Karaman & Frazzoli 2011), baseline của Hình 11."""
from __future__ import annotations

import math
import time
from typing import Protocol, Sequence

import numpy as np

from .base import Result

EPS = 1e-12


class CompositeSpace(Protocol):
    lower: np.ndarray
    upper: np.ndarray
    R: int

    def edge_free(self, a: Sequence[float], b: Sequence[float]) -> bool: ...


class CompositeRRTStar:
    """Chi phí là tổng độ dài cung của từng robot, cùng thước đo với dRRT* để so sánh được.

    `step` là độ dài steer trong không gian hợp, mặc định sqrt(R) (mỗi robot đi khoảng 1 đơn vị). Bài không cho tham số RRT*.
    """

    def __init__(self, space: CompositeSpace, start: np.ndarray, goal: np.ndarray, rng: np.random.Generator,
                 free_volume: float, step: float | None = None, goal_bias: float = 0.05, capacity: int = 4096):
        self.space = space
        self.R = space.R
        self.goal = np.asarray(goal, dtype=float)
        self.rng = rng
        self.goal_bias = goal_bias
        self.step = math.sqrt(self.R) if step is None else step
        self.D = len(self.goal)
        zeta = math.pi ** (self.D / 2) / math.gamma(self.D / 2 + 1)
        self.gamma = 2 * (1 + 1 / self.D) ** (1 / self.D) * (free_volume / zeta) ** (1 / self.D)
        self._q = np.empty((capacity, self.D))
        self.cost: list[float] = []
        self.parent: list[int] = []
        self.children: list[list[int]] = []
        self.goal_id: int | None = None
        self.collision_checks = 0
        self._add(np.asarray(start, dtype=float), -1, 0.0)

    def __len__(self) -> int:
        return len(self.cost)

    def _add(self, q: np.ndarray, parent: int, cost: float) -> int:
        i = len(self.cost)
        if i == len(self._q):
            self._q = np.concatenate([self._q, np.empty_like(self._q)])
        self._q[i] = q
        self.cost.append(cost)
        self.parent.append(parent)
        self.children.append([])
        if parent >= 0:
            self.children[parent].append(i)
        return i

    def _rewire(self, i: int, new_parent: int, new_cost: float) -> None:
        self.children[self.parent[i]].remove(i)
        self.parent[i] = new_parent
        self.children[new_parent].append(i)
        delta = new_cost - self.cost[i]
        stack = [i]
        while stack:
            j = stack.pop()
            self.cost[j] += delta
            stack.extend(self.children[j])

    def edge_cost(self, a: np.ndarray, b: np.ndarray) -> float:
        return float(np.linalg.norm((a - b).reshape(self.R, -1), axis=1).sum())

    def _edge_free(self, a: np.ndarray, b: np.ndarray) -> bool:
        self.collision_checks += 1
        return self.space.edge_free(a.tolist(), b.tolist())

    def expand(self) -> None:
        n = len(self.cost)
        q_rand = self.goal if self.rng.random() < self.goal_bias else self.rng.uniform(self.space.lower, self.space.upper)
        dist2 = ((self._q[:n] - q_rand) ** 2).sum(axis=1)
        nearest = int(dist2.argmin())
        gap = math.sqrt(float(dist2[nearest]))
        if gap < 1e-12:
            return
        q_new = q_rand if gap <= self.step else self._q[nearest] + (q_rand - self._q[nearest]) * (self.step / gap)
        if not self._edge_free(self._q[nearest], q_new):
            return

        radius = min(self.gamma * (math.log(n + 1) / (n + 1)) ** (1 / self.D), self.step)
        near = np.flatnonzero(((self._q[:n] - q_new) ** 2).sum(axis=1) <= radius * radius).tolist()
        if nearest not in near:
            near.append(nearest)

        ranked = sorted((self.cost[i] + self.edge_cost(self._q[i], q_new), i) for i in near)
        parent, cost = -1, math.inf
        for c, i in ranked:
            if i == nearest or self._edge_free(self._q[i], q_new):
                parent, cost = i, c
                break
        is_goal = q_new is self.goal
        if is_goal and self.goal_id is not None:
            if cost < self.cost[self.goal_id] - EPS:
                self._rewire(self.goal_id, parent, cost)
            return
        new_id = self._add(q_new, parent, cost)
        if is_goal:
            self.goal_id = new_id
        for i in near:
            if i == parent:
                continue
            c = cost + self.edge_cost(q_new, self._q[i])
            if c < self.cost[i] - EPS and self.parent[i] >= 0 and self._edge_free(q_new, self._q[i]):
                self._rewire(i, new_id, c)

    def path(self) -> list[np.ndarray] | None:
        if self.goal_id is None:
            return None
        out, i = [], self.goal_id
        while i >= 0:
            out.append(self._q[i].copy())
            i = self.parent[i]
        return out[::-1]

    def solve(self, max_iters: int | None = None, time_limit: float | None = None) -> Result:
        if max_iters is None and time_limit is None:
            raise ValueError("cần max_iters hoặc time_limit")
        result = Result()
        t0 = time.perf_counter()
        it = 0
        best = math.inf
        while (max_iters is None or it < max_iters) and (time_limit is None or time.perf_counter() - t0 < time_limit):
            self.expand()
            it += 1
            if self.goal_id is not None and self.cost[self.goal_id] < best - 1e-12:
                best = self.cost[self.goal_id]
                now = time.perf_counter() - t0
                if result.first_time is None:
                    result.first_time, result.first_iter, result.first_cost = now, it, best
                result.history.append((it, now, best))
        result.iterations = it
        result.elapsed = time.perf_counter() - t0
        result.tree_size = len(self)
        result.collision_checks = self.collision_checks
        if self.goal_id is not None:
            result.cost = self.cost[self.goal_id]
            result.path = self.path()
        return result
