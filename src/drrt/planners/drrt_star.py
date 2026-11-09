"""dRRT* (Alg. 6, 7, 8). Các chỗ bài chưa rõ được đánh số theo docs/paper_notes.md."""
from __future__ import annotations

import numpy as np

from ..tensor_graph import TensorGraph, Vertex
from .base import TreePlanner

EPS = 1e-12


class DRRTStar(TreePlanner):
    def __init__(self, graph: TensorGraph, rng: np.random.Generator, n_it: int = 1, goal_bias: float = 0.1):
        super().__init__(graph, rng, n_it)
        self.goal_bias = goal_bias
        self.v_last: Vertex | None = graph.start

    def expand(self) -> None:
        self.v_last = self._expand(self.v_last)

    def informed_oracle(self, near: Vertex, goal: np.ndarray | None) -> Vertex:
        """I_d (Alg. 8). goal=None nghĩa là Q_rand = T cho mọi robot."""
        new = []
        for i, (rm, v) in enumerate(zip(self.g.roadmaps, near)):
            adj = rm.adj[v]
            if goal is None or goal[i]:
                table = self.g.h_i[i]
                new.append(int(min(adj.tolist(), key=table.__getitem__)))
            else:
                new.append(int(adj[self.rng.integers(len(adj))]))
        return tuple(new)

    def _expand(self, v_last: Vertex | None) -> Vertex | None:
        """Alg. 7. Trả về đỉnh để mở rộng tham lam ở vòng sau, hoặc None."""
        g, tree = self.g, self.tree
        if v_last is None:
            q_rand, goal = g.sample(self.rng, self.goal_bias)
            near = tree.V[tree.nearest(q_rand)]
        else:
            goal = None
            near = v_last
        new = self.informed_oracle(near, goal)
        if new == near:  # ghi chú 3: Adj chứa chính đỉnh đó, mọi robot đều đứng yên
            return None

        new_id = tree.ids.get(new)
        neighbours = [int(i) for i in tree.neighbours(new) if int(i) != new_id]

        # Dòng 9-10: argmin có ràng buộc va chạm, kiểm tra theo thứ tự chi phí tăng dần (ghi chú 4).
        ranked = sorted((tree.cost[i] + g.edge_cost(tree.V[i], new), i) for i in neighbours)
        best_id = best_cost = None
        for c, i in ranked:
            if new_id is not None and c >= tree.cost[new_id] - EPS:
                break  # đỉnh đã có đường tốt hơn mọi ứng viên còn lại, không cần kiểm tra va chạm
            if g.edge_free(tree.V[i], new):
                best_id, best_cost = i, c
                break
        if best_id is None:
            if new_id is None or tree.parent[new_id] < 0:
                return None
            best_id, best_cost = tree.parent[new_id], tree.cost[new_id]

        # Dòng 11: branch-and-bound (ghi chú 5).
        if best_cost > self.best_cost():
            return None

        # Dòng 12-15.
        if new_id is None:
            new_id = tree.add(new, best_id, best_cost)
        elif best_cost < tree.cost[new_id] - EPS:
            tree.rewire(new_id, best_id, best_cost)

        # Dòng 16-18: dùng đỉnh mới làm cha tốt hơn cho láng giềng.
        for i in neighbours:
            c = tree.cost[new_id] + g.edge_cost(new, tree.V[i])
            if c < tree.cost[i] - EPS and g.edge_free(new, tree.V[i]):
                tree.rewire(i, new_id, c)

        # Dòng 19-21.
        return new if g.h(new) < g.h(tree.V[best_id]) else None
