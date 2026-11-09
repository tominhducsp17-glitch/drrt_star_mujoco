"""dRRT (Alg. 1, 2, 5) và ao-dRRT (Alg. 3, 4)."""
from __future__ import annotations

import numpy as np

from ..tensor_graph import TensorGraph, Vertex
from .base import TreePlanner

EPS = 1e-12


class DRRT(TreePlanner):
    """Oracle góc O_d. rewire=False là dRRT, rewire=True là ao-dRRT.

    Vì bài không định nghĩa Connect_to_Target (ghi chú 1), sau mỗi n_it vòng thử cho mọi robot cùng đi theo đường ngắn
    nhất của mình từ đỉnh vừa thêm tới T. Đây là bản đơn giản hơn local connector của dRRT gốc (Solovey et al. 2015).
    """

    rewire = False

    def __init__(self, graph: TensorGraph, rng: np.random.Generator, n_it: int = 100, goal_bias: float = 0.0):
        super().__init__(graph, rng, n_it)
        self.goal_bias = goal_bias
        self._last_added = 0
        self._hops = [rm.next_hop(t) for rm, t in zip(graph.roadmaps, graph.target)]

    def direction_oracle(self, near: Vertex, q_rand: np.ndarray) -> Vertex:
        """O_d (Alg. 5). Góc tới chính đỉnh đó không xác định, trong khi chú thích 1 cho phép robot đứng yên, nên mỗi robot
        đứng yên với xác suất 1/(bậc+1). Nếu không, các nghiệm có robot chờ sẽ không bao giờ được xét."""
        new = []
        d = self.g.d
        for i, (rm, v) in enumerate(zip(self.g.roadmaps, near)):
            nbrs = rm.nbrs[v]
            if len(nbrs) == 0 or self.rng.random() < 1 / (len(nbrs) + 1):
                new.append(v)
                continue
            direction = q_rand[i * d:(i + 1) * d] - rm.q[v]
            new.append(int(nbrs[int(np.argmax(rm.unit_vectors(v) @ direction))]))
        return tuple(new)

    def expand(self) -> None:
        """Alg. 2 và Alg. 4."""
        g, tree = self.g, self.tree
        q_rand, _ = g.sample(self.rng, self.goal_bias)
        near_id = tree.nearest(q_rand)
        near = tree.V[near_id]
        new = self.direction_oracle(near, q_rand)
        if new == near:
            return
        self._add_or_rewire(near_id, new)

    def _add_or_rewire(self, parent_id: int, new: Vertex) -> bool:
        g, tree = self.g, self.tree
        parent = tree.V[parent_id]
        new_id = tree.ids.get(new)
        cost = tree.cost[parent_id] + g.edge_cost(parent, new)
        if new_id is None:
            if not g.edge_free(parent, new):
                return False
            self._last_added = tree.add(new, parent_id, cost)
        elif self.rewire and cost < tree.cost[new_id] - EPS and g.edge_free(parent, new):
            tree.rewire(new_id, parent_id, cost)
        return True

    def connect_to_target(self) -> None:
        g, tree = self.g, self.tree
        walk = [tree.V[self._last_added]]
        while walk[-1] != g.target:
            step = tuple(int(hop[v]) if v != t else v for hop, v, t in zip(self._hops, walk[-1], g.target))
            if min(step) < 0 or not g.edge_free(walk[-1], step):
                return
            walk.append(step)
        last_added = self._last_added
        for a, b in zip(walk, walk[1:]):
            self._add_or_rewire(tree.ids[a], b)
        self._last_added = last_added


class AODRRT(DRRT):
    rewire = True
