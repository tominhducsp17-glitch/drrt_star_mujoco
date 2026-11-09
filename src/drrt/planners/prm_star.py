"""PRM* dựng tường minh trong không gian hợp, baseline N²-PRM* của Mục 8.1."""
from __future__ import annotations

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from ..roadmap import Roadmap


def composite_sum_graph(roadmap: Roadmap, R: int) -> csr_matrix:
    """Đồ thị của roadmap hợp với trọng số là tổng độ dài cung của R robot."""
    d = roadmap.d // R
    rows, cols, data = [], [], []
    for u in range(roadmap.n):
        for v in roadmap.nbrs[u]:
            delta = (roadmap.q[u] - roadmap.q[v]).reshape(R, d)
            rows.append(u)
            cols.append(int(v))
            data.append(float(np.linalg.norm(delta, axis=1).sum()))
    return csr_matrix((data, (rows, cols)), shape=(roadmap.n, roadmap.n))


def shortest_path_cost(graph: csr_matrix, source: int, target: int) -> float:
    return float(dijkstra(graph, directed=False, indices=source)[target])
