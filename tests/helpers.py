from __future__ import annotations

import numpy as np

from drrt.checkers.disk2d import DiskPairChecker, DiskSpace
from drrt.roadmap import Roadmap, r_star
from drrt.tensor_graph import TensorGraph

SWAP = ((0.0, 0.0), (9.0, 9.0))


def _connected_roadmap(space: DiskSpace, n: int, rng: np.random.Generator, required) -> Roadmap:
    radius = r_star(n, 2, space.free_area)
    while True:
        roadmap = Roadmap.build(space, n, rng, radius=radius, required=required)
        if roadmap.connected(0, 1):
            return roadmap


def swap_graph(n: int, seed: int, cost: str = "sum") -> TensorGraph:
    """Hai đĩa đổi chỗ (0,0) ↔ (9,9) trong môi trường Hình 9, chỉ nhận roadmap nối được s_i với t_i để test luôn có
    nghiệm thật. s_i, t_i là hai đỉnh đầu của mỗi roadmap."""
    rng = np.random.default_rng(seed)
    space = DiskSpace()
    a = _connected_roadmap(space, n, rng, SWAP)
    b = _connected_roadmap(space, n, rng, SWAP[::-1])
    return TensorGraph([a, b], DiskPairChecker(0.2), (0, 0), (1, 1), cost=cost)


def assert_valid_path(graph: TensorGraph, path, cost: float) -> None:
    assert path[0] == graph.start
    assert path[-1] == graph.target
    for u, v in zip(path, path[1:]):
        assert graph.adjacent(u, v), f"không kề: {u} {v}"
        assert graph.edge_free(u, v), f"va chạm: {u} {v}"
    assert abs(graph.path_cost(path) - cost) < 1e-6
