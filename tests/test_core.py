import itertools
import math

import numpy as np
import pytest

from drrt.checkers.disk2d import DiskSpace
from drrt.roadmap import Roadmap, k_prm_star, r_star
from drrt.tree import Tree

from helpers import swap_graph


def test_r_star_matches_hand_calculation_for_one_ur10e_arm():
    # Thu khớp về ±π: thể tích (2π)^6, d = 6, n = 1000 (bảng trong docs/robot_survey.md).
    assert r_star(1000, 6, (2 * math.pi) ** 6) == pytest.approx(3.09, abs=0.01)
    assert k_prm_star(1000, 6) == 22


def test_roadmap_is_symmetric_and_edges_are_free():
    space = DiskSpace()
    rng = np.random.default_rng(0)
    rm = Roadmap.build(space, 80, rng, radius=2.5, required=[(0.0, 0.0), (9.0, 9.0)])
    assert np.allclose(rm.q[0], (0, 0)) and np.allclose(rm.q[1], (9, 9))
    for u in range(rm.n):
        assert u in rm.adj[u] and u not in rm.nbrs[u]
        for v in rm.nbrs[u]:
            assert u in rm.nbrs[v]
            assert rm.dist[u][int(v)] == pytest.approx(np.linalg.norm(rm.q[u] - rm.q[v]))
            assert space.edge_free(rm.q[u], rm.q[v])


def test_k_nearest_roadmap_connects_each_vertex_to_at_most_k_candidates_symmetrically():
    rm = Roadmap.build(DiskSpace(rects=[]), 60, np.random.default_rng(0), k=5)
    assert all(len(x) >= 5 for x in rm.nbrs)
    assert all(u in rm.nbrs[int(v)] for u in range(rm.n) for v in rm.nbrs[u])


def test_heuristic_is_consistent_on_the_roadmap():
    rm = Roadmap.build(DiskSpace(), 80, np.random.default_rng(1), radius=2.5, required=[(0.0, 0.0), (9.0, 9.0)])
    h = rm.heuristic(1)
    assert h[1] == 0
    for u in range(rm.n):
        for v in rm.nbrs[u]:
            if np.isfinite(h[u]):
                assert h[u] <= rm.dist[u][int(v)] + h[v] + 1e-9


def test_tree_neighbours_match_brute_force():
    g = swap_graph(40, 3)
    tree = Tree(g.start, g.roadmaps)
    rng = np.random.default_rng(0)
    for _ in range(300):
        v = tuple(int(rng.integers(rm.n)) for rm in g.roadmaps)
        if v not in tree:
            tree.add(v, 0, 1.0)
    for _ in range(50):
        v = tuple(int(rng.integers(rm.n)) for rm in g.roadmaps)
        expected = {i for i, u in enumerate(tree.V) if all(b in rm.dist[a] for rm, a, b in zip(g.roadmaps, v, u))}
        assert set(tree.neighbours(v).tolist()) == expected


def test_tree_neighbours_for_many_robots_use_the_same_rule():
    g = swap_graph(30, 4)
    roadmaps = [g.roadmaps[0], g.roadmaps[1], g.roadmaps[0]]
    tree = Tree((0, 0, 0), roadmaps)
    rng = np.random.default_rng(0)
    for _ in range(300):
        v = tuple(int(rng.integers(rm.n)) for rm in roadmaps)
        if v not in tree:
            tree.add(v, 0, 1.0)
    v = (0, 0, 0)
    expected = {i for i, u in enumerate(tree.V) if all(b in rm.dist[a] for rm, a, b in zip(roadmaps, v, u))}
    assert set(tree.neighbours(v).tolist()) == expected


def test_rewire_propagates_cost_through_the_subtree():
    g = swap_graph(30, 5)
    tree = Tree((0, 0), g.roadmaps)
    a = tree.add((1, 0), 0, 5.0)
    b = tree.add((2, 0), a, 6.0)
    c = tree.add((3, 0), b, 8.0)
    d = tree.add((4, 0), 0, 1.0)
    tree.rewire(a, d, 2.0)
    assert tree.parent[a] == d and a in tree.children[d] and a not in tree.children[0]
    assert tree.cost[a] == 2.0 and tree.cost[b] == 3.0 and tree.cost[c] == 5.0
    assert tree.path_to(c) == [(0, 0), (4, 0), (1, 0), (2, 0), (3, 0)]


def test_tensor_adjacency_includes_waiting():
    g = swap_graph(40, 6)
    v = g.start
    neighbour = int(g.roadmaps[0].nbrs[v[0]][0])
    assert g.adjacent(v, (neighbour, v[1]))
    assert g.edge_cost(v, (neighbour, v[1])) == pytest.approx(g.roadmaps[0].dist[v[0]][neighbour])
    assert g.edge_cost(v, v) == 0


def test_euclid_cost_never_exceeds_sum_cost():
    a = swap_graph(40, 7, cost="sum")
    b = swap_graph(40, 7, cost="euclid")
    pairs = itertools.product(a.roadmaps[0].adj[0].tolist(), a.roadmaps[1].adj[0].tolist())
    for u in pairs:
        assert b.edge_cost((0, 0), u) <= a.edge_cost((0, 0), u) + 1e-12
