import heapq
import itertools

import numpy as np
import pytest

from drrt.planners import AODRRT, DRRT, DRRTStar, implicit_astar
from drrt.tensor_graph import TensorGraph

from helpers import assert_valid_path, swap_graph

N = 30
SEEDS = [11, 12, 13]


def _dijkstra_on_explicit_tensor_graph(g: TensorGraph) -> float:
    dist = {g.start: 0.0}
    heap = [(0.0, g.start)]
    while heap:
        c, v = heapq.heappop(heap)
        if v == g.target:
            return c
        if c > dist[v]:
            continue
        for u in itertools.product(*(rm.adj[a].tolist() for rm, a in zip(g.roadmaps, v))):
            if u == v:
                continue
            nc = c + g.edge_cost(v, u)
            if nc < dist.get(u, float("inf")) and g.edge_free(v, u):
                dist[u] = nc
                heapq.heappush(heap, (nc, u))
    return float("inf")


def _assert_tree_is_consistent(g: TensorGraph, tree) -> None:
    for i, v in enumerate(tree.V):
        p = tree.parent[i]
        if p < 0:
            assert v == g.start and tree.cost[i] == 0
            continue
        assert tree.cost[i] == pytest.approx(tree.cost[p] + g.edge_cost(tree.V[p], v))
        assert g.edge_free(tree.V[p], v)
        assert i in tree.children[p]
        seen = set()
        while i >= 0:
            assert i not in seen, "cây có chu trình"
            seen.add(i)
            i = tree.parent[i]


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("cost", ["sum", "euclid"])
def test_astar_is_optimal(seed, cost):
    r = implicit_astar(swap_graph(N, seed, cost))
    assert r.solved
    assert_valid_path(swap_graph(N, seed, cost), r.path, r.cost)
    assert r.cost == pytest.approx(_dijkstra_on_explicit_tensor_graph(swap_graph(N, seed, cost)))


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("cost", ["sum", "euclid"])
def test_drrt_star_converges_to_the_optimum_of_the_tensor_roadmap(seed, cost):
    """Kiểm chứng trực tiếp Theorem 2 trên roadmap nhỏ."""
    optimum = implicit_astar(swap_graph(N, seed, cost)).cost
    g = swap_graph(N, seed, cost)
    planner = DRRTStar(g, np.random.default_rng(seed))
    r = planner.solve(max_iters=60000)
    assert r.solved
    assert_valid_path(g, r.path, r.cost)
    _assert_tree_is_consistent(g, planner.tree)
    assert r.cost == pytest.approx(optimum, rel=1e-6)


@pytest.mark.parametrize("seed", SEEDS)
def test_ao_drrt_converges_and_keeps_a_consistent_tree(seed):
    optimum = implicit_astar(swap_graph(N, seed)).cost
    g = swap_graph(N, seed)
    planner = AODRRT(g, np.random.default_rng(seed))
    r = planner.solve(max_iters=700000)  # ao-dRRT hội tụ chậm hơn dRRT* cỡ một bậc
    assert r.solved
    assert_valid_path(g, r.path, r.cost)
    _assert_tree_is_consistent(g, planner.tree)
    assert r.cost == pytest.approx(optimum, rel=1e-6)


@pytest.mark.parametrize("seed", SEEDS)
def test_drrt_returns_a_valid_first_solution(seed):
    optimum = implicit_astar(swap_graph(N, seed)).cost
    g = swap_graph(N, seed)
    r = DRRT(g, np.random.default_rng(seed)).solve(max_iters=30000, stop_on_first=True)
    assert r.solved
    assert_valid_path(g, r.path, r.cost)
    assert r.cost >= optimum - 1e-9


def test_drrt_star_first_solution_is_nearly_immediate_without_interaction():
    # Hai đĩa đi hai phía nên đường ngắn nhất riêng lẻ ít va nhau: tham lam tới đích gần như ngay.
    g = swap_graph(100, 21)
    planner = DRRTStar(g, np.random.default_rng(0))
    r = planner.solve(max_iters=2000, stop_on_first=True)
    assert r.solved
    assert_valid_path(g, r.path, r.cost)
    assert r.first_iter < 500


def test_branch_and_bound_never_worsens_the_best_cost():
    g = swap_graph(60, 22)
    r = DRRTStar(g, np.random.default_rng(0)).solve(max_iters=5000)
    assert r.solved
    costs = [c for _, _, c in r.history]
    assert costs == sorted(costs, reverse=True)
    assert r.cost == costs[-1]
