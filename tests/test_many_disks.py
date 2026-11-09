import math

import numpy as np
import pytest

from drrt.checkers.disk2d import CompositeDiskSpace, DiskPairChecker, DiskSpace, perimeter_point, perimeter_swap_query
from drrt.planners import AODRRT, CompositeRRTStar, DRRTStar, implicit_astar
from drrt.roadmap import Roadmap, r_star
from drrt.tensor_graph import TensorGraph

from helpers import assert_valid_path


def many_disk_graph(R: int, n: int, seed: int, offset: float = 0.0) -> TensorGraph:
    rng = np.random.default_rng(seed)
    space = DiskSpace()
    radius = r_star(n, 2, space.free_area)
    roadmaps = []
    for s, t in perimeter_swap_query(R, offset):
        while True:
            rm = Roadmap.build(space, n, rng, radius=radius, required=[s, t])
            if rm.connected(0, 1):
                roadmaps.append(rm)
                break
    return TensorGraph(roadmaps, DiskPairChecker(0.2), (0,) * R, (1,) * R)


def test_perimeter_point_walks_the_square_and_opposite_is_point_symmetric():
    assert perimeter_point(0.0) == pytest.approx((-0.2, -0.2))
    assert perimeter_point(0.25) == pytest.approx((9.2, -0.2))
    assert perimeter_point(0.5) == pytest.approx((9.2, 9.2))
    assert perimeter_point(0.75) == pytest.approx((-0.2, 9.2))
    for f in np.linspace(0, 1, 41):
        a, b = perimeter_point(f), perimeter_point(f + 0.5)
        assert a[0] + b[0] == pytest.approx(9.0) and a[1] + b[1] == pytest.approx(9.0)


@pytest.mark.parametrize("R", range(3, 11))
def test_perimeter_query_is_free_and_starts_do_not_touch(R):
    space = DiskSpace()
    query = perimeter_swap_query(R, offset=0.013)
    for s, t in query:
        assert space.config_free(s) and space.config_free(t)
    starts = [q[0] for q in query]
    goals = [q[1] for q in query]
    checker = DiskPairChecker(0.2)
    assert checker.edge_free(starts, starts)
    assert checker.edge_free(goals, goals)


@pytest.mark.parametrize("R", [3, 5])
def test_drrt_star_solves_many_disks_with_a_valid_path(R):
    g = many_disk_graph(R, 50, seed=R)
    r = DRRTStar(g, np.random.default_rng(0)).solve(max_iters=5000)
    assert r.solved
    assert_valid_path(g, r.path, r.cost)
    assert r.cost >= sum(table[0] for table in g.h_i) - 1e-9


def test_r3_drrt_star_matches_astar_on_small_roadmaps():
    optimum = implicit_astar(many_disk_graph(3, 15, seed=2), time_limit=120).cost
    assert math.isfinite(optimum)
    g = many_disk_graph(3, 15, seed=2)
    r = DRRTStar(g, np.random.default_rng(0)).solve(max_iters=40000)
    assert r.cost == pytest.approx(optimum, rel=1e-6)


def test_astar_time_limit_is_respected_inside_a_single_huge_expansion():
    r = implicit_astar(many_disk_graph(10, 50, seed=1), time_limit=0.2)
    assert r.elapsed < 5.0


def test_ao_drrt_solves_three_disks():
    g = many_disk_graph(3, 50, seed=4)
    r = AODRRT(g, np.random.default_rng(0)).solve(max_iters=5000)
    assert r.solved
    assert_valid_path(g, r.path, r.cost)


def _rrt_star_problem():
    base = DiskSpace()
    space = CompositeDiskSpace(base, 2, 0.2)
    start = np.array([0.0, 0.0, 9.0, 9.0])
    goal = np.array([9.0, 9.0, 0.0, 0.0])
    return space, start, goal, base.free_area ** 2


def test_rrt_star_tree_is_consistent_and_path_is_collision_free():
    space, start, goal, volume = _rrt_star_problem()
    planner = CompositeRRTStar(space, start, goal, np.random.default_rng(1), volume)
    r = planner.solve(max_iters=6000)
    assert r.solved
    assert np.allclose(r.path[0], start) and np.allclose(r.path[-1], goal)
    total = 0.0
    for a, b in zip(r.path, r.path[1:]):
        assert space.edge_free(a.tolist(), b.tolist())
        total += planner.edge_cost(a, b)
    assert total == pytest.approx(r.cost)
    for i in range(1, len(planner)):
        p = planner.parent[i]
        assert planner.cost[i] == pytest.approx(planner.cost[p] + planner.edge_cost(planner._q[p], planner._q[i]))
        assert i in planner.children[p]


def test_rrt_star_cost_only_improves():
    space, start, goal, volume = _rrt_star_problem()
    r = CompositeRRTStar(space, start, goal, np.random.default_rng(2), volume).solve(max_iters=8000)
    costs = [c for _, _, c in r.history]
    assert r.solved and costs == sorted(costs, reverse=True)
