import numpy as np
import pytest

from sortcell.allocation.hitbox import Ellipse
from sortcell.allocation.milp import Instance, simulate_rule, solve_cpsat
from sortcell.control.staggered import subdivide
from sortcell.planning.models import ArmSpace, PairChecker
from sortcell.planning.query import QueryChecker
from sortcell.scene.builder import Layout
from sortcell.skills.ik import HOME

L = Layout()


def test_ellipse_contains_its_segment_and_detects_crossing():
    e = Ellipse((0, 0), (1, 0), w=0.1)
    assert e.contains(np.array([[0.5, 0.0], [0.0, 0.0], [1.0, 0.0]])).all()
    assert not e.contains(np.array([[0.5, 0.2]])).any()
    assert e.intersects(Ellipse((0.5, -1), (0.5, 1), w=0.1))
    assert not e.intersects(Ellipse((0, 1), (1, 1), w=0.1))


def test_subdivide_keeps_endpoints_and_bounds_steps():
    p = [np.zeros(24), np.full(24, 2.0), np.full(24, 2.1)]
    out = subdivide(p, 0.8)
    assert np.allclose(out[0], p[0]) and np.allclose(out[-1], p[-1])
    assert max(np.max(np.abs(b - a)) for a, b in zip(out, out[1:])) <= 0.8 + 1e-12


@pytest.fixture(scope="module")
def pair():
    return PairChecker(L)


def _pairs_in_contact(pair, q) -> set[tuple[int, int]]:
    import mujoco as mj
    d = pair.data
    d.qpos[pair.idx] = q
    mj.mj_kinematics(pair.model, d)
    mj.mj_collision(pair.model, d)
    arm = lambda g: int(pair.model.geom_contype[g]).bit_length() - 1  # noqa: E731
    return {tuple(sorted((arm(c.geom[0]), arm(c.geom[1])))) for c in d.contact[: d.ncon]}


def test_query_checker_agrees_with_the_plain_online_checker(pair):
    """Tay 0, 1 di chuyển, tay 2, 3 cố định (một cấu hình): va chạm = mọi cặp tay trừ cặp (2, 3) giữa hai tay cố định."""
    rng = np.random.default_rng(0)
    for _ in range(300):
        q = np.tile(HOME, 4) + rng.normal(0, 0.8, 24)
        qc = QueryChecker(pair, [0, 1], {2: [q[12:18]], 3: [q[18:24]]})
        got = qc.config_free([q[0:6], q[6:12]])
        qc.restore()
        assert got == (not (_pairs_in_contact(pair, q) - {(2, 3)}))


def test_query_checker_treats_every_config_of_a_fixed_set_as_an_obstacle(pair):
    rng = np.random.default_rng(1)
    far = np.array([0.0, -3.0, 0.0, 0.0, 0.0, 0.0])
    for _ in range(100):
        q0 = HOME + rng.normal(0, 0.8, 6)
        S = [HOME + rng.normal(0, 0.8, 6) for _ in range(3)]
        qc = QueryChecker(pair, [0], {1: S})
        got = qc.arm_config_free(0, q0)
        qc.restore()
        want = all((0, 1) not in _pairs_in_contact(pair, np.concatenate([q0, s, far, far])) for s in S)
        assert got == want


def test_cpsat_is_never_worse_than_greedy_rules_in_its_own_model():
    rng = np.random.default_rng(2)
    K, I = 4, 6
    p = rng.uniform(5, 15, (K, I))
    conflict = rng.random((K, I, K, I)) < 0.2
    conflict = conflict | conflict.transpose(2, 3, 0, 1)
    for k in range(K):
        conflict[k, :, k, :] = False
    inst = Instance(p, conflict, rng.uniform(-0.3, 0.3, (I, 2)), ["red"] * I)
    opt = solve_cpsat(inst, time_limit=10)
    assert opt["status"] == "OPTIMAL"
    for rule in ("level0", "level1"):
        assert simulate_rule(inst, L, rule)["makespan"] >= opt["makespan"] - 0.1 - 1e-9


def test_patched_roadmap_equals_a_rebuilt_one():
    from drrt.roadmap import Roadmap
    from sortcell.planning.roadmaps import _patched
    rng = np.random.default_rng(0)
    q = rng.normal(size=(40, 6))
    edges = sorted({(int(a), int(b)) for a, b in rng.integers(0, 40, (120, 2)) if a < b})
    extra = [rng.normal(size=6) for _ in range(2)]
    new = [(3, 40), (7, 40), (40, 41), (5, 41)]
    base = Roadmap(q, edges)
    got, want = _patched(base, extra, new), Roadmap(np.vstack([q] + extra), edges + new)
    assert np.array_equal(got.q, want.q) and got.n_edges == want.n_edges
    assert all(np.array_equal(a, b) for a, b in zip(got.nbrs, want.nbrs))
    assert all(np.array_equal(a, b) for a, b in zip(got.adj, want.adj))
    assert got.dist == want.dist
    assert np.allclose(got.heuristic(41), want.heuristic(41))
    assert base.dist == Roadmap(q, edges).dist  # base không bị sửa
