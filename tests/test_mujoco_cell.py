import mujoco as mj
import numpy as np
import pytest

from drrt import ik
from drrt.cell_tasks import DOWN, CompositeSpace, crossing_two_arms, make_problem
from drrt.execute import Trajectory, execute, time_parameterize
from drrt.mujoco_cell import ArmSpace, CellPairChecker, FullChecker, two_arm_cell
from drrt.roadmap import Roadmap
from drrt.shortcut import path_length, shortcut


@pytest.fixture(scope="module")
def cell():
    return two_arm_cell()


@pytest.fixture(scope="module")
def problem():
    return make_problem(crossing_two_arms())


def test_mujoco_adds_the_margins_of_both_geoms():
    """Giả định nền của cách đặt lề an toàn trong mujoco_cell."""
    xml = ('<mujoco><worldbody><geom type="sphere" size="0.1" margin="{}"/><body pos="0.23 0 0"><freejoint/>'
           '<geom type="sphere" size="0.1" margin="{}"/></body></worldbody></mujoco>')
    for a, b, expected in ((0.025, 0.01, True), (0.02, 0.0, False)):
        m = mj.MjModel.from_xml_string(xml.format(a, b))
        d = mj.MjData(m)
        mj.mj_forward(m, d)
        assert (d.ncon > 0) == expected


def test_split_checking_never_misses_a_collision_of_the_full_model(cell):
    arms = [ArmSpace(cell, k) for k in range(2)]
    pair = CellPairChecker(cell)
    full = FullChecker(cell)
    rng = np.random.default_rng(3)
    disagree = 0
    for q in rng.uniform(-np.pi, np.pi, (3000, 12)):
        split_free = arms[0].config_free(q[:6]) and arms[1].config_free(q[6:]) and pair.config_free(q)
        full_free = not full.contacts(q)
        assert not (split_free and not full_free)
        disagree += split_free != full_free
    # Chỉ lệch theo hướng an toàn, do bộ va chạm của hình trụ trả khoảng cách hơi khác khi margin khác nhau.
    assert disagree <= 15


def test_ik_reaches_the_target_with_the_tool_pointing_down(cell):
    arm = ArmSpace(cell, 0)
    sols = ik.solutions(arm, (0.15, 0.3, 0.9), DOWN, np.random.default_rng(0), tries=60)
    assert sols
    for q in sols:
        pos, rot = arm.tip_pose(q)
        assert np.allclose(pos, (0.15, 0.3, 0.9), atol=1e-3)
        assert rot[:, 2] @ np.array(DOWN) > 0.999
        assert arm.config_free(q)


def test_problem_endpoints_are_free(problem):
    space = CompositeSpace(problem)
    s, g = np.concatenate(problem.start), np.concatenate(problem.goal)
    assert space.edge_free(s, s) and space.edge_free(g, g)


def test_roadmap_round_trips_through_disk(problem, tmp_path):
    rm = Roadmap.build(problem.arms[0], 60, np.random.default_rng(0), k=6, required=[problem.start[0]])
    rm.save(tmp_path / "rm.npz")
    back = Roadmap.load(tmp_path / "rm.npz")
    assert np.array_equal(back.q, rm.q)
    assert all(np.array_equal(a, b) for a, b in zip(back.nbrs, rm.nbrs))


def _free_detour(space, s, g):
    """Một điểm vòng trung gian sao cho cả hai đoạn đều tự do."""
    for delta in (0.3, -0.3, 0.6, -0.6):
        for joints in ([1, 7], [2, 8], [1, 2, 7, 8]):
            mid = (s + g) / 2
            mid[joints] += delta
            if space.edge_free(s, mid) and space.edge_free(mid, g):
                return mid
    raise AssertionError("không tìm được điểm vòng tự do")


def test_shortcut_keeps_endpoints_shortens_and_stays_free(problem):
    space = CompositeSpace(problem)
    s, g = np.concatenate(problem.start), np.concatenate(problem.start)
    g[[0, 6]] += 0.2
    path = [s, _free_detour(space, s, g), g]
    out = shortcut(path, space.edge_free, 2, np.random.default_rng(0), attempts=50)
    assert np.allclose(out[0], s) and np.allclose(out[-1], g)
    assert path_length(out, 2) <= path_length(path, 2) + 1e-9
    assert all(space.edge_free(a, b) for a, b in zip(out, out[1:]))


def test_quintic_trajectory_hits_waypoints_and_respects_vmax():
    traj = time_parameterize([np.zeros(12), np.full(12, 0.5), np.full(12, -0.2)], vmax=1.0, amax=2.0)
    assert np.allclose(traj.sample(0), 0) and np.allclose(traj.sample(traj.times[1]), 0.5)
    ts = np.linspace(0, traj.duration, 2000)
    assert max(np.max(np.abs(traj.velocity(t))) for t in ts) <= 1.0 + 1e-6


def test_execution_tracks_a_short_move_without_collision(problem):
    space = CompositeSpace(problem)
    s = np.concatenate(problem.start)
    g = _free_detour(space, s, s.copy())
    assert space.edge_free(s, g)
    result = execute(problem.task.cell, time_parameterize([s, g]), settle=0.3)
    assert result.collision_steps == 0
    assert result.max_tracking_error < 0.03
