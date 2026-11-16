import math

import mujoco as mj
import numpy as np
import pytest

from sortcell.planning.models import ArmSpace, FullChecker, PairChecker
from sortcell.scene.builder import CUBE, Layout, build_spec
from sortcell.scene.sampling import MIN_DIST, eval_scenes, sample_scene
from sortcell.skills.ik import HOME, ArmKin, down_rot
from sortcell.skills.scripted import ArmTools

L = Layout()


def test_gripper_contact_options_survive_attach():
    """Menagerie đặt cone=elliptic, impratio=10 ở model gripper; attach bỏ chúng nên builder phải đặt lại."""
    m = build_spec(L, "exec", arms=[0], cameras=False).compile()
    assert m.opt.cone == mj.mjtCone.mjCONE_ELLIPTIC and m.opt.impratio == 10


def test_scene_sampling_respects_spacing_and_zone():
    for cubes in eval_scenes(10):
        xy = np.array([(c.x, c.y) for c in cubes])
        assert len(cubes) == 8
        assert np.all(np.abs(xy) <= L.zone_half)
        d = np.linalg.norm(xy[:, None] - xy[None], axis=-1) + np.eye(8)
        assert d.min() >= MIN_DIST
    assert sample_scene(1000) == sample_scene(1000)


def test_ik_reaches_pose_with_gripper_down():
    kin = ArmKin(ArmSpace(L, 0).model, 0)
    target, R = np.array([-0.2, 0.1, 0.95]), down_rot(0.3)
    q = kin.solve(target, R, HOME)
    p, Rc = kin.fk(q)
    assert np.allclose(p, target, atol=1e-3) and np.allclose(Rc, R, atol=2e-3)


def test_split_checking_never_misses_a_collision_of_the_full_model():
    arms = [ArmSpace(L, k) for k in range(4)]
    pair, full = PairChecker(L), FullChecker(L)
    rng = np.random.default_rng(3)
    Q = np.vstack([rng.uniform(-np.pi, np.pi, (400, 24)), np.tile(HOME, 4) + rng.normal(0, 0.6, (400, 24))])
    for q in Q:
        split_free = all(arms[k].config_free(q[6 * k:6 * k + 6]) for k in range(4)) and pair.config_free(q)
        assert not (split_free and full.contacts(q))


def test_phantom_cube_blocks_carrying_through_the_table():
    """Khối ảo làm cho tư thế mà khối đang mang xuyên bàn bị coi là va chạm, dù gripper không chạm bàn."""
    t = ArmTools(L, 0)
    zc = L.table_top + CUBE / 2
    R = down_rot(0.0)
    q = t.kin.solve(np.array([-0.3, 0.0, zc - 0.01]), R, HOME)
    assert q is not None and not t.space.config_free(q)


@pytest.mark.parametrize("arm", range(4))
def test_owner_reaches_every_tray_slot(arm):
    t = ArmTools(L, arm)
    for j in range(3):
        for s in L.tray_slots(arm, j):
            assert t.place(s) is not None


def test_ik_does_not_converge_to_the_pose_flipped_about_the_tool_axis():
    """Hồi quy: sai số hướng cũ (½Σ Rc_i × R_i) bằng 0 ở hướng quay π quanh z."""
    kin = ArmKin(ArmSpace(L, 0).model, 0)
    target, R = np.array([-0.3, 0.05, 0.95]), down_rot(0.4)
    q = kin.solve(target, R, HOME)
    flipped = R @ np.diag([-1.0, -1.0, 1.0])
    q2 = kin.solve(target, flipped, q)  # bắt đầu từ nghiệm của R, đích là R quay π: phải thực sự quay cổ tay
    assert np.allclose(kin.fk(q2)[1], flipped, atol=2e-3)


def test_descent_line_is_continuous():
    t = ArmTools(L, 1)
    for x, y, yaw in ((0.1, -0.2, 0.3), (-0.2, 0.25, 1.2), (0.3, 0.0, 0.7)):
        mv = t.pick((x, y), yaw)
        assert mv is not None and np.max(np.abs(np.diff(mv.down, axis=0))) <= 0.3
