import math
import os

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco as mj  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from sortcell.perception.overhead import Detection, OverheadCamera  # noqa: E402
from sortcell.perception.tracker import Tracker, yaw_diff  # noqa: E402
from sortcell.scene.builder import Layout, build_spec  # noqa: E402
from sortcell.scene.sampling import sample_scene  # noqa: E402
from sortcell.skills.ik import HOME, ArmKin  # noqa: E402


def test_yaw_diff_symmetry():
    assert abs(yaw_diff(math.radians(89), math.radians(1))) == pytest.approx(math.radians(2))
    assert abs(yaw_diff(0.3, 0.3 + math.pi / 2)) < 1e-12


@pytest.fixture(scope="module")
def scene():
    L = Layout()
    cubes = sample_scene(1003, L)
    m = build_spec(L, "exec", cubes=[c.as_tuple(L) for c in cubes]).compile()
    d = mj.MjData(m)
    for k in range(L.n_arms):
        d.qpos[ArmKin(m, k).idx] = HOME
    mj.mj_forward(m, d)
    cam = OverheadCamera(m, L)
    yield L, cubes, m, d, cam
    cam.close()


def test_detects_every_cube(scene):
    L, cubes, m, d, cam = scene
    rgb, dep = cam.capture(d)
    dets, _ = cam.detect(d, rgb, dep)
    assert len(dets) == len(cubes)
    for c in cubes:
        det = min(dets, key=lambda x: np.linalg.norm(x.xy - [c.x, c.y]))
        assert det.color == c.color
        assert det.full
        assert np.linalg.norm(det.xy - [c.x, c.y]) < 0.002
        assert abs(yaw_diff(det.yaw, c.yaw)) < math.radians(2.5)


def test_project_inverts_points(scene):
    L, cubes, m, d, cam = scene
    rgb, dep = cam.capture(d)
    P = cam.points(d, dep)
    u, v = cam.project(d, P[100, 200])
    assert (u, v) == (200, 100)


def _det(color, x, y, frac=1.0):
    return Detection(color, np.array([x, y]), 0.8, 0.1, frac, (0.05, 0.05))


def test_tracker_matching_and_partial():
    tr = Tracker(Layout())
    tr.update([_det("red", 0.0, 0.0), _det("blue", 0.2, 0.0)], 0.0)
    assert len(tr.tracks) == 2
    tr.update([_det("red", 0.01, 0.0), _det("blue", 0.25, 0.0, frac=0.5)], 0.2)
    assert len(tr.tracks) == 2                      # đỏ ghép được; xanh bị che một phần, lệch 5 cm: không ghép, không tạo mới
    assert tr.tracks[0].xy[0] == pytest.approx(0.01)
    tr.update([_det("blue", 0.21, 0.0, frac=0.5)], 0.4)
    assert tr.tracks[1].xy[0] == pytest.approx(0.2)  # phát hiện bị che không cập nhật pose
    assert tr.tracks[1].n_partial == 1
    tr.tracks[0].state = "busy"
    tr.update([_det("red", 0.012, 0.0)], 0.6)
    assert len(tr.tracks) == 2                      # vật đang bận: không sinh bản ghi trùng
