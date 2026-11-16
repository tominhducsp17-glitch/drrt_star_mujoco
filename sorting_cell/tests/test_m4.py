import math
import os

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco as mj  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from sortcell.perception.overhead import OverheadCamera  # noqa: E402
from sortcell.perception.tracker import yaw_diff  # noqa: E402
from sortcell.scene.builder import CUBE, Layout, build_spec  # noqa: E402
from sortcell.scene.sampling import (CubeSpec, sample_arrival_scene, sample_cluster_scene,  # noqa: E402
                                     squares_gap)
from sortcell.skills.ik import HOME, ArmKin  # noqa: E402


def test_squares_gap():
    assert squares_gap((0, 0, 0), (CUBE + 0.01, 0, 0)) == pytest.approx(0.01)
    assert squares_gap((0, 0, 0), (CUBE / 2, 0, 0)) < 0
    assert squares_gap((0, 0, 0), (0.2, 0.2, 0.3)) > 0.1


def test_cluster_scenes_do_not_overlap_and_touch():
    touching = 0
    for seed in range(2000, 2010):
        cs = sample_cluster_scene(seed)
        assert len(cs) == 8
        for i, a in enumerate(cs):
            for b in cs[i + 1:]:
                g = squares_gap((a.x, a.y, a.yaw), (b.x, b.y, b.yaw))
                assert g > 0
                touching += g < 0.002
    assert touching > 0


def _detect(cubes):
    L = Layout()
    m = build_spec(L, "exec", cubes=[c.as_tuple(L) for c in cubes]).compile()
    d = mj.MjData(m)
    for k in range(L.n_arms):
        d.qpos[ArmKin(m, k).idx] = HOME
    mj.mj_forward(m, d)
    cam = OverheadCamera(m, L)
    rgb, dep = cam.capture(d)
    dets, _ = cam.detect(d, rgb, dep)
    cam.close()
    return dets


@pytest.mark.parametrize("yaw2", [0.2, 0.9])
def test_split_touching_same_color(yaw2):
    a = (0.05, 0.02, 0.2)
    th = 0.4
    # đẩy khối thứ hai ra theo hướng th tới khi khe 0,5 mm
    lo, hi = 0.0, 2 * CUBE
    for _ in range(40):
        mid = (lo + hi) / 2
        if squares_gap(a, (a[0] + mid * math.cos(th), a[1] + mid * math.sin(th), yaw2)) < 0.0005:
            lo = mid
        else:
            hi = mid
    b = (a[0] + hi * math.cos(th), a[1] + hi * math.sin(th), yaw2)
    cubes = [CubeSpec("cube0", "green", *a), CubeSpec("cube1", "green", *b)]
    dets = _detect(cubes)
    assert len(dets) == 2 and all(d_.full for d_ in dets)
    if yaw2 == 0.2:  # gần thẳng hàng: hai mặt trên dính thành một vùng, phải qua bước tách
        assert all(d_.split == 2 for d_ in dets)
    for c in cubes:
        det = min(dets, key=lambda d_: np.hypot(*(d_.xy - [c.x, c.y])))
        assert np.hypot(*(det.xy - [c.x, c.y])) < 0.003
        assert abs(yaw_diff(det.yaw, c.yaw)) < math.radians(4)


def test_arrival_scene_parks_new_cubes_off_table():
    cubes, arrivals = sample_arrival_scene(3000)
    assert len(cubes) == 12 and len(arrivals) == 4
    assert all(cubes[a.cube].x > 2 for a in arrivals)
    assert all(5 <= a.t <= 40 for a in arrivals)
