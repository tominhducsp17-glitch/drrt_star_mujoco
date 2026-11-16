"""Render thử: camera trên cao RGB-D 640×480 và camera cổ tay 84×84 (RGB + depth) ở tư thế chuẩn bị gắp.

Ra results/cameras/. Depth lưu .npy (mét) và ảnh màu để xem; depth cổ tay cắt ở 30 cm như mục 5.4.
"""
from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import imageio.v2 as imageio  # noqa: E402
import mujoco as mj  # noqa: E402
import numpy as np  # noqa: E402

from sortcell.scene.builder import Layout, build_spec  # noqa: E402
from sortcell.scene.sampling import sample_scene  # noqa: E402
from sortcell.skills.ik import HOME, ArmKin  # noqa: E402
from sortcell.skills.scripted import ArmTools  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "results/cameras"


def depth_vis(depth, lo, hi):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import cm
    x = np.clip((depth - lo) / (hi - lo), 0, 1)
    return (cm.viridis(1 - x)[..., :3] * 255).astype(np.uint8)


def render(m, d, cam, w, h, depth=False):
    r = mj.Renderer(m, h, w)
    r.update_scene(d, cam)
    rgb = r.render()
    dep = None
    if depth:
        r.enable_depth_rendering()
        r.update_scene(d, cam)
        dep = r.render()
        r.disable_depth_rendering()
    r.close()
    return rgb, dep


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    L = Layout()
    cubes = sample_scene(1000, L)
    m = build_spec(L, "exec", cubes=[c.as_tuple(L) for c in cubes]).compile()
    d = mj.MjData(m)
    kins = [ArmKin(m, k) for k in range(4)]
    for k in kins:
        d.qpos[k.idx] = HOME
    mj.mj_forward(m, d)
    # Ảnh tổng quan
    cam = mj.MjvCamera()
    cam.lookat[:] = [0, 0, 0.7]
    cam.distance, cam.azimuth, cam.elevation = 3.6, 135, -35
    rgb, _ = render(m, d, cam, 1280, 960)
    imageio.imwrite(OUT / "overview.png", rgb)
    # Camera trên cao
    rgb, dep = render(m, d, "overhead", 640, 480, depth=True)
    imageio.imwrite(OUT / "overhead_rgb.png", rgb)
    np.save(OUT / "overhead_depth.npy", dep.astype(np.float32))
    imageio.imwrite(OUT / "overhead_depth.png", depth_vis(dep, dep.min(), L.table_top + 1.0))
    cid = m.camera("overhead").id
    print("overhead: z =", round(float(m.cam_pos[cid][2]), 3), "m, fovy 60°, depth min/max",
          round(float(dep.min()), 3), round(float(dep.max()), 3))
    # Kiểm tra bàn nằm trọn trong ảnh: chiếu 4 góc bàn
    f = 480 / 2 / np.tan(np.radians(60) / 2)
    zc = m.cam_pos[cid][2] - L.table_top
    corner_px = [(320 + f * x / zc, 240 - f * y / zc) for x in (-L.table_half, L.table_half)
                 for y in (-L.table_half, L.table_half)]
    print("góc bàn (pixel):", [(round(u), round(v)) for u, v in corner_px])
    # Camera cổ tay: tay 0 ở tư thế chuẩn bị gắp khối gần tay 0 nhất
    bx, by, _, _ = L.base(0)
    target = min(cubes, key=lambda c: (c.x - bx) ** 2 + (c.y - by) ** 2)
    tools = ArmTools(L, 0)
    act = m.actuator("a0_g_fingers_actuator").id
    d.ctrl[act] = 50
    for d_h in (0.16, 0.08):
        mv = tools.pick((target.x, target.y), target.yaw, d_h)
        d.qpos[kins[0].idx] = mv.q_top
        mj.mj_forward(m, d)
        rgb, dep = render(m, d, "a0_wrist_cam", 84, 84, depth=True)
        tag = f"dh{int(d_h * 100)}"
        imageio.imwrite(OUT / f"wrist_{tag}_rgb.png", rgb)
        np.save(OUT / f"wrist_{tag}_depth.npy", dep.astype(np.float32))
        imageio.imwrite(OUT / f"wrist_{tag}_depth.png", depth_vis(np.minimum(dep, 0.30), 0.0, 0.30))
        # bản phóng to để xem
        big = np.kron(rgb, np.ones((4, 4, 1), np.uint8))
        imageio.imwrite(OUT / f"wrist_{tag}_rgb_x4.png", big)
        cube_px = int(np.sum(np.all(np.abs(rgb.astype(int) - [217, 38, 38]) < 60, axis=-1)))
        print(f"wrist d_h={d_h}: khối {target.name} ({target.color}), depth min {dep.min():.3f} m, "
              f"số pixel trong khoảng 30 cm: {int((dep < 0.30).sum())}/{84*84}")
    rgb, _ = render(m, d, cam, 1280, 960)
    imageio.imwrite(OUT / "overview_pregrasp.png", rgb)


if __name__ == "__main__":
    main()
