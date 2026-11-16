"""Ảnh minh hoạ perception: camera trên cao, tay 0 và tay 2 ở tư thế chuẩn bị gắp (che một phần bàn), khung minAreaRect
của từng phát hiện (liền = thấy rõ, đứt = bị che một phần) và tâm thật của khối (dấu +). Ra results/m3/."""
from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import cv2  # noqa: E402
import imageio.v2 as imageio  # noqa: E402
import mujoco as mj  # noqa: E402
import numpy as np  # noqa: E402

from sortcell.perception.overhead import OverheadCamera  # noqa: E402
from sortcell.scene.builder import CUBE, Layout, build_spec  # noqa: E402
from sortcell.scene.sampling import sample_scene  # noqa: E402
from sortcell.skills.ik import HOME, ArmKin  # noqa: E402
from sortcell.skills.scripted import ArmTools  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "results/m3"


def main(seed: int = 1000):
    OUT.mkdir(parents=True, exist_ok=True)
    L = Layout()
    cubes = sample_scene(seed, L)
    m = build_spec(L, "exec", cubes=[c.as_tuple(L) for c in cubes]).compile()
    d = mj.MjData(m)
    for k in range(L.n_arms):
        d.qpos[ArmKin(m, k).idx] = HOME
    for k in (0, 2):  # tay 0 và 2 cúi xuống khối gần nhất
        bx, by, _, _ = L.base(k)
        c = min(cubes, key=lambda c: (c.x - bx) ** 2 + (c.y - by) ** 2)
        mv = ArmTools(L, k).pick((c.x, c.y), c.yaw, 0.08)
        d.qpos[ArmKin(m, k).idx] = mv.q_top
    mj.mj_forward(m, d)
    cam = OverheadCamera(m, L)
    rgb, dep = cam.capture(d)
    dets, _ = cam.detect(d, rgb, dep)
    img = cv2.resize(rgb, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST).copy()
    z = L.table_top + CUBE
    for det in dets:
        corners = []
        R = np.array([[np.cos(det.yaw), -np.sin(det.yaw)], [np.sin(det.yaw), np.cos(det.yaw)]])
        for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
            p = det.xy + R @ (np.array([sx, sy]) * CUBE / 2)
            u, v = cam.project(d, [p[0], p[1], z])
            corners.append((2 * u + 1, 2 * v + 1))
        pts = np.array(corners, np.int32)
        col = (255, 255, 0) if det.full else (255, 128, 0)
        if det.full:
            cv2.polylines(img, [pts], True, col, 2)
        else:
            for a, b in zip(pts, np.roll(pts, -1, 0)):
                for t in np.linspace(0, 1, 8)[::2]:
                    p0 = a + (b - a) * t
                    p1 = a + (b - a) * min(t + 1 / 7, 1)
                    cv2.line(img, tuple(p0.astype(int)), tuple(p1.astype(int)), col, 2)
        u, v = cam.project(d, [*det.xy, z])
        cv2.putText(img, f"{det.color[0].upper()} {det.area_frac:.2f}", (2 * u + 14, 2 * v - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    for c in cubes:
        u, v = cam.project(d, [c.x, c.y, z])
        cv2.drawMarker(img, (2 * u + 1, 2 * v + 1), (0, 0, 0), cv2.MARKER_CROSS, 12, 2)
    imageio.imwrite(OUT / "perception_overlay.png", img)
    print(f"{len(dets)} phát hiện, {sum(d_.full for d_ in dets)} thấy rõ; ảnh: {OUT / 'perception_overlay.png'}")
    for det in dets:
        c = min(cubes, key=lambda c: np.hypot(c.x - det.xy[0], c.y - det.xy[1]))
        print(det.color, c.color, f"lệch {np.hypot(c.x - det.xy[0], c.y - det.xy[1]) * 1000:.2f} mm, diện tích {det.area_frac:.2f}")
    cam.close()


if __name__ == "__main__":
    main()
