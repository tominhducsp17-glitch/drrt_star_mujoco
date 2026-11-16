"""Đo hình học gripper sau khi gắn vào UR10e: tâm má kẹp so với mặt bích, và khoảng hở giữa hai má theo lệnh ctrl."""
import mujoco as mj
import numpy as np

from sortcell.scene.builder import Layout, build_spec

L = Layout()
m = build_spec(L, "exec", arms=[0], environment=False, cameras=False).compile()
d = mj.MjData(m)
site = m.site("a0_attachment_site").id
act = m.actuator("a0_g_fingers_actuator").id


def pads():
    R, p0 = d.site_xmat[site].reshape(3, 3), d.site_xpos[site]
    return {s: np.mean([R.T @ (d.geom_xpos[m.geom(f"a0_g_{s}_pad{i}").id] - p0) for i in (1, 2)], axis=0)
            for s in ("left", "right")}


print("ctrl  khe_trong_mm  z_tâm_má_mm(từ mặt bích)")
for ctrl in (0, 25, 50, 75, 100, 125, 150, 200, 255):
    mj.mj_resetData(m, d)
    d.qpos[[m.jnt_qposadr[m.joint(f"a0_{j}").id] for j in ("shoulder_lift_joint",)]] = -1.57
    d.ctrl[act] = ctrl
    for _ in range(2000):
        mj.mj_step(m, d)
    p = pads()
    gap = np.linalg.norm(p["left"] - p["right"]) - 0.008
    print(f"{ctrl:4d}  {gap*1000:8.1f}  {p['left'][2]*1000:8.1f}")
