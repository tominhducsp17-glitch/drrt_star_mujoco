"""Thử kẹp: UR10e + 2F-85 kẹp khối 5 cm, nhấc 10 cm, giữ 10 s; N lần với vị trí và góc khối ngẫu nhiên.

Điều khiển: actuator vị trí của UR10e bám theo quỹ đạo khớp nội suy min-jerk giữa các điểm IK; đoạn đi xuống và nhấc lên
nội suy thẳng trong không gian Descartes (IK từng điểm, khởi đầu ấm). Gripper: ctrl 0 = mở, 255 = đóng.

Đo mỗi lần thử (trong hệ TCP):
- lifted: khối cao hơn vị trí ban đầu ≥ 5 cm ở cuối 10 s giữ, và không lần nào trong lúc giữ thấp hơn mức đó;
- slip_hold: dịch chuyển của khối so với TCP từ đầu tới cuối 10 s giữ (mm), và góc xoay tương đối (độ);
- slip_lift: dịch chuyển tương đối từ lúc đóng xong tới đầu pha giữ (mm).
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass

import mujoco as mj
import numpy as np

from sortcell.skills.ik import HOME, ArmKin, down_rot
from sortcell.scene.builder import CUBE, GripperCfg, Layout, build_spec

LIFT_OK = 0.05


@dataclass
class Trial:
    seed: int
    x: float
    y: float
    yaw_deg: float
    lifted: bool
    min_rise_hold: float
    slip_hold_mm: float
    rot_hold_deg: float
    slip_lift_mm: float
    final_gap_mm: float
    shake_slip_mm: float | None = None
    shake_kept: bool | None = None


def build(cfg: GripperCfg, L: Layout, cube_pos, cube_yaw, cube_mass, cube_friction):
    spec = build_spec(L, "exec", cfg=cfg, arms=[0], cameras=False,
                      cubes=[("cube", cube_pos, cube_yaw, "red")])
    g = spec.geom("cube_geom")
    g.mass, g.friction = cube_mass, list(cube_friction)
    return spec.compile()


def grasp_yaw(cube_yaw: float, ref: float) -> float:
    """Góc má kẹp đồng trục với một cặp mặt khối, chọn trong 4 khả năng cái gần ref nhất."""
    cands = [cube_yaw + k * math.pi / 2 for k in range(-4, 5)]
    return min(cands, key=lambda a: abs((a - ref + math.pi) % (2 * math.pi) - math.pi))


def minjerk(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


class Runner:
    def __init__(self, m: mj.MjModel, video=None):
        self.m, self.d = m, mj.MjData(m)
        self.kin = ArmKin(m, 0)
        self.arm_act = np.array([m.actuator(f"a0_{n}").id for n in
                                 ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")])
        self.grip = m.actuator("a0_g_fingers_actuator").id
        self.cube = m.body("cube").id
        self.tcp = self.kin.tcp
        self.video = video
        self.t_frame = 0.0

    def collides(self, q) -> bool:
        """Tay chạm bất cứ thứ gì (trừ khối) ở cấu hình q."""
        d = mj.MjData(self.m)
        d.qpos[self.kin.idx] = q
        mj.mj_forward(self.m, d)
        for i in range(d.ncon):
            c = d.contact[i]
            names = [mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_BODY, self.m.geom_bodyid[g]) or "" for g in c.geom]
            if any(n.startswith("a0_") for n in names) and not any(n == "cube" for n in names) and c.dist < 0:
                return True
        return False

    def set_q(self, q):
        self.d.qpos[self.kin.idx] = q
        self.d.qvel[:] = 0
        self.d.ctrl[self.arm_act] = q
        mj.mj_forward(self.m, self.d)

    def step(self, n=1):
        for _ in range(n):
            mj.mj_step(self.m, self.d)
            if self.video is not None and self.d.time >= self.t_frame:
                self.video.capture(self.d)
                self.t_frame += 1 / 30

    def move_joint(self, q_to, T):
        q_from = self.d.ctrl[self.arm_act].copy()
        n = max(1, int(T / self.m.opt.timestep))
        for i in range(1, n + 1):
            self.d.ctrl[self.arm_act] = q_from + (q_to - q_from) * minjerk(i / n)
            self.step()

    def move_line(self, p_to, R, T, waypoints=20):
        """Đường thẳng Descartes cho TCP: IK ở các điểm trung gian, nối bằng min-jerk theo thời gian toàn đoạn."""
        q = self.d.ctrl[self.arm_act].copy()
        p_from, _ = self.kin.fk(q)
        qs = [q]
        for i in range(1, waypoints + 1):
            p = p_from + (p_to - p_from) * (i / waypoints)
            qi = self.kin.solve(p, R, qs[-1])
            if qi is None:
                raise RuntimeError("IK thất bại trên đoạn thẳng")
            qs.append(qi)
        qs = np.array(qs)
        n = max(1, int(T / self.m.opt.timestep))
        for i in range(1, n + 1):
            s = minjerk(i / n) * waypoints
            k = min(int(s), waypoints - 1)
            self.d.ctrl[self.arm_act] = qs[k] + (qs[k + 1] - qs[k]) * (s - k)
            self.step()

    def cube_in_tcp(self):
        R = self.d.site_xmat[self.tcp].reshape(3, 3)
        p = R.T @ (self.d.xpos[self.cube] - self.d.site_xpos[self.tcp])
        Rc = R.T @ self.d.xmat[self.cube].reshape(3, 3)
        return p, Rc

    def pad_gap(self):
        m, d = self.m, self.d
        pts = {}
        for side in ("left", "right"):
            pts[side] = np.mean([d.geom_xpos[m.geom(f"a0_g_{side}_pad{i}").id] for i in (1, 2)], axis=0)
        return float(np.linalg.norm(pts["left"] - pts["right"])) - 0.008  # trừ bề dày má kẹp (2 × 4 mm)


def rot_angle_deg(R):
    return math.degrees(math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def run_trial(seed: int, cfg: GripperCfg, L: Layout, mass: float, friction, hold: float, grasp_dz: float,
              shake: bool, video=None) -> Trial:
    rng = np.random.default_rng(seed)
    bx, by, _, _ = L.base(0)
    # Vùng thử: phần bàn phía tay 0, cách đế 0,40–0,80 m theo x, lệch ±0,25 m theo y.
    x = bx + rng.uniform(0.40, 0.80)
    y = rng.uniform(-0.25, 0.25)
    cyaw = rng.uniform(0, math.pi / 2)
    z0 = L.table_top + CUBE / 2
    m = build(cfg, L, (x, y, z0), cyaw, mass, friction)
    r = Runner(m, video)
    gyaw = grasp_yaw(cyaw, math.atan2(y - by, x - bx) + math.pi / 2)
    R = down_rot(gyaw)
    target = np.array([x, y, z0 + grasp_dz])
    pre = target + [0, 0, 0.15]
    q_pre = None
    for s0 in [HOME] + [rng.uniform(-np.pi, np.pi, 6) for _ in range(50)]:
        q = r.kin.solve(pre, R, s0)
        if q is not None and not r.collides(q):
            q_pre = q
            break
    if q_pre is None:
        raise RuntimeError("không có IK cho tư thế chuẩn bị gắp")
    r.set_q(q_pre)
    r.d.ctrl[r.grip] = 0
    r.step(int(0.5 / m.opt.timestep))  # khối nằm yên trên bàn
    cube_start = r.d.xpos[r.cube].copy()
    r.move_line(target, R, 1.5)
    r.step(int(0.3 / m.opt.timestep))
    r.d.ctrl[r.grip] = 255
    r.step(int(1.0 / m.opt.timestep))
    p_closed, _ = r.cube_in_tcp()
    r.move_line(target + [0, 0, 0.10], R, 1.5)
    r.step(int(0.2 / m.opt.timestep))
    p_h0, R_h0 = r.cube_in_tcp()
    min_rise = np.inf
    for _ in range(int(hold / m.opt.timestep)):
        r.step()
        min_rise = min(min_rise, r.d.xpos[r.cube][2] - cube_start[2])
    p_h1, R_h1 = r.cube_in_tcp()
    tr = Trial(seed, float(x), float(y), math.degrees(cyaw), bool(min_rise >= LIFT_OK), float(min_rise),
               float(np.linalg.norm(p_h1 - p_h0) * 1000), rot_angle_deg(R_h0.T @ R_h1),
               float(np.linalg.norm(p_h0 - p_closed) * 1000), r.pad_gap() * 1000)
    if shake and tr.lifted:
        # Kiểm tra thêm (ngoài nghiệm thu): quay khớp vai 90° trong 1 s rồi quay lại, như lúc mang vật tới khay.
        q = r.d.ctrl[r.arm_act].copy()
        q2 = q.copy()
        q2[0] += math.pi / 2
        r.move_joint(q2, 1.0)
        r.move_joint(q, 1.0)
        r.step(int(0.5 / m.opt.timestep))
        p_s, _ = r.cube_in_tcp()
        tr.shake_slip_mm = float(np.linalg.norm(p_s - p_h1) * 1000)
        tr.shake_kept = bool(r.d.xpos[r.cube][2] - cube_start[2] >= LIFT_OK)
    return tr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--variant", default="2f85")
    ap.add_argument("--mass", type=float, default=0.1)
    ap.add_argument("--hold", type=float, default=10.0)
    ap.add_argument("--grasp-dz", type=float, default=0.0, help="TCP cao hơn tâm khối bao nhiêu khi kẹp")
    ap.add_argument("--pad-friction", type=float, nargs=3, default=None)
    ap.add_argument("--pad-condim", type=int, default=None)
    ap.add_argument("--force", type=float, default=None)
    ap.add_argument("--pyramidal", action="store_true", help="bỏ cone=elliptic/impratio=10 (như khi attach quên)")
    ap.add_argument("--shake", action="store_true")
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--video", default=None, help="ghi video lần thử đầu tiên")
    a = ap.parse_args()
    if a.variant != "2f85" or a.pad_condim is not None:
        raise SystemExit("repo chỉ giữ bản 2f85; bản v4 và condim đã thử ở prototype (results/grasp/v4_*)")
    cfg = GripperCfg(pad_friction=tuple(a.pad_friction) if a.pad_friction else None, force_limit=a.force,
                     cone_elliptic=not a.pyramidal)
    L = Layout()
    trials = []
    for i in range(a.n):
        vid = None
        if a.video and i == 0:
            from sortcell.control.video import Recorder
            vid = Recorder(a.video, lookat=(-0.3, 0.0, 0.85), distance=1.3, elevation=-25.0)
        try:
            t = run_trial(a.seed0 + i, cfg, L, a.mass, (1.0, 0.005, 0.0001), a.hold, a.grasp_dz, a.shake, vid)
        finally:
            if vid is not None:
                vid.close()
        trials.append(t)
        print(f"seed {t.seed:3d} yaw {t.yaw_deg:5.1f}° lifted {t.lifted} min_rise {t.min_rise_hold*100:5.2f} cm "
              f"slip_hold {t.slip_hold_mm:6.3f} mm rot {t.rot_hold_deg:5.2f}° slip_lift {t.slip_lift_mm:5.2f} mm "
              f"gap {t.final_gap_mm:5.1f} mm" + (f" shake {t.shake_slip_mm:.2f} mm kept {t.shake_kept}"
                                                  if t.shake_slip_mm is not None else ""), flush=True)
    ok = sum(t.lifted for t in trials)
    hs = np.array([t.slip_hold_mm for t in trials if t.lifted]) if ok else np.array([np.nan])
    print(f"KẾT QUẢ {ok}/{len(trials)}; slip_hold max {hs.max():.3f} mm, trung bình {hs.mean():.3f} mm")
    if a.out:
        with open(a.out, "w") as f:
            json.dump({"args": vars(a), "success": ok, "n": len(trials), "trials": [asdict(t) for t in trials]}, f,
                      indent=1)


if __name__ == "__main__":
    main()
