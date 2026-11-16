"""Mô phỏng cell bằng mj_step: bám quỹ đạo khớp, chạy chuyển động cục bộ, đo va chạm thật ở từng bước.

Phân loại tiếp xúc lún (dist < 0) theo mục 2 của kế hoạch:
- arm_arm: geom của hai tay khác nhau;
- arm_env: tay với bàn, bệ, khay, sàn;
- arm_cube: tay với một khối mà tay đó không được phép chạm (không phải khối nó đang gắp, mang hoặc thả). Gồm cả khối do
  tay khác đang mang chạm vào tay này. Mục 2 không nói rõ loại này; ở đây tính là va chạm cho chặt;
- self: hai geom của cùng một tay (không tính gripper với chính khối nó đang kẹp, và hai geom bên trong gripper: má kẹp
  chạm nhau khi đóng hụt);
- held_env: khối đang được mang (đã nhấc lên, chưa hạ vào khay) chạm bàn, khay hay khối khác.
Tiếp xúc khối–bàn, khối–khay, khối–khối không tính.
"""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field

import mujoco as mj
import numpy as np

from drrt.execute import Trajectory

from ..scene.builder import ARM_ACTUATORS, CUBE, GRIP_ACT, TCP, Layout, arm_of_geom, build_spec
from ..scene.sampling import CubeSpec
from ..skills.ik import HOME, ArmKin

GRIP_OPEN = 50      # khe 70 mm (experiments/measure_gripper.py)
GRIP_CLOSE = 255
CATEGORIES = ("arm_arm", "arm_env", "arm_cube", "self", "held_env")
CUBE_MARGIN = 0.005


def minjerk(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10 - 15 * s + 6 * s * s)


@dataclass
class LineSeg:
    """Chuyển động dọc dãy cấu hình qs (đoạn thẳng Descartes đã IK), min-jerk theo thời gian toàn đoạn."""
    qs: np.ndarray
    T: float

    def sample(self, t: float) -> np.ndarray:
        n = len(self.qs) - 1
        s = minjerk(t / self.T) * n
        i = min(int(s), n - 1)
        return self.qs[i] + (self.qs[i + 1] - self.qs[i]) * (s - i)


@dataclass
class Step:
    """Một bước của kịch bản cục bộ: đi theo seg (nếu có) với lệnh gripper grip, rồi chờ wait giây."""
    seg: LineSeg | None
    grip: float
    wait: float = 0.0

    @property
    def duration(self) -> float:
        return (self.seg.T if self.seg else 0.0) + self.wait


@dataclass
class CollisionLog:
    steps: Counter = field(default_factory=Counter)           # số bước mj_step có ít nhất một tiếp xúc mỗi loại
    pairs: Counter = field(default_factory=Counter)           # (loại, geom1, geom2) → số bước
    first: dict = field(default_factory=dict)                 # loại → (thời điểm, geom1, geom2, dist)
    max_depth: dict = field(default_factory=dict)

    @property
    def total_steps(self) -> int:
        return sum(self.steps.values())


class CellSim:
    def __init__(self, L: Layout, cubes: list[CubeSpec], recorder=None):
        self.L = L
        spec = build_spec(L, "exec", cubes=[c.as_tuple(L) for c in cubes])
        self.m = m = spec.compile()
        self.d = mj.MjData(m)
        # model riêng để kiểm tra trước đoạn đi xuống: geom của khối có lề CUBE_MARGIN
        self.chk_m = spec.compile()
        self.chk_d = mj.MjData(self.chk_m)
        for c in cubes:
            self.chk_m.geom_margin[self.chk_m.geom(c.name + "_geom").id] = CUBE_MARGIN
        self.n = L.n_arms
        self.kins = [ArmKin(m, k) for k in range(self.n)]
        self.act = [np.array([m.actuator(f"a{k}_{a}").id for a in ARM_ACTUATORS]) for k in range(self.n)]
        self.grip = [m.actuator(f"a{k}_{GRIP_ACT}").id for k in range(self.n)]
        self.tcp = [m.site(f"a{k}_{TCP}").id for k in range(self.n)]
        a = self.act[0]
        self.lead = m.actuator_biasprm[a, 2] / m.actuator_biasprm[a, 1]  # kd/kp: feedforward vận tốc như drrt.execute
        self.cubes = cubes
        self.cube_body = [m.body(c.name).id for c in cubes]
        self.geom_arm = np.array([arm_of_geom(m, g) for g in range(m.ngeom)])
        # geom của gripper: hai má kẹp chạm nhau khi đóng hụt (không có vật) là bình thường, không tính "self"
        self.geom_grip = np.array(["_g_" in (mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "")
                                   for g in range(m.ngeom)])
        self.geom_cube = -np.ones(m.ngeom, int)
        for i, c in enumerate(cubes):
            self.geom_cube[m.geom(c.name + "_geom").id] = i
        self.allowed: dict[int, int] = {}   # khối → tay được phép chạm
        self.carrying: set[int] = set()     # khối đang được mang (đã nhấc, chưa bắt đầu hạ xuống khay)
        self.track_worst = (0.0, -1, -1.0)  # (sai số, chỉ số khớp trong 24, thời điểm)
        self.coll = CollisionLog()
        self.recorder = recorder
        self._next_frame = 0.0
        for k in range(self.n):
            self.set_arm(k, HOME)
            self.d.ctrl[self.grip[k]] = GRIP_OPEN
        mj.mj_forward(m, self.d)

    # ------------------------------------------------------------------ trạng thái
    @property
    def time(self) -> float:
        return float(self.d.time)

    def set_arm(self, k: int, q) -> None:
        self.d.qpos[self.kins[k].idx] = q
        self.d.ctrl[self.act[k]] = q

    def q(self, k: int) -> np.ndarray:
        return self.d.qpos[self.kins[k].idx].copy()

    def cube_pose(self, i: int) -> tuple[np.ndarray, float]:
        """Vị trí và góc quanh trục đứng, quy về [0, π/2)."""
        b = self.cube_body[i]
        R = self.d.xmat[b].reshape(3, 3)
        return self.d.xpos[b].copy(), math.atan2(R[1, 0], R[0, 0]) % (math.pi / 2)

    def spot_free(self, x: float, y: float, yaw: float, i: int, clearance: float = 0.03) -> bool:
        """Đặt thử khối i ở (x, y) trên bàn (trong model kiểm tra, trạng thái hiện tại của sim): không tay nào tới gần
        hơn `clearance`, không chạm khối khác. Dùng để chọn chỗ thả khối mới (M4 việc 2)."""
        d = self.chk_d
        d.qpos[:] = self.d.qpos
        adr = self.chk_m.jnt_qposadr[self.chk_m.body_jntadr[self.cube_body[i]]]
        d.qpos[adr:adr + 7] = [x, y, self.L.table_top + CUBE / 2, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        g = self.chk_m.geom(self.cubes[i].name + "_geom").id
        old = self.chk_m.geom_margin[g]
        self.chk_m.geom_margin[g] = clearance
        try:
            mj.mj_kinematics(self.chk_m, d)
            mj.mj_collision(self.chk_m, d)
            for (g1, g2), dd in zip(d.contact.geom[: d.ncon], d.contact.dist[: d.ncon]):
                if g not in (g1, g2):
                    continue
                other = g2 if g1 == g else g1
                if self.geom_arm[other] >= 0 and dd < clearance:
                    return False
                if self.geom_cube[other] >= 0 and dd < 0.005:
                    return False
            return True
        finally:
            self.chk_m.geom_margin[g] = old

    def spawn_cube(self, i: int, x: float, y: float, yaw: float) -> None:
        """Đặt khối i lên bàn tại (x, y), vận tốc 0 (khối được thả thêm vào cell)."""
        jnt = self.m.body_jntadr[self.cube_body[i]]
        adr, vadr = self.m.jnt_qposadr[jnt], self.m.jnt_dofadr[jnt]
        self.d.qpos[adr:adr + 7] = [x, y, self.L.table_top + CUBE / 2 + 0.001, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        self.d.qvel[vadr:vadr + 6] = 0
        mj.mj_forward(self.m, self.d)

    def lifted_cube(self, k: int, lift: float) -> int | None:
        """Đánh giá: khối thật đang nằm trong gripper của tay k và cao hơn mặt bàn ít nhất lift (mục 2: nhấc lên 5 cm)."""
        tcp = self.d.site_xpos[self.tcp[k]]
        z0 = self.L.table_top + CUBE / 2
        for i in range(len(self.cubes)):
            p, _ = self.cube_pose(i)
            if p[2] - z0 >= lift and np.linalg.norm(p - tcp) < 0.04:
                return i
        return None

    def descent_touches(self, k: int, qs, target: int | None, cube_qpos: dict | None = None) -> bool:
        """Đặt tay k lần lượt ở các cấu hình qs, mọi thứ khác giữ trạng thái hiện tại của sim: tay k có tới gần
        (< CUBE_MARGIN) khối nào khác khối target không. cube_qpos: {địa chỉ qpos: 7 số} đặt khối theo ước lượng thay
        cho trạng thái thật (M3: bộ điều phối chỉ biết pose ước lượng)."""
        d = self.chk_d
        d.qpos[:] = self.d.qpos
        for adr, v in (cube_qpos or {}).items():
            d.qpos[adr:adr + 7] = v
        idx = self.kins[k].idx
        for q in qs:
            d.qpos[idx] = q
            mj.mj_kinematics(self.chk_m, d)
            mj.mj_collision(self.chk_m, d)
            for (g1, g2), dd in zip(d.contact.geom[: d.ncon], d.contact.dist[: d.ncon]):
                a1, a2 = self.geom_arm[g1], self.geom_arm[g2]
                c1, c2 = self.geom_cube[g1], self.geom_cube[g2]
                if dd < CUBE_MARGIN and ((a1 == k and c2 >= 0 and c2 != target) or
                                         (a2 == k and c1 >= 0 and c1 != target)):
                    return True
        return False

    # ------------------------------------------------------------------ bước mô phỏng
    def step(self) -> None:
        mj.mj_step(self.m, self.d)
        self._check_contacts()
        if self.recorder is not None and self.d.time >= self._next_frame:
            self.recorder.capture(self.d)
            self._next_frame += 1 / self.recorder.fps

    def _check_contacts(self) -> None:
        d = self.d
        if d.ncon == 0:
            return
        geom = d.contact.geom[: d.ncon]
        dist = d.contact.dist[: d.ncon]
        seen = set()
        for (g1, g2), dd in zip(geom, dist):
            if dd >= 0:
                continue
            a1, a2 = self.geom_arm[g1], self.geom_arm[g2]
            c1, c2 = self.geom_cube[g1], self.geom_cube[g2]
            if a1 < 0 and a2 < 0:
                # khối–bàn, khối–khay, khối–khối: chỉ tính khi khối đang được mang đi (giữa nhấc lên và hạ xuống khay)
                if (c1 >= 0 and int(c1) in self.carrying) or (c2 >= 0 and int(c2) in self.carrying):
                    cat = "held_env"
                    key = (cat, self._gname(g1), self._gname(g2))
                    self.coll.pairs[key] += 1
                    self.coll.first.setdefault(cat, (round(self.time, 3), key[1], key[2], float(dd)))
                    seen.add(cat)
                continue
            if a1 >= 0 and a2 >= 0:
                if a1 == a2 and self.geom_grip[g1] and self.geom_grip[g2]:
                    continue
                cat = "self" if a1 == a2 else "arm_arm"
            else:
                arm, cube = (a1, c2) if a1 >= 0 else (a2, c1)
                if cube >= 0:
                    if self.allowed.get(int(cube)) == arm:
                        continue
                    cat = "arm_cube"
                else:
                    cat = "arm_env"
            key = (cat, self._gname(g1), self._gname(g2))
            self.coll.pairs[key] += 1
            self.coll.first.setdefault(cat, (round(self.time, 3), key[1], key[2], float(dd)))
            self.coll.max_depth[cat] = min(self.coll.max_depth.get(cat, 0.0), float(dd))
            seen.add(cat)
        for cat in seen:
            self.coll.steps[cat] += 1

    def _gname(self, g: int) -> str:
        name = mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_GEOM, g)
        if name:
            return name
        return f"{mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_BODY, self.m.geom_bodyid[g])}#{g}"

    def wait(self, T: float) -> None:
        for _ in range(int(round(T / self.m.opt.timestep))):
            self.step()

    # ------------------------------------------------------------------ chuyển động
    def track(self, traj: Trajectory, settle: float = 0.1) -> float:
        """Mọi tay bám quỹ đạo hợp (24 chiều). Trả sai số bám lớn nhất (rad)."""
        dt = self.m.opt.timestep
        t0 = self.time
        err = 0.0
        while self.time - t0 < traj.duration + settle:
            t = self.time - t0
            q, v = traj.sample(t), traj.velocity(t)
            for k in range(self.n):
                s = slice(6 * k, 6 * k + 6)
                self.d.ctrl[self.act[k]] = q[s] + self.lead * v[s]
            self.step()
            qa = np.concatenate([self.q(k) for k in range(self.n)])
            e = np.abs(qa - traj.sample(self.time - t0))
            if e.max() > err:
                err = float(e.max())
                if err > self.track_worst[0]:
                    self.track_worst = (err, int(e.argmax()), round(self.time, 3))
        del dt
        return err

    def run_scripts(self, scripts: dict[int, list[Step]]) -> None:
        """Chạy song song kịch bản cục bộ của các tay; tay không có kịch bản giữ nguyên lệnh."""
        plans = {}
        for k, steps in scripts.items():
            t, seq = 0.0, []
            for st in steps:
                seq.append((t, st))
                t += st.duration
            plans[k] = (seq, t)
        T = max((t for _, t in plans.values()), default=0.0)
        t0 = self.time
        while self.time - t0 < T:
            t = self.time - t0
            for k, (seq, _) in plans.items():
                cur = None
                for start, st in seq:
                    if t >= start:
                        cur = (start, st)
                if cur is None:
                    continue
                start, st = cur
                self.d.ctrl[self.grip[k]] = st.grip
                if st.seg is not None:
                    self.d.ctrl[self.act[k]] = st.seg.sample(min(t - start, st.seg.T))
            self.step()
