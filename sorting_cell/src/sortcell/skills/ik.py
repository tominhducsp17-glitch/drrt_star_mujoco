"""IK 6 bậc cho TCP (damped least squares), cùng cách với drrt.ik.solve nhưng ràng buộc cả hướng kẹp."""
from __future__ import annotations

import math

import mujoco as mj
import numpy as np

from drrt.mujoco_cell import JOINTS, _qpos_indices

from ..scene.builder import TCP, Layout

# Khuỷu hướng lên, cổ tay chúc xuống, hướng vào tâm bàn. Đế UR10e của Menagerie quay 180° quanh z nên pan = π.
HOME = np.array([math.pi, -1.9, 1.9, -1.57, -1.57, 0.0])


def down_rot(yaw: float) -> np.ndarray:
    """TCP chúc xuống (z = −Z), phương đóng mở má kẹp (x) quay góc yaw trong mặt phẳng ngang."""
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([x, np.cross(z, x), z])


def wrap_near(q, ref):
    """Đưa từng khớp về cùng vòng với ref (không đổi tư thế, chỉ bỏ bội 2π)."""
    return ref + (np.asarray(q) - ref + math.pi) % (2 * math.pi) - math.pi


def seeds(L: Layout, k: int, target) -> list[np.ndarray]:
    """Điểm khởi đầu 'đẹp' cho IK: khuỷu lên, cổ tay chúc xuống, pan hướng về đích."""
    x, y, _, yaw = L.base(k)
    pan = math.atan2(target[1] - y, target[0] - x) - yaw + math.pi
    out = []
    for lift, elbow in ((-1.9, 1.9), (-1.3, 1.6), (-2.3, 2.3), (-1.0, 1.0)):
        out.append(np.array([pan, lift, elbow, -math.pi / 2 - lift - elbow, -math.pi / 2, 0.0]))
    return [(s + math.pi) % (2 * math.pi) - math.pi for s in out]


class ArmKin:
    """FK/IK cho tay `arm` trên một model bất kỳ (model có tay đó). Dùng MjData riêng."""

    def __init__(self, model: mj.MjModel, arm: int, joint_limit: float = math.pi):
        self.m, self.d = model, mj.MjData(model)
        self.arm = arm
        self.idx = _qpos_indices(model, arm)
        self.cols = np.array([model.jnt_dofadr[model.joint(f"a{arm}_{j}").id] for j in JOINTS])
        self.tcp = model.site(f"a{arm}_{TCP}").id
        self.lower, self.upper = np.full(6, -joint_limit), np.full(6, joint_limit)

    def fk(self, q):
        self.d.qpos[self.idx] = q
        mj.mj_kinematics(self.m, self.d)
        return self.d.site_xpos[self.tcp].copy(), self.d.site_xmat[self.tcp].reshape(3, 3).copy()

    def solve(self, pos, R, q0, iters: int = 200, damping: float = 1e-3, tol: float = 1e-4):
        """Trả nghiệm cùng vòng với q0 (để nội suy từ q0 không quay thêm 2π), hoặc None."""
        m, d = self.m, self.d
        q0 = np.asarray(q0, float)
        q = q0.copy()
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        quat, er = np.zeros(4), np.zeros(3)
        for _ in range(iters):
            p, Rc = self.fk(q)
            ep = pos - p
            # Vector quay của R·Rcᵀ (hệ thế giới). Không dùng ½Σ Rc_i × R_i: biểu thức đó cũng bằng 0 khi Rc là R quay
            # π quanh z, làm IK hội tụ về nghiệm lật và cổ tay nhảy gần π giữa hai điểm liền nhau (mục 12).
            mj.mju_mat2Quat(quat, (R @ Rc.T).flatten())
            mj.mju_quat2Vel(er, quat, 1.0)
            if np.linalg.norm(ep) < tol and np.linalg.norm(er) < 1e-3:
                q = wrap_near(q, q0)
                return q if np.all(q >= self.lower) and np.all(q <= self.upper) else None
            mj.mj_comPos(m, d)
            mj.mj_jacSite(m, d, jacp, jacr, self.tcp)
            J = np.vstack([jacp[:, self.cols], jacr[:, self.cols]])
            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(6), np.concatenate([ep, er]))
            q = q + np.clip(dq, -0.2, 0.2)
        return None

    def line(self, q, top, bottom, R, n: int = 10, max_step: float = 0.3):
        """Các cấu hình dọc đoạn thẳng top → bottom của TCP, khởi đầu ấm từ q. None nếu IK đứt giữa chừng hoặc có khớp
        nhảy quá max_step rad giữa hai điểm liền nhau (đổi nhánh)."""
        qs = [np.asarray(q, float)]
        for i in range(1, n + 1):
            qi = self.solve(top + (bottom - top) * i / n, R, qs[-1])
            if qi is None or np.max(np.abs(qi - qs[-1])) > max_step:
                return None
            qs.append(qi)
        return np.array(qs)
