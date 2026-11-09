"""IK số cho một tay: đưa đầu dụng cụ tới một điểm với trục dụng cụ theo hướng cho trước, xoay quanh trục để tự do."""
from __future__ import annotations

import mujoco as mj
import numpy as np

from .mujoco_cell import DOF, ArmSpace


def solve(arm: ArmSpace, target: np.ndarray, axis: np.ndarray, q0: np.ndarray, iters: int = 300,
          damping: float = 1e-2, tol: float = 1e-4) -> np.ndarray | None:
    """Damped least squares trên vị trí (3) và hướng trục z của site (2 bậc thực sự). Trả None nếu không hội tụ."""
    m, d = arm.model, arm.data
    axis = axis / np.linalg.norm(axis)
    q = q0.copy()
    jacp = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))
    cols = np.array([m.jnt_dofadr[m.joint(f"a{arm.arm}_{j}").id] for j in
                     ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                      "wrist_3_joint")])
    for _ in range(iters):
        d.qpos[arm.idx] = q
        mj.mj_kinematics(m, d)
        mj.mj_comPos(m, d)
        pos = d.site_xpos[arm.tip]
        z = d.site_xmat[arm.tip].reshape(3, 3)[:, 2]
        err = np.concatenate([target - pos, np.cross(z, axis)])
        if np.linalg.norm(err[:3]) < tol and np.linalg.norm(err[3:]) < tol * 10:
            return np.clip((q + np.pi) % (2 * np.pi) - np.pi, arm.lower, arm.upper)
        mj.mj_jacSite(m, d, jacp, jacr, arm.tip)
        J = np.vstack([jacp[:, cols], jacr[:, cols]])
        dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(6), err)
        q = q + np.clip(dq, -0.3, 0.3)
    return None


def solutions(arm: ArmSpace, target, axis, rng: np.random.Generator, tries: int = 200,
              distinct: float = 0.3) -> list[np.ndarray]:
    """Các nghiệm IK không va chạm (với môi trường và tự va chạm) từ nhiều điểm khởi đầu ngẫu nhiên, đã loại trùng."""
    found: list[np.ndarray] = []
    target, axis = np.asarray(target, float), np.asarray(axis, float)
    for _ in range(tries):
        q = solve(arm, target, axis, rng.uniform(arm.lower, arm.upper))
        if q is None or not arm.config_free(q):
            continue
        pos, _ = arm.tip_pose(q)
        if np.linalg.norm(pos - target) > 1e-3:
            continue  # bị kẹp bởi giới hạn khớp
        if all(np.max(np.abs(q - p)) > distinct for p in found):
            found.append(q)
    return found


def pick_query(arms: list[ArmSpace], pair_free, starts: list[list[np.ndarray]],
               goals: list[list[np.ndarray]]) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Chọn một nghiệm IK cho start và goal của mỗi tay sao cho cấu hình hợp ở start và goal không va chạm giữa các tay,
    ưu tiên tổng khoảng cách khớp nhỏ (start và goal cùng nhánh IK)."""
    choices = []
    for k in range(len(arms)):
        pairs = sorted(((float(np.linalg.norm(g - s)), s, g) for s in starts[k] for g in goals[k]), key=lambda x: x[0])
        if not pairs:
            raise ValueError(f"tay {k} không có nghiệm IK")
        choices.append(pairs[:6])

    def search(k, chosen):
        if k == len(arms):
            s = np.concatenate([c[1] for c in chosen])
            g = np.concatenate([c[2] for c in chosen])
            return (s, g) if pair_free(s) and pair_free(g) else None
        for c in choices[k]:
            out = search(k + 1, chosen + [c])
            if out is not None:
                return out
        return None

    found = search(0, [])
    if found is None:
        raise ValueError("không có tổ hợp start/goal nào tự do giữa các tay")
    s, g = found
    return [s[DOF * k:DOF * (k + 1)] for k in range(len(arms))], [g[DOF * k:DOF * (k + 1)] for k in range(len(arms))]
