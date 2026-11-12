"""Thực thi đường đi hình học trên model động lực học: tham số hoá thời gian, bám bằng actuator vị trí, kiểm tra va chạm.

Mỗi đoạn của đường đi chạy theo đa thức bậc năm với vận tốc và gia tốc bằng 0 ở hai đầu, mọi tay cùng thời gian. Cách này
giữ nguyên hình học của đường (chỉ đổi tốc độ dọc theo đường), nên tính tự do va chạm của planner vẫn còn đúng; đổi lại
robot dừng ở mỗi đỉnh.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import mujoco as mj
import numpy as np

from .mujoco_cell import CellSpec, _qpos_indices, execution_model


@dataclass
class Trajectory:
    times: np.ndarray        # thời điểm của từng đỉnh
    waypoints: np.ndarray    # (k, 6R)

    @property
    def duration(self) -> float:
        return float(self.times[-1])

    def sample(self, t: float) -> np.ndarray:
        if t >= self.times[-1]:
            return self.waypoints[-1]
        i = int(np.searchsorted(self.times, t, side="right")) - 1
        s = (t - self.times[i]) / (self.times[i + 1] - self.times[i])
        s = s * s * s * (10 - 15 * s + 6 * s * s)
        return self.waypoints[i] + (self.waypoints[i + 1] - self.waypoints[i]) * s

    def velocity(self, t: float) -> np.ndarray:
        if t >= self.times[-1]:
            return np.zeros(self.waypoints.shape[1])
        i = int(np.searchsorted(self.times, t, side="right")) - 1
        T = self.times[i + 1] - self.times[i]
        s = (t - self.times[i]) / T
        return (self.waypoints[i + 1] - self.waypoints[i]) * (30 * s * s * (1 - s) ** 2 / T)


def time_parameterize(waypoints: Sequence[np.ndarray], vmax: float = 1.0, amax: float = 2.0) -> Trajectory:
    """Thời lượng mỗi đoạn theo khớp đi xa nhất: đa thức bậc năm có vận tốc đỉnh 1,875 L/T, gia tốc đỉnh 5,77 L/T²."""
    w = np.asarray(waypoints, float)
    times = [0.0]
    for a, b in zip(w, w[1:]):
        L = float(np.max(np.abs(b - a)))
        times.append(times[-1] + max(1.875 * L / vmax, math.sqrt(5.7735 * L / amax), 1e-3))
    return Trajectory(np.array(times), w)


@dataclass
class Execution:
    max_tracking_error: float
    collision_steps: int
    collision_pairs: set[tuple[str, str]] = field(default_factory=set)
    frames: list[np.ndarray] = field(default_factory=list)


def execute(cell: CellSpec, traj: Trajectory, settle: float = 0.5, render_fps: float = 0.0,
            size: tuple[int, int] = (480, 854)) -> Execution:
    """Chạy mj_step. Bù trọng lực bằng gravcomp; bộ điều khiển là actuator vị trí có sẵn của Menagerie
    (lực = kp(ctrl - q) - kd q̇, kp=5000, kd=500). Đặt ctrl = q_ref + (kd/kp) q̇_ref để có thêm feedforward vận tốc, nếu không
    robot trễ khoảng kd/kp = 0,1 s so với quỹ đạo."""
    m = execution_model(cell)
    for b in range(m.nbody):
        name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, b) or ""
        if name.startswith("a") and name[1:2].isdigit():
            m.body_gravcomp[b] = 1.0
    d = mj.MjData(m)
    idx = np.concatenate([_qpos_indices(m, k) for k in range(cell.n_arms)])
    act = np.concatenate([[m.actuator(f"a{k}_{j}").id for j in
                           ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")]
                          for k in range(cell.n_arms)])
    lead = m.actuator_biasprm[act, 2] / m.actuator_biasprm[act, 1]  # (-kd)/(-kp)
    q0 = traj.sample(0.0)
    d.qpos[idx] = q0
    d.ctrl[act] = q0
    mj.mj_forward(m, d)

    renderer = camera = None
    if render_fps > 0:
        renderer = mj.Renderer(m, *size)
        camera = mj.MjvCamera()
        camera.lookat[:] = (0, 0, 0.9)
        camera.distance, camera.azimuth, camera.elevation = 3.2, 135, -28
    result = Execution(0.0, 0)
    next_frame = 0.0
    names = lambda g: mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "world"
    while d.time < traj.duration + settle:
        ref = traj.sample(d.time)
        d.ctrl[act] = ref + lead * traj.velocity(d.time)
        mj.mj_step(m, d)
        result.max_tracking_error = max(result.max_tracking_error, float(np.max(np.abs(d.qpos[idx] - ref))))
        hit = False
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist < 0:
                hit = True
                result.collision_pairs.add(tuple(sorted((names(c.geom1), names(c.geom2)))))
        result.collision_steps += hit
        if renderer is not None and d.time >= next_frame:
            renderer.update_scene(d, camera)
            result.frames.append(renderer.render().copy())
            next_frame += 1 / render_fps
    if renderer is not None:
        renderer.close()
    return result


def save_video(frames: Sequence[np.ndarray], path, fps: float) -> None:
    import imageio.v2 as imageio

    with imageio.get_writer(path, fps=fps, codec="libx264", quality=7, macro_block_size=1) as writer:
        for f in frames:
            writer.append_data(f)
