"""Quỹ đạo liền mạch (ngoài bài): spline bậc ba qua các đỉnh, kiểm tra va chạm lại trên spline, rồi TOPP-RA.

Spline không trùng đường gấp khúc mà planner đã kiểm tra, nên phải kiểm tra lại. Đoạn nào va chạm thì chèn thêm điểm của
đường gấp khúc vào đoạn đó để kéo spline sát lại, rồi lặp. Không được thì trả None để dùng cách dừng ở từng đỉnh.
"""
from __future__ import annotations

import warnings
from typing import Callable, Sequence

import numpy as np

with warnings.catch_warnings():
    warnings.simplefilter("ignore", SyntaxWarning)  # docstring của toppra
    import toppra as ta
    import toppra.algorithm as ta_algo
    import toppra.constraint as ta_constraint

ta.setup_logging("WARNING")


class SplineTrajectory:
    """Cùng giao diện với execute.Trajectory: sample, velocity, duration."""

    def __init__(self, traj, path, knots: np.ndarray):
        self._traj = traj
        self.path = path
        self.knots = knots

    @property
    def duration(self) -> float:
        return float(self._traj.duration)

    def sample(self, t: float) -> np.ndarray:
        return np.asarray(self._traj(min(max(t, 0.0), self.duration)))

    def velocity(self, t: float) -> np.ndarray:
        if t >= self.duration:
            return np.zeros(self.knots.shape[1])
        return np.asarray(self._traj(max(t, 0.0), 1))


def _dedupe(points: Sequence[np.ndarray]) -> list[np.ndarray]:
    out = [np.asarray(points[0], float)]
    for p in points[1:]:
        if np.max(np.abs(p - out[-1])) > 1e-6:
            out.append(np.asarray(p, float))
    return out


def _spline(points: list[np.ndarray]):
    w = np.array(points)
    chord = np.concatenate([[0.0], np.cumsum(np.max(np.abs(np.diff(w, axis=0)), axis=1))])
    return ta.SplineInterpolator(chord / chord[-1], w, bc_type="clamped"), chord / chord[-1]


def _first_collision(path, knots: np.ndarray, edge_free: Callable, resolution: float) -> int | None:
    """Chỉ số đoạn (giữa hai đỉnh) có va chạm đầu tiên trên spline, hoặc None."""
    for i in range(len(knots) - 1):
        span = path(np.linspace(knots[i], knots[i + 1], 9))
        steps = max(1, int(np.ceil(np.max(np.abs(np.diff(span, axis=0)).sum(axis=0)) / resolution)))
        samples = path(np.linspace(knots[i], knots[i + 1], steps + 1))
        for a, b in zip(samples, samples[1:]):
            if not edge_free(a, b):
                return i
    return None


def smooth(points: Sequence[np.ndarray], edge_free: Callable, vmax: float = 1.0, amax: float = 2.0,
           resolution: float = 0.05, max_rounds: int = 8, grid: int = 400) -> SplineTrajectory | None:
    pts = _dedupe(points)
    if len(pts) < 2:
        return None
    for _ in range(max_rounds):
        path, knots = _spline(pts)
        bad = _first_collision(path, knots, edge_free, resolution)
        if bad is None:
            break
        a, b = pts[bad], pts[bad + 1]
        pts = pts[:bad + 1] + [a + (b - a) * f for f in (0.25, 0.5, 0.75)] + pts[bad + 1:]
    else:
        return None
    dof = pts[0].shape[0]
    constraints = [ta_constraint.JointVelocityConstraint(np.tile([[-vmax, vmax]], (dof, 1))),
                   ta_constraint.JointAccelerationConstraint(np.tile([[-amax, amax]], (dof, 1)))]
    instance = ta_algo.TOPPRA(constraints, path, gridpoints=np.linspace(0, 1, grid + 1),
                              parametrizer="ParametrizeConstAccel")
    traj = instance.compute_trajectory()
    if traj is None:
        return None
    return SplineTrajectory(traj, path, np.array(pts))
