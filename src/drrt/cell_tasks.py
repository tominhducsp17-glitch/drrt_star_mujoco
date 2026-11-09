"""Tác vụ trên cell UR10e: từ điểm đặt dụng cụ tới cấu hình khớp start/goal, roadmap có cache, và không gian hợp."""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from . import ik
from .mujoco_cell import DOF, ArmSpace, Box, CellPairChecker, CellSpec, four_arm_cell, two_arm_cell
from .roadmap import Roadmap, r_star
from .tensor_graph import TensorGraph

DOWN = (0.0, 0.0, -1.0)


@dataclass
class Task:
    name: str
    cell: CellSpec
    points: Sequence[tuple[tuple[float, float, float], tuple[float, float, float]]]  # (start, goal) của đầu dụng cụ


def crossing_two_arms() -> Task:
    """Mỗi tay đưa dụng cụ từ một bên cột sang bên kia, trong nửa bàn phía tay đối diện: hai tay phải đi chéo qua nhau
    và qua cột, đường ngắn nhất riêng lẻ của từng tay va nhau."""
    return Task("crossing2", two_arm_cell(), [((0.15, 0.3, 0.9), (0.15, -0.3, 0.9)),
                                              ((-0.15, -0.3, 0.9), (-0.15, 0.3, 0.9))])


def rotate_four_arms() -> Task:
    """Bốn tay quanh bàn vuông, mỗi tay chuyển dụng cụ từ góc bàn bên trái sang góc bên phải của nửa bàn phía mình.
    Mỗi góc bàn là điểm đến của tay này và điểm đi của tay bên cạnh, nên các tay tranh nhau vùng góc."""
    cell = four_arm_cell()
    points = []
    for k in range(4):
        yaw = math.pi / 2 * k
        c, s = math.cos(yaw), math.sin(yaw)
        local = ((-0.1, 0.25), (-0.1, -0.25))  # toạ độ quanh tâm bàn, trong hệ quay theo tay: x hướng ra xa tay
        world = [(x * c - y * s, x * s + y * c, 0.9) for x, y in local]
        points.append((world[0], world[1]))
    return Task("rotate4", cell, points)


def _two_arm_variant(name: str, spacing: float, pole: bool, bar: bool, points, bar_height: float = 1.4) -> Task:
    cell = two_arm_cell(spacing, pole)
    if bar:
        # Thanh ngang phía trên bàn, dọc theo trục y: chặn đường "nhấc cao lên rồi vòng qua" của các tay.
        cell.obstacles = list(cell.obstacles) + [Box("bar", (0.0, 0.0, bar_height), (0.04, 0.6, 0.04), (0.85, 0.35, 0.2, 1))]
    return Task(name, cell, points)


def swap_two_arms(spacing: float = 1.5, pole: bool = False, bar: bool = True, bar_height: float = 1.3) -> Task:
    """Hai dụng cụ đổi chỗ cho nhau ở giữa bàn: tay 0 đi từ +y sang -y, tay 1 ngược lại, gần như cùng một đường. Một tay
    phải nhường (vòng hoặc chờ); thanh ngang phía trên hạn chế cách vượt qua bằng cách nhấc cao."""
    return _two_arm_variant("swap2", spacing, pole, bar, [((0.05, 0.3, 0.9), (0.05, -0.3, 0.9)),
                                                         ((-0.05, -0.3, 0.9), (-0.05, 0.3, 0.9))], bar_height)


TASKS = {"crossing2": crossing_two_arms, "swap2": swap_two_arms, "rotate4": rotate_four_arms}


@dataclass
class Problem:
    task: Task
    arms: list[ArmSpace]
    pair: CellPairChecker
    start: list[np.ndarray]
    goal: list[np.ndarray]

    @property
    def R(self) -> int:
        return len(self.arms)


def make_problem(task: Task, seed: int = 0) -> Problem:
    arms = [ArmSpace(task.cell, k) for k in range(task.cell.n_arms)]
    pair = CellPairChecker(task.cell)
    rng = np.random.default_rng(seed)
    starts = [ik.solutions(arm, p[0], DOWN, rng) for arm, p in zip(arms, task.points)]
    goals = [ik.solutions(arm, p[1], DOWN, rng) for arm, p in zip(arms, task.points)]
    start, goal = ik.pick_query(arms, pair.config_free, starts, goals)
    return Problem(task, arms, pair, start, goal)


def free_volume(arm: ArmSpace, samples: int = 20000) -> float:
    """μ(C_free) cho Theorem 1: thể tích hộp khớp nhân tỉ lệ tự do ước lượng bằng lấy mẫu."""
    return float(np.prod(arm.upper - arm.lower)) * arm.free_fraction(samples, np.random.default_rng(12345))


def roadmaps(problem: Problem, n: int, seed: int, cache: Path | None = None,
             k: int | None = None) -> list[Roadmap]:
    """Roadmap của từng tay (s_i, t_i ở chỉ số 0, 1), dựng lại tới khi s_i và t_i liên thông. Có cache theo tham số."""
    out = []
    # Khoá cache gồm cả mô tả cell và start/goal, không chỉ tên tác vụ: đổi vật cản thì roadmap cũ không còn đúng.
    key = hashlib.sha1(repr((problem.task.cell, [np.round(q, 9).tolist() for q in problem.start + problem.goal]))
                       .encode()).hexdigest()[:10]
    for arm, s, g in zip(problem.arms, problem.start, problem.goal):
        tag = f"{problem.task.name}_{key}_a{arm.arm}_n{n}_s{seed}" + (f"_k{k}" if k else "")
        path = cache / f"{tag}.npz" if cache else None
        if path is not None and path.exists():
            out.append(Roadmap.load(path))
            continue
        rng = np.random.default_rng([n, seed, arm.arm])
        radius = None if k else r_star(n, DOF, free_volume(arm))
        while True:
            rm = Roadmap.build(arm, n, rng, radius=radius, k=k, required=[s, g])
            if rm.connected(0, 1):
                break
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            rm.save(path)
        out.append(rm)
    return out


def tensor_graph(problem: Problem, rms: list[Roadmap], cost: str = "sum") -> TensorGraph:
    return TensorGraph(rms, problem.pair, (0,) * problem.R, (1,) * problem.R, cost=cost)


class CompositeSpace:
    """Không gian hợp 6R chiều cho RRT*: tự do khi từng tay tự do với môi trường và các tay không va nhau."""

    def __init__(self, problem: Problem):
        self.p = problem
        self.R = problem.R
        self.lower = np.concatenate([a.lower for a in problem.arms])
        self.upper = np.concatenate([a.upper for a in problem.arms])

    def edge_free(self, a, b) -> bool:
        a, b = np.asarray(a, float), np.asarray(b, float)
        parts_a = [a[DOF * k:DOF * (k + 1)] for k in range(self.R)]
        parts_b = [b[DOF * k:DOF * (k + 1)] for k in range(self.R)]
        return (all(arm.edge_free(x, y) for arm, x, y in zip(self.p.arms, parts_a, parts_b))
                and self.p.pair.edge_free(parts_a, parts_b))
