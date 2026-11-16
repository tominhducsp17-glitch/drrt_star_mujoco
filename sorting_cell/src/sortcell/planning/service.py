"""PlannerService (mục 9): một truy vấn dRRT* cho các tay đang di chuyển, các tay còn lại là tập cấu hình cố định.

- Cấu hình hiện tại và đích của tay di chuyển được chèn vào roadmap của tay đó thành đỉnh tạm (ArmRoadmap.with_extra).
- Tay đứng yên hoặc đang gắp/thả không nằm trong Ĝ (planning.query.QueryChecker).
- Ngân sách thời gian cố định; không ra lời giải thì tìm tiếp trên cùng cây (từng khoảng bằng ngân sách) tới khi có lời
  giải hoặc hết tổng ngân sách max_total. dRRT* là anytime nên dùng nghiệm tốt nhất có được trong ngân sách. Không có giới
  hạn tổng thì truy vấn vô nghiệm (đường bị chặn bởi vùng dành riêng của tay đang gắp) đốt hết thời gian: M2 bản đầu gấp
  đôi ngân sách và dựng lại cây mỗi lần, tới 18,9 s cho một lần thất bại.
- Rút gọn đường đi: từ mỗi đỉnh nối thẳng tới đỉnh xa nhất còn nối được (kiểm tra cả môi trường lẫn giữa các tay, cùng
  checker như planner), để bớt số lần dừng ở đỉnh khi thực thi.

M1 gọi theo lượt (mọi tay xuất phát cùng lúc). M2 gọi khi có sự kiện, từ cấu hình hiện tại của các tay đang di chuyển.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from drrt.planners.drrt_star import DRRTStar
from drrt.tensor_graph import TensorGraph

from ..scene.builder import Layout
from .models import ArmSpace, PairChecker
from .query import QueryChecker
from .roadmaps import ArmRoadmap, build_roadmap


@dataclass
class PlanResult:
    path: list[np.ndarray] | None      # cấu hình hợp 24 chiều dọc đường đi; None nếu thất bại
    reason: str = ""
    planning_time: float = 0.0
    first_time: float | None = None
    iterations: int = 0
    budget_used: float = 0.0
    cost: float = float("inf")
    raw_vertices: int = 0
    moving: tuple[int, ...] = ()


class PlannerService:
    def __init__(self, L: Layout, n_task: int = 1500, n_uniform: int = 1500, seed: int = 0, budget: float = 0.5,
                 max_total: float = 1.0, shortcut: bool = True):
        self.L = L
        self.spaces = [ArmSpace(L, k) for k in range(L.n_arms)]
        self.rms = [ArmRoadmap(build_roadmap(L, k, n_task, n_uniform, seed), self.spaces[k]) for k in range(L.n_arms)]
        self.pair = PairChecker(L)
        self.budget, self.max_total, self.shortcut = budget, max_total, shortcut

    # ------------------------------------------------------------------ kiểm tra tĩnh
    def composite_free(self, qs) -> bool:
        return self.pair.config_free(np.concatenate(qs))

    def sets_free(self, sets: dict[int, list[np.ndarray]]) -> bool:
        """Mọi tổ hợp (tay i ở một cấu hình trong tập của i, tay j ở một cấu hình trong tập của j) không va nhau,
        xét từng cặp tay."""
        arms = list(sets)
        for x in range(len(arms)):
            for y in range(x + 1, len(arms)):
                i, j = arms[x], arms[y]
                qc = QueryChecker(self.pair, [i], {j: sets[j]})
                try:
                    if not all(qc.arm_config_free(i, q) for q in sets[i]):
                        return False
                finally:
                    qc.restore()
        return True

    # ------------------------------------------------------------------ truy vấn
    def plan(self, current: list[np.ndarray], goals: dict[int, np.ndarray], fixed: dict[int, list[np.ndarray]],
             rng: np.random.Generator, budget: float | None = None) -> PlanResult:
        """current: cấu hình hiện tại của mọi tay. goals: đích của các tay di chuyển. fixed: tập cấu hình của các tay
        còn lại (mặc định {cấu hình hiện tại})."""
        t0 = time.perf_counter()
        moving = sorted(k for k, g in goals.items() if not np.allclose(g, current[k]))
        fx = {k: fixed.get(k, [current[k]]) for k in range(self.L.n_arms) if k not in moving}
        if not moving:
            return PlanResult([np.concatenate(current)], "đứng yên", time.perf_counter() - t0, cost=0.0)
        qc = QueryChecker(self.pair, moving, fx)
        try:
            starts = [current[k] for k in moving]
            ends = [goals[k] for k in moving]
            if not qc.config_free(starts):
                return PlanResult(None, "start va tay khác", time.perf_counter() - t0, moving=tuple(moving))
            if not qc.config_free(ends):
                return PlanResult(None, "đích va tay khác", time.perf_counter() - t0, moving=tuple(moving))
            rms, s_idx, t_idx = [], [], []
            for k in moving:
                out = self.rms[k].with_extra([current[k], goals[k]])
                if out is None:
                    return PlanResult(None, f"tay {k}: không nối được đỉnh tạm", time.perf_counter() - t0,
                                      moving=tuple(moving))
                rm, (i, j) = out
                rms.append(rm)
                s_idx.append(i)
                t_idx.append(j)
            graph = TensorGraph(rms, qc, tuple(s_idx), tuple(t_idx))
            b = budget or self.budget
            planner = DRRTStar(graph, rng)
            res, used = planner.solve(time_limit=b), b
            while not res.solved and used < self.max_total - 1e-9:
                step = min(b, self.max_total - used)
                res = planner.solve(time_limit=step)  # cây giữ nguyên giữa các lần gọi
                used += step
            if not res.solved:
                return PlanResult(None, f"dRRT* không ra lời giải trong {used:.1f} s", time.perf_counter() - t0,
                                  iterations=res.iterations, budget_used=used, moving=tuple(moving))
            path = [graph.flat(v) for v in res.path]
            raw = len(path)
            if self.shortcut:
                path = self._shortcut(path, moving, qc)
            full = []
            for p in path:
                q = np.concatenate(current).copy()
                for j, k in enumerate(moving):
                    q[6 * k:6 * k + 6] = p[6 * j:6 * j + 6]
                full.append(q)
            return PlanResult(full, "ok", time.perf_counter() - t0, res.first_time, res.iterations, used, res.cost, raw,
                              tuple(moving))
        finally:
            qc.restore()

    def _shortcut(self, path, moving, qc: QueryChecker):
        def ok(a, b):
            for j, k in enumerate(moving):
                s = slice(6 * j, 6 * j + 6)
                if not self.spaces[k].edge_free(a[s], b[s]):
                    return False
            return qc.edge_free([a[6 * j:6 * j + 6] for j in range(len(moving))],
                                [b[6 * j:6 * j + 6] for j in range(len(moving))])

        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not ok(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out


# Tên cũ dùng trong M1
RoundPlanner = PlannerService
