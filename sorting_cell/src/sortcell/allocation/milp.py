"""Phân việc tham chiếu offline (mục 5.2, mức 3): mô hình kiểu Oberthür et al. (IFAC 2025) giải bằng OR-Tools CP-SAT.

Lập lịch nhiều máy không đồng nhất, một công đoạn: mỗi vật do đúng một tay làm (gắp rồi thả vào khay đúng màu của tay đó),
mỗi tay làm tuần tự, hai việc (k, i) và (k', i') của hai tay khác nhau có hitbox elip giao nhau thì không được chạy chồng
thời gian. Mục tiêu: makespan nhỏ nhất. Biết cả 8 vật từ đầu.

Thời lượng việc (k, i) lấy từ bảng chi phí: tới vật (trung bình từ 3 khay) + từ vật về khay đúng màu, chia VMAX, cộng thời
gian pha gắp và thả của script. Mô hình bỏ qua phụ thuộc thứ tự (như bài gốc dùng thời lượng cố định cho mỗi cặp).

Cùng mô hình này cũng dùng để chấm điểm các luật phân việc tham lam (mức 0, mức 1), để đo khoảng cách tới tối ưu.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from ortools.sat.python import cp_model

from ..control.arm_fsm import T_CLOSE, T_DOWN, T_OPEN, T_PLACE_DOWN, T_PLACE_UP, T_UP
from ..scene.builder import COLOR_NAMES, Layout
from .costs import CostTable
from .greedy import PENALTY
from .hitbox import Ellipse

LOCAL_T = T_DOWN + T_CLOSE + T_UP + T_PLACE_DOWN + T_OPEN + T_PLACE_UP
SCALE = 10  # đơn vị nguyên của CP-SAT: 0,1 s


@dataclass
class Instance:
    p: np.ndarray                  # (K, I) thời lượng (s), inf = không làm được
    conflict: np.ndarray           # (K, I, K, I) bool
    xy: np.ndarray
    colors: list[str]


def make_instance(L: Layout, tables: list[CostTable], cubes) -> Instance:
    K, I = L.n_arms, len(cubes)
    xy = np.array([(c.x, c.y) for c in cubes])
    p = np.full((K, I), np.inf)
    for k in range(K):
        for i, c in enumerate(cubes):
            to = np.mean([tables[k].t_tray_to(j, xy[i]) for j in range(3)])
            back = tables[k].t_tray_to(COLOR_NAMES.index(c.color), xy[i])
            if np.isfinite(to + back):
                p[k, i] = to + back + LOCAL_T
    ell = [[Ellipse(L.base(k)[:2], xy[i]) for i in range(I)] for k in range(K)]
    conflict = np.zeros((K, I, K, I), bool)
    for k in range(K):
        for i in range(I):
            for k2 in range(K):
                if k2 == k:
                    continue
                for i2 in range(I):
                    if i2 == i:
                        continue
                    e1, e2 = ell[k][i], ell[k2][i2]
                    conflict[k, i, k2, i2] = (e1.intersects(e2) or e1.contains(xy[i2][None]).any()
                                              or e2.contains(xy[i][None]).any())
    return Instance(p, conflict, xy, [c.color for c in cubes])


def solve_cpsat(inst: Instance, time_limit: float = 20.0, workers: int = 8) -> dict:
    K, I = inst.p.shape
    m = cp_model.CpModel()
    dur = np.where(np.isfinite(inst.p), np.round(inst.p * SCALE), 0).astype(int)
    horizon = int(dur.sum())
    iv, pres, start, end = {}, {}, {}, {}
    for k in range(K):
        for i in range(I):
            if not np.isfinite(inst.p[k, i]):
                continue
            pres[k, i] = m.NewBoolVar(f"x{k}_{i}")
            start[k, i] = m.NewIntVar(0, horizon, f"s{k}_{i}")
            end[k, i] = m.NewIntVar(0, horizon, f"e{k}_{i}")
            iv[k, i] = m.NewOptionalIntervalVar(start[k, i], int(dur[k, i]), end[k, i], pres[k, i], f"iv{k}_{i}")
    for i in range(I):
        m.AddExactlyOne(pres[k, i] for k in range(K) if (k, i) in pres)
    for k in range(K):
        m.AddNoOverlap([iv[k, i] for i in range(I) if (k, i) in iv])
    for (k, i) in iv:
        for (k2, i2) in iv:
            if (k, i) < (k2, i2) and k != k2 and i != i2 and inst.conflict[k, i, k2, i2]:
                m.AddNoOverlap([iv[k, i], iv[k2, i2]])
    mk = m.NewIntVar(0, horizon, "makespan")
    for key in iv:
        m.Add(mk >= end[key]).OnlyEnforceIf(pres[key])
    m.Minimize(mk)
    s = cp_model.CpSolver()
    s.parameters.max_time_in_seconds = time_limit
    s.parameters.num_workers = workers
    st = s.Solve(m)
    assign = {i: k for (k, i) in pres if s.Value(pres[k, i])} if st in (cp_model.OPTIMAL, cp_model.FEASIBLE) else {}
    return {"status": s.StatusName(st), "makespan": s.Value(mk) / SCALE if assign else None,
            "bound": s.BestObjectiveBound() / SCALE, "time": s.WallTime(), "assign": assign}


def simulate_rule(inst: Instance, L: Layout, rule: str, penalty: float = PENALTY) -> dict:
    """Chấm một luật tham lam trong cùng mô hình: tay rảnh sớm nhất chọn việc; việc chỉ bắt đầu khi mọi việc tranh chấp
    đang chạy đã xong (ràng buộc của mô hình)."""
    K, I = inst.p.shape
    free = np.zeros(K)
    running: list[tuple[int, int, float, float]] = []  # (k, i, bắt đầu, kết thúc)
    todo = set(range(I))
    owner = None
    if rule == "level0":
        owner = {}
        for i in range(I):
            order = sorted(range(K), key=lambda k: np.hypot(*(inst.xy[i] - np.array(L.base(k)[:2]))))
            owner[i] = next(k for k in order if np.isfinite(inst.p[k, i]))
    blocked = np.zeros(K, bool)
    while todo:
        cand_arms = [k for k in range(K) if not blocked[k]]
        if not cand_arms:
            break
        k = min(cand_arms, key=lambda a: free[a])
        t = free[k]
        options = [i for i in todo if np.isfinite(inst.p[k, i]) and (owner is None or owner[i] == k)]
        if not options:
            blocked[k] = True
            continue

        def earliest(i):
            s, moved = t, True
            while moved:  # dời tới khi không chồng với việc tranh chấp nào
                moved = False
                for (k2, i2, s2, e2) in running:
                    if k2 != k and inst.conflict[k, i, k2, i2] and e2 > s and s2 < s + inst.p[k, i]:
                        s, moved = e2, True
            return s

        if rule == "level1":
            active = [(k2, i2) for (k2, i2, s2, e2) in running if k2 != k and s2 <= t < e2]
            cost = {i: inst.p[k, i] + penalty * sum(inst.conflict[k, i, k2, i2] for k2, i2 in active) for i in options}
            i = min(options, key=lambda j: cost[j])
        else:
            i = min(options, key=lambda j: np.linalg.norm(inst.xy[j]))  # mức 0: thứ tự bất kỳ trong vùng
        s = earliest(i)
        e = s + inst.p[k, i]
        running.append((k, i, s, e))
        free[k] = e
        todo.discard(i)
    return {"makespan": float(max(e for *_, e in running)), "assign": {i: k for k, i, _, _ in running}}
