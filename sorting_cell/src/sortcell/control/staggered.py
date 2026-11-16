"""Điều phối cell M2: các tay xong việc lệch giờ, lập kế hoạch lại theo sự kiện (mở rộng 1 và 2 của mục 5.3).

Trạng thái mỗi tay: IDLE (không có đích), TRANSIT (trong Ĝ, đang tới đích), WAIT_LOCAL (đã tới tư thế chuẩn bị gắp/thả,
chờ được phép đi xuống), LOCAL (đang chạy kịch bản gắp hoặc thả).

Vòng lặp:
1. Kịch bản cục bộ xong → xử lý kết quả (gắp được hay không, đã thả) → tay về IDLE.
2. Tay WAIT_LOCAL được đi xuống khi tập quét của nó (vùng dành riêng) không va: cấu hình hiện tại của các tay khác, tập
   cố định của các tay khác, và phần đường còn lại của các tay đang TRANSIT. Nếu chỉ vướng phần đường còn lại thì lập
   kế hoạch lại cho các tay TRANSIT với tập quét này là vật cản.
3. Tay IDLE nhận việc (mức 0 hoặc mức 1). Đích hợp lệ khi tập quét ở đích không va tập đích/tập cố định của mọi tay khác,
   nên một tay tới đích không bao giờ chặn đích của tay khác.
4. Có thay đổi → một truy vấn dRRT* cho mọi tay TRANSIT, xuất phát từ cấu hình hiện tại (đỉnh tạm), tay khác là tập cố
   định (PlannerService). Thất bại → huỷ các việc gắp vừa giao, các tay đang đi đứng yên, thử lại sau COOLDOWN giây
   mô phỏng (không thử lại ngay: mỗi lần thất bại tốn trọn ngân sách lập kế hoạch). Thất bại từ lần thứ hai liên tiếp:
   các tay rảnh không ở HOME được cho về HOME, vì có thể chính chúng đang chắn đường.
5. Chạy cạnh kế tiếp của đường đi hợp (mọi tay TRANSIT dừng ở cuối cạnh), song song với các kịch bản cục bộ.
   Không có đường đi → chạy 0,05 s. Sự kiện xảy ra giữa cạnh được xử lý ở cuối cạnh.

Mô phỏng tạm dừng trong lúc lập kế hoạch; thời gian lập kế hoạch được đo riêng (độ trễ lập kế hoạch lại).
"""
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field

import numpy as np

from drrt.execute import time_parameterize

from ..allocation.costs import CostTable
from ..allocation.greedy import rank_level1
from ..allocation.hitbox import Ellipse
from ..allocation.zones import assign_level0
from ..planning.query import QueryChecker
from ..planning.service import PlannerService
from ..scene.builder import COLOR_NAMES, CUBE, Layout
from ..scene.sampling import MIN_DIST, CubeSpec
from ..skills.ik import HOME
from ..skills.scripted import ArmTools, LocalMove
from .arm_fsm import (AMAX, LIFT_OK, MAX_ATTEMPTS, T_CLOSE, T_DOWN, T_OPEN, T_PLACE_DOWN, T_PLACE_UP, T_UP, VMAX)
from .executor import GRIP_CLOSE, GRIP_OPEN, CellSim, LineSeg, Step
from ..perception.tracker import yaw_diff
from .world import PerceivedWorld, TruthWorld

IDLE_DT = 0.05
MAX_PIECE = 0.8   # rad: cạnh dài hơn được chia nhỏ để sự kiện được xử lý sớm (xem docstring của subdivide)
MAX_SIM = 400.0
COOLDOWN = 1.0
STUCK_T = 5.0       # s mô phỏng: mọi tay rảnh mà không giao được việc nào thì coi vật còn lại là bị vây
DISTURB_MM = 3.0


@dataclass
class Runner:
    start: float
    steps: list[Step]

    @property
    def end(self) -> float:
        return self.start + sum(s.duration for s in self.steps)

    def apply(self, sim: CellSim, k: int, t: float) -> None:
        s0 = self.start
        for st in self.steps:
            if t < s0 + st.duration or st is self.steps[-1]:
                sim.d.ctrl[sim.grip[k]] = st.grip
                if st.seg is not None:
                    sim.d.ctrl[sim.act[k]] = st.seg.sample(min(max(t - s0, 0.0), st.seg.T))
                return
            s0 += st.duration


@dataclass
class Arm:
    k: int
    q: np.ndarray
    state: str = "IDLE"
    action: str | None = None          # pick | place | home
    target: int | None = None
    holding: bool = False
    move: LocalMove | None = None
    truth_target: int | None = None    # chỉ số khối thật ghép với vật đích (chỉ để đánh giá)
    goal: np.ndarray | None = None
    where: int | None = None           # None = HOME/bàn, j = vừa thả ở khay j
    queue: list[int] = field(default_factory=list)
    slots: dict[int, int] = field(default_factory=dict)
    runner: Runner | None = None
    cooldown_until: float = 0.0
    sweep_ok: bool = False             # WAIT_LOCAL: tập quét đã được đưa vào tập cố định khi lập kế hoạch
    arrived_t: float = 0.0
    task: dict | None = None           # bản ghi tương tác của việc gắp hiện tại (xem StaggeredResult.tasks)
    snapshot: dict | None = None       # đánh giá: vị trí thật các khối khác lúc bắt đầu gắp

    def sweep(self) -> list[np.ndarray]:
        d = self.move.down
        return list(d[::2]) + ([d[-1]] if (len(d) - 1) % 2 else [])


@dataclass
class StaggeredResult:
    seed: int
    alloc: str
    n_cubes: int
    placed_correct: int
    placed_wrong: int
    not_in_tray: int
    picks: int
    pick_success: int
    sim_time: float
    t_last_place: float
    planning_wall: float
    replans: int
    replan_times: list
    plan_failures: int
    collision_steps: dict
    collision_pairs: dict
    max_tracking_error: float
    deadlock: bool
    events: list = field(default_factory=list)
    cube_status: list = field(default_factory=list)
    wait_local_time: float = 0.0
    # mỗi việc gắp: hitbox dự đoán tranh chấp lúc giao việc, và tương tác thật quan sát được trong lúc làm việc đó
    tasks: list = field(default_factory=list)
    perception: dict | None = None
    lift_disagree: int = 0             # số lần bộ điều khiển (cảm biến gripper) và pose thật kết luận khác nhau sau khi nhấc
    blocked: int = 0                   # vật không tay nào gắp được (bị khối hàng xóm vây) khi cell dừng
    disturbed: list = field(default_factory=list)  # khối khác bị xê dịch > DISTURB_MM trong một lần gắp
    arrivals: list = field(default_factory=list)   # khối thả thêm: t_spawn, t_detect, t_assign, t_lift (s)
    busy_time: list = field(default_factory=list)  # mỗi tay: thời gian mô phỏng không rảnh (đang đi, chờ, gắp, thả, mang vật)
    picks_per_arm: list = field(default_factory=list)


class StaggeredCell:
    def __init__(self, L: Layout, planner: PlannerService, tools: list[ArmTools], tables: list[CostTable] | None = None,
                 alloc: str = "level1", budget: float = 0.3, penalty: float | None = None, perception: bool = False,
                 split: bool = True, neighbor_check: bool = True):
        self.L, self.planner, self.tools, self.tables = L, planner, tools, tables
        self.alloc, self.budget = alloc, budget
        self.perception = perception
        self.split, self.neighbor_check = split, neighbor_check   # tắt để so sánh (mục 11, M4)
        self.world = None
        self.penalty = penalty
        self._place_cache: dict = {}
        self._pick_cache: dict = {}

    # ------------------------------------------------------------------ chuyển động cục bộ
    def pick_move(self, sim: CellSim, k: int, i: int) -> LocalMove | None:
        """Cách gắp đầu tiên (theo thứ tự hướng kẹp ưu tiên) mà đoạn đi xuống không tới gần khối nào khác; None nếu không
        có (khối bị khối hàng xóm vây). Cache theo phiên bản pose của các vật trên bàn."""
        key = (k, i)
        ver = self.world.version()
        hit = self._pick_cache.get(key)
        if hit is not None and hit[0] == ver:
            return hit[1]
        mv = self._pick_move(sim, k, i)
        self._pick_cache[key] = (ver, mv)
        return mv

    def _pick_move(self, sim: CellSim, k: int, i: int) -> LocalMove | None:
        pos, yaw = self.world.pose(i)
        cube_qpos, tgt = self.world.check_cubes(i)
        for mv in self.tools[k].pick_options(pos[:2], yaw):
            if not self.neighbor_check or not sim.descent_touches(k, mv.down, tgt, cube_qpos):
                return mv
        return None

    def place_move(self, sim: CellSim, a: Arm) -> tuple[LocalMove, int, int] | None:
        j = COLOR_NAMES.index(self.world.color(a.target))
        cube_qpos, tgt = self.world.check_cubes(None)
        if cube_qpos is None:
            tgt = a.target
        for n in range(5):
            slot = (a.slots.get(j, 0) + n) % 5
            key = (a.k, j, slot)
            if key not in self._place_cache:
                self._place_cache[key] = list(self.tools[a.k].place_options(self.L.tray_slots(a.k, j)[slot]))
            for mv in self._place_cache[key]:
                if not sim.descent_touches(a.k, mv.down, tgt, cube_qpos):
                    return mv, j, a.slots.get(j, 0) + n
        return None

    # ------------------------------------------------------------------ tập cấu hình
    def goal_set(self, a: Arm) -> list[np.ndarray]:
        """Tập mà tay a sẽ chiếm khi xong việc hiện tại."""
        if a.state in ("TRANSIT", "WAIT_LOCAL", "LOCAL") and a.move is not None and a.action in ("pick", "place"):
            return a.sweep()
        if a.state == "TRANSIT":
            return [a.goal]
        return [a.q]

    def fixed_set(self, a: Arm) -> list[np.ndarray]:
        """Tập cố định của tay không nằm trong Ĝ."""
        if a.state == "LOCAL" or (a.state == "WAIT_LOCAL" and a.sweep_ok):
            return a.sweep()
        return [a.q]

    def _set_free_vs(self, k: int, S: list[np.ndarray], others: dict[int, list[np.ndarray]]) -> bool:
        if not others:
            return True
        qc = QueryChecker(self.planner.pair, [k], others)
        try:
            return all(qc.arm_config_free(k, q) for q in S)
        finally:
            qc.restore()

    def _path_free_vs(self, path, moving, k: int, S) -> bool:
        """Phần đường còn lại của các tay di chuyển không va tập S của tay k."""
        if not path or len(path) < 2:
            return True
        qc = QueryChecker(self.planner.pair, list(moving), {k: S})
        try:
            for a, b in zip(path, path[1:]):
                for i in moving:
                    if not qc.arm_edge_free(i, a[6 * i:6 * i + 6], b[6 * i:6 * i + 6]):
                        return False
            return True
        finally:
            qc.restore()

    # ------------------------------------------------------------------ một cảnh
    def run(self, cubes: list[CubeSpec], seed: int = 0, recorder=None, verbose: bool = False,
            arrivals: list | None = None) -> StaggeredResult:
        L = self.L
        self.cubes = cubes
        sim = CellSim(L, cubes, recorder)
        sim.wait(0.3)
        world = self.world = PerceivedWorld(sim, cubes, split=self.split) if self.perception else TruthWorld(sim, cubes)
        self._handled: set[int] = set()   # đánh giá: khối thật đã từng được giao cho một tay
        pending = sorted(arrivals or [], key=lambda a_: a_.t)   # khối sẽ được thả thêm (M4 việc 2)
        self._arrivals: dict[int, dict] = {}                    # đánh giá: mốc thời gian của từng khối thả thêm
        spawn_rng = np.random.default_rng([seed, 11])
        self._pick_cache.clear()
        world.update(force=True)
        rng = np.random.default_rng(seed)
        status: dict[int, str] = {i: "on_table" for i in world.ids()}
        attempts: dict[int, int] = {i: 0 for i in status}
        lift_disagree = 0
        arms = [Arm(k, HOME.copy()) for k in range(L.n_arms)]
        if self.alloc == "level0":
            assert not self.perception, "mức 0 chỉ dùng với pose thật"

            queues, orphans = assign_level0(L, cubes, lambda k, i: self.tools[k].pick(
                (cubes[i].x, cubes[i].y), cubes[i].yaw) is not None)
            for k, q in queues.items():
                arms[k].queue = q
            for i in orphans:
                status[i] = "failed"
        events = []

        def log(k, ph, info=""):
            events.append({"t": round(sim.time, 3), "arm": k, "phase": ph, "info": info})
            if verbose:
                print(events[-1], flush=True)

        path: list[np.ndarray] = []
        moving: tuple[int, ...] = ()
        seg_times: np.ndarray | None = None
        t_start = sim.time
        self._t_start = t_start
        t_last = t_start
        replan_times, plan_failures, picks, pick_ok, max_err = [], 0, 0, 0, 0.0
        planning_wall = 0.0
        wait_local = 0.0
        deadlock = False
        need_plan = False
        next_plan_t = 0.0   # sau một lần thất bại: chờ COOLDOWN giây mô phỏng rồi mới thử lại
        stuck_since = None  # mọi tay rảnh mà còn vật trên bàn không tay nào nhận được (bị vây)
        busy = np.zeros(L.n_arms)
        picks_arm = [0] * L.n_arms
        disturbed: list[dict] = []
        fail_streak = 0
        tasks: list[dict] = []

        def replan(reason: str, extra_fixed: dict | None = None) -> bool:
            nonlocal path, moving, seg_times, planning_wall, plan_failures, next_plan_t, fail_streak
            goals = {a.k: a.goal for a in arms if a.state == "TRANSIT"}
            fixed = {a.k: self.fixed_set(a) for a in arms if a.state != "TRANSIT"}
            if extra_fixed:
                fixed.update(extra_fixed)
            res = self.planner.plan([a.q for a in arms], goals, fixed, rng, self.budget)
            planning_wall += res.planning_time
            replan_times.append(res.planning_time)
            if res.path is None:
                plan_failures += 1
                fail_streak += 1
                next_plan_t = sim.time + COOLDOWN
                log(-1, "PLAN_FAIL", f"{reason}: {res.reason}")
                path, moving = [], ()
                if fail_streak >= 2:
                    self._park_idle(arms, log)
                return False
            fail_streak = 0
            for k, n in path_waits(res.path, res.moving).items():
                if arms[k].task is not None:
                    arms[k].task["path_waits"] += n
                    arms[k].task["plans"] += 1
            path, moving = subdivide(res.path, MAX_PIECE), res.moving
            if len(path) > 1:
                seg_times = np.diff(time_parameterize(path, VMAX, AMAX).times)
            log(-1, "PLAN", f"{reason}: tay {list(moving)}, {len(path)} đỉnh, {res.planning_time*1000:.0f} ms")
            return True

        while True:
            now = sim.time
            while pending and now - t_start >= pending[0].t:   # thả thêm khối: môi trường, bộ điều phối không biết
                arr = pending[0]
                spot = self._spawn_spot(sim, arms, arr, spawn_rng)
                if spot is None:
                    break  # chưa có chỗ trống: thử lại ở vòng sau
                sim.spawn_cube(arr.cube, *spot)
                self._arrivals[arr.cube] = {"cube": arr.cube, "t_spawn": round(sim.time - t_start, 3),
                                            "x": round(spot[0], 4), "y": round(spot[1], 4)}
                log(-1, "SPAWN", f"khối thật {arr.cube} ở ({spot[0]:.3f}, {spot[1]:.3f})")
                pending.pop(0)
            world.update()
            for oid in world.ids():
                if oid not in status:
                    status[oid], attempts[oid] = "on_table", 0
                    log(-1, "NEW_OBJECT", f"vật {oid}")
                    j = world.truth(oid)
                    if j in self._arrivals and "t_detect" not in self._arrivals[j]:
                        self._arrivals[j]["t_detect"] = round(sim.time - t_start, 3)
                elif status[oid] == "on_table" and world.lost(oid):
                    status[oid] = "lost"
                    log(-1, "LOST", f"vật {oid}")
            if now - t_start > MAX_SIM:
                deadlock = True
                log(-1, "TIMEOUT")
                break
            # 1. kịch bản cục bộ xong
            for a in arms:
                if a.state == "LOCAL" and now >= a.runner.end - 1e-9:
                    i = a.target
                    a.runner = None
                    if a.action == "pick":
                        picks += 1
                        for j, p0 in (a.snapshot or {}).items():   # đánh giá: khối khác bị xê dịch trong lần gắp này
                            pj = sim.cube_pose(j)[0]
                            if j in self._handled:
                                continue  # khối đã được giao cho một tay (đang gắp, đang mang, đã thả): không tính
                            dd = float(np.linalg.norm(pj - p0))
                            if dd > DISTURB_MM / 1000:
                                disturbed.append({"t": round(sim.time, 3), "arm": a.k, "cube": j, "mm": round(dd * 1000, 1)})
                        a.snapshot = None
                        lifted = sim.lifted_cube(a.k, LIFT_OK)      # đánh giá: khối thật nào đang ở trong gripper
                        if lifted is not None:
                            self._handled.add(lifted)
                            pick_ok += 1
                            picks_arm[a.k] += 1
                            if lifted in self._arrivals and "t_lift" not in self._arrivals[lifted]:
                                self._arrivals[lifted]["t_lift"] = round(sim.time - t_start, 3)
                            sim.carrying.add(lifted)
                        held = world.holding(a.k, i)               # quyết định của bộ điều khiển
                        if held != (lifted is not None):
                            lift_disagree += 1
                        if held:
                            a.holding = True
                            a.truth_target = lifted
                            status[i] = "held"
                            world.set_done(i)
                            log(a.k, "LIFT_CHECK", f"vật {i} (khối thật {lifted})")
                        else:
                            attempts[i] += 1
                            status[i] = "on_table" if attempts[i] < MAX_ATTEMPTS else "failed"
                            world.set_busy(i, False)
                            if a.truth_target is not None:
                                sim.allowed.pop(a.truth_target, None)
                            a.target = a.truth_target = None
                            log(a.k, "ABORT", f"vật {i} không lên, lần {attempts[i]}")
                    else:
                        status[i] = "placed"
                        if a.truth_target is not None:
                            sim.allowed.pop(a.truth_target, None)
                        a.holding, a.target, a.truth_target = False, None, None
                        t_last = sim.time
                        log(a.k, "PLACED", f"khối {i}")
                    a.state, a.action, a.move, a.goal = "IDLE", None, None, None
            # 2. tay chờ đi xuống
            for a in arms:
                if a.state != "WAIT_LOCAL":
                    continue
                S = a.sweep()
                others = {b.k: (self.fixed_set(b) if b.state != "TRANSIT" else [b.q]) for b in arms if b is not a}
                if not self._set_free_vs(a.k, S, others):
                    continue
                if path and len(path) > 1 and not self._path_free_vs(path, moving, a.k, S):
                    if a.sweep_ok or now < next_plan_t:
                        continue  # đã lập kế hoạch với tập quét nhưng chưa xong đoạn cũ, hoặc đang chờ sau thất bại
                    a.sweep_ok = True
                    if not replan(f"tay {a.k} cần vùng dành riêng"):
                        a.sweep_ok = False
                    continue
                wait_local += sim.time - a.arrived_t
                if a.task is not None and a.action == "pick":
                    a.task["wait_local"] = sim.time - a.arrived_t
                    tasks.append(a.task)
                    a.task = None
                self._start_local(sim, a, log)
            # 3. giao việc cho tay rảnh
            for a in arms:
                if a.state != "IDLE" or now < a.cooldown_until:
                    continue
                if self._assign(sim, a, arms, status, log):
                    a.state = "TRANSIT"
                    need_plan = True
            # kẹt: mọi tay rảnh, không có đường đi, còn vật trên bàn mà không tay nào gắp được (bị khối hàng xóm vây)
            idle_all = all(a.state == "IDLE" and not a.holding for a in arms) and not path and not need_plan
            if idle_all and any(s == "on_table" for s in status.values()) and not pending:
                stuck_since = now if stuck_since is None else stuck_since
                if now - stuck_since > STUCK_T:
                    for oid, s_ in status.items():
                        if s_ == "on_table":
                            status[oid] = "blocked"
                            log(-1, "BLOCKED", f"vật {oid}: không tay nào gắp được")
            else:
                stuck_since = None
            # kết thúc?
            if (not pending and all(s in ("placed", "failed", "lost", "blocked") for s in status.values())
                    and all(a.state == "IDLE" and not a.holding for a in arms)):
                break
            # 4. lập kế hoạch
            if need_plan and now >= next_plan_t:
                need_plan = False
                ok = replan("giao việc")
                if not ok:
                    # huỷ các việc chưa đi được bước nào; tay khác giữ nguyên chỗ
                    for a in arms:
                        if a.state == "TRANSIT" and not a.holding and a.action == "pick":
                            status[a.target] = "on_table"
                            world.set_busy(a.target, False)
                            if a.truth_target is not None:
                                sim.allowed.pop(a.truth_target, None)
                            a.target = a.truth_target = None
                            a.task = None
                        if a.state == "TRANSIT" and a.action == "pick":
                            a.state, a.action, a.move, a.goal = "IDLE", None, None, None
                            a.cooldown_until = sim.time + COOLDOWN
                        elif a.state == "TRANSIT":  # đang mang vật hoặc đang về HOME: giữ đích, thử lại sau
                            a.cooldown_until = sim.time + COOLDOWN
                    if any(a.state == "TRANSIT" for a in arms):
                        need_plan = True
            # 5. chạy
            t_before = sim.time
            was_busy = [a.state != "IDLE" or a.holding for a in arms]
            if path and len(path) > 1:
                q0, q1, T = path[0], path[1], float(seg_times[0])
                max_err = max(max_err, self._run_edge(sim, arms, moving, q0, q1, T))
                path, seg_times = path[1:], seg_times[1:]
                for k in moving:
                    arms[k].q = q1[6 * k:6 * k + 6].copy()
                if len(path) == 1:
                    path, moving = [], ()
                # tới đích
                for a in arms:
                    if a.state == "TRANSIT" and np.allclose(a.q, a.goal, atol=1e-9):
                        if a.action in ("pick", "place"):
                            a.state, a.sweep_ok, a.arrived_t = "WAIT_LOCAL", False, sim.time
                            log(a.k, "ARRIVED", f"{a.action} khối {a.target}")
                        else:
                            a.state, a.action, a.goal = "IDLE", None, None
                            log(a.k, "HOME")
            else:
                if any(a.state == "TRANSIT" for a in arms) and not need_plan:
                    need_plan = True  # đứng yên mà còn tay chưa tới đích: lập kế hoạch lại
                self._run_edge(sim, arms, (), None, None, IDLE_DT)
            busy += np.array(was_busy) * (sim.time - t_before)
        sim.wait(0.5)
        from .arm_fsm import Cell
        correct, wrong, out = Cell._verify(self, sim, cubes)
        return StaggeredResult(seed, self.alloc, len(cubes), correct, wrong, out, picks, pick_ok, sim.time - t_start,
                               t_last - t_start, planning_wall, len(replan_times), replan_times, plan_failures,
                               dict(sim.coll.steps), {"|".join(k): v for k, v in sim.coll.pairs.most_common(10)},
                               max_err, deadlock, events, [status[k] for k in sorted(status)], wait_local, tasks,
                               self._perception_summary(), lift_disagree,
                               sum(s_ == "blocked" for s_ in status.values()), disturbed,
                               [self._arrivals[k] for k in sorted(self._arrivals)], busy.round(3).tolist(), picks_arm)

    # ------------------------------------------------------------------ giao việc
    def _assign(self, sim: CellSim, a: Arm, arms: list[Arm], status: dict[int, str], log) -> bool:
        others = {b.k: self.goal_set(b) for b in arms if b is not a}

        def accept(mv: LocalMove) -> bool:
            return self._set_free_vs(a.k, a_sweep(mv), others)

        if a.holding:
            got = self.place_move(sim, a)
            if got is None:
                return False
            mv, j, slot = got
            if not accept(mv):
                return False
            a.action, a.move, a.goal = "place", mv, mv.q_top
            a.slots[j] = slot + 1
            a.where = j
            log(a.k, "TRANSIT_TO_PLACE", f"khối {a.target} → khay {j}")
            return True
        for i in self._ranked(sim, a, arms, status):
            mv = self.pick_move(sim, a.k, i)
            if mv is None or not accept(mv):
                continue
            a.action, a.move, a.goal, a.target = "pick", mv, mv.q_top, i
            status[i] = "assigned"
            self.world.set_busy(i, True)
            a.truth_target = self.world.truth(i)
            if a.truth_target is not None:
                sim.allowed[a.truth_target] = a.k
                self._handled.add(a.truth_target)
                rec = self._arrivals.get(a.truth_target)
                if rec is not None and "t_assign" not in rec:
                    rec["t_assign"] = round(sim.time - self._t_start, 3)
            a.task = self._task_record(sim, a, arms, i)
            log(a.k, "TRANSIT_TO_PICK", f"khối {i}")
            return True
        if not np.allclose(a.q, HOME) and not any(s == "on_table" for s in status.values()):
            if self._set_free_vs(a.k, [HOME], others):
                a.action, a.move, a.goal, a.target = "home", None, HOME.copy(), None
                log(a.k, "TRANSIT_HOME")
                return True
        return False

    def _task_record(self, sim: CellSim, a: Arm, arms: list[Arm], i: int) -> dict:
        """Hitbox (W mặc định) của việc (a, i) với việc gắp đang làm của các tay khác, như mức 1 dùng để phạt."""
        xy = self.world.pose(i)[0][:2]
        mine = Ellipse(self.L.base(a.k)[:2], xy)
        hits = []
        for b in arms:
            if b is a or b.target is None or b.action != "pick":
                continue
            oxy = self.world.pose(b.target)[0][:2]
            e = Ellipse(self.L.base(b.k)[:2], oxy)
            if mine.intersects(e) or e.contains(xy[None]).any() or mine.contains(oxy[None]).any():
                hits.append(b.k)
        active = [b.k for b in arms if b is not a and b.target is not None and b.action == "pick"]
        rec = {"arm": a.k, "cube": i, "t_assign": sim.time, "active": active, "hitbox": hits,
               "path_waits": 0, "plans": 0, "wait_local": 0.0}
        if self.world.perceived:  # đánh giá: sai số của pose mà quyết định gắp dùng
            j = self.world.truth(i)
            if j is not None:
                (p, yaw), (pt, yt) = self.world.pose(i), sim.cube_pose(j)
                rec.update(truth=j, err_xy=float(np.linalg.norm(p[:2] - pt[:2])), err_yaw=abs(yaw_diff(yaw, yt)),
                           color_ok=self.world.color(i) == self.cubes[j].color)
        return rec

    def _ranked(self, sim: CellSim, a: Arm, arms: list[Arm], status: dict[int, str]) -> list[int]:
        live = [i for i, s in status.items() if s == "on_table"]
        if self.alloc == "level0":
            p = self.tools[a.k].kin.fk(a.q)[0]
            mine = [i for i in a.queue if i in live]
            return sorted(mine, key=lambda i: np.linalg.norm(sim.cube_pose(i)[0][:2] - p[:2]))
        cands = [(i, self.world.pose(i)[0][:2], self.world.color(i)) for i in live]
        active = [(b.k, self.world.pose(b.target)[0][:2]) for b in arms
                  if b is not a and b.target is not None and b.action == "pick"]
        kw = {} if self.penalty is None else {"penalty": self.penalty}
        ranked = rank_level1(self.L, a.k, a.where, cands, self.tables[a.k], active, **kw)
        return [i for _, i in ranked]  # mọi ứng viên: khối bị vây thì pick_move trả None, xét khối kế tiếp

    def _park_idle(self, arms: list[Arm], log) -> None:
        """Thất bại liên tiếp: có thể một tay đang rảnh đứng chắn đường tay khác. Cho các tay rảnh không ở HOME về HOME
        (vào Ĝ cùng lúc, nên dRRT* phối hợp được với tay bị chặn)."""
        for a in arms:
            if a.state != "IDLE" or a.holding or np.allclose(a.q, HOME):
                continue
            others = {b.k: self.goal_set(b) for b in arms if b is not a}
            if self._set_free_vs(a.k, [HOME], others):
                a.action, a.move, a.goal, a.target = "home", None, HOME.copy(), None
                a.state = "TRANSIT"
                log(a.k, "PARK", "về HOME để dọn đường")

    def _spawn_spot(self, sim: CellSim, arms: list[Arm], arr, rng) -> tuple[float, float, float] | None:
        """Chỗ thả khối mới: chỗ mong muốn của cảnh, không được thì chỗ ngẫu nhiên trong vùng đặt vật. Điều kiện: tâm cách
        mọi khối trên bàn ≥ MIN_DIST, cách khối mà một tay đang nhắm tới ≥ 0,12 m, không tay nào tới gần hơn 3 cm."""
        L = self.L
        z0 = L.table_top + CUBE / 2
        lim = L.zone_half - CUBE / 2 * math.sqrt(2)
        on_table = [sim.cube_pose(j)[0][:2] for j in range(len(sim.cubes)) if abs(sim.cube_pose(j)[0][2] - z0) < 0.02]
        targets = [sim.cube_pose(a.truth_target)[0][:2] for a in arms if a.truth_target is not None]
        cands = [(arr.x, arr.y)] + [tuple(rng.uniform(-lim, lim, 2)) for _ in range(300)]
        for x, y in cands:
            p = np.array([x, y])
            if any(np.linalg.norm(p - q) < MIN_DIST for q in on_table):
                continue
            if any(np.linalg.norm(p - q) < 0.12 for q in targets):
                continue
            if sim.spot_free(x, y, arr.yaw, arr.cube):
                return float(x), float(y), float(arr.yaw)
        return None

    def _perception_summary(self) -> dict | None:
        w = self.world
        if w is None or not w.perceived:
            return None
        return {"frames": w.frames, "tracks": [{"id": t.id, "color": t.color, "state": t.state, "t_first": t.t_first,
                                                "n_full": t.n_full, "n_partial": t.n_partial} for t in w.tracker.tracks]}

    def _start_local(self, sim: CellSim, a: Arm, log) -> None:
        down, up = LineSeg(a.move.down, T_DOWN if a.action == "pick" else T_PLACE_DOWN), \
            LineSeg(a.move.down[::-1], T_UP if a.action == "pick" else T_PLACE_UP)
        if a.action == "pick":
            steps = [Step(down, GRIP_OPEN), Step(None, GRIP_CLOSE, T_CLOSE), Step(up, GRIP_CLOSE)]
            z0 = self.L.table_top + CUBE / 2
            a.snapshot = {j: sim.cube_pose(j)[0] for j in range(len(sim.cubes))
                          if j != a.truth_target and abs(sim.cube_pose(j)[0][2] - z0) < 0.01}
            log(a.k, "GRASP", f"khối {a.target}")
        else:
            steps = [Step(down, GRIP_CLOSE), Step(None, GRIP_OPEN, T_OPEN), Step(up, GRIP_OPEN)]
            if a.truth_target is not None:
                sim.carrying.discard(a.truth_target)  # từ đây khối được hạ vào khay: chạm khay là bình thường
            log(a.k, "PLACE", f"khối {a.target}")
        a.runner = Runner(sim.time, steps)
        a.state = "LOCAL"

    def _run_edge(self, sim: CellSim, arms: list[Arm], moving, q0, q1, T: float) -> float:
        """Chạy T giây: tay trong `moving` đi từ q0 tới q1 (đa thức bậc năm, feedforward vận tốc), tay LOCAL theo kịch
        bản, tay khác giữ nguyên. Trả sai số bám lớn nhất của các tay di chuyển."""
        t0 = sim.time
        err = 0.0
        n = max(1, int(round(T / sim.m.opt.timestep)))
        for step in range(n):
            t = sim.time
            if moving:
                s = (t - t0) / T
                sp = s * s * s * (10 - 15 * s + 6 * s * s)
                v = 30 * s * s * (1 - s) ** 2 / T
                for k in moving:
                    sl = slice(6 * k, 6 * k + 6)
                    sim.d.ctrl[sim.act[k]] = q0[sl] + (q1[sl] - q0[sl]) * sp + sim.lead * (q1[sl] - q0[sl]) * v
            for a in arms:
                if a.runner is not None:
                    a.runner.apply(sim, a.k, t)
                elif a.k not in moving:
                    sim.d.ctrl[sim.act[a.k]] = a.q
                    sim.d.ctrl[sim.grip[a.k]] = GRIP_CLOSE if a.holding else GRIP_OPEN
            sim.step()
            if self.world is not None and self.world.perceived:
                self.world.update()
            if moving and step % 10 == 0:
                s = min((sim.time - t0) / T, 1.0)
                sp = s * s * s * (10 - 15 * s + 6 * s * s)
                for k in moving:
                    sl = slice(6 * k, 6 * k + 6)
                    err = max(err, float(np.max(np.abs(sim.q(k) - (q0[sl] + (q1[sl] - q0[sl]) * sp)))))
        if moving:
            for k in moving:
                sim.d.ctrl[sim.act[k]] = q1[6 * k:6 * k + 6]
        return err


def subdivide(path: list[np.ndarray], max_piece: float) -> list[np.ndarray]:
    """Chèn điểm trên đoạn thẳng để mỗi cạnh dịch mọi khớp không quá max_piece. Hình học không đổi nên tính tự do va chạm
    giữ nguyên. Với đa thức bậc năm bị giới hạn vận tốc (T = 1,875 L / VMAX khi L ≥ 0,82 rad) tổng thời gian gần như
    không đổi, còn sự kiện xảy ra giữa cạnh chỉ phải chờ tối đa một đoạn ngắn."""
    out = [path[0]]
    for a, b in zip(path, path[1:]):
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / max_piece)))
        out += [a + (b - a) * (i / n) for i in range(1, n + 1)]
    return out


def path_waits(path: list[np.ndarray], moving) -> dict[int, int]:
    """Số cạnh của đường đi hợp mà tay k (trong moving) đứng yên trong khi tay khác di chuyển, trước lần cuối nó còn di
    chuyển: dRRT* bắt tay k chờ."""
    out = {}
    for k in moving:
        sl = slice(6 * k, 6 * k + 6)
        d = [float(np.max(np.abs(b[sl] - a[sl]))) for a, b in zip(path, path[1:])]
        dother = [float(np.max(np.abs(b - a))) for a, b in zip(path, path[1:])]
        last = max((j for j, x in enumerate(d) if x > 1e-9), default=-1)
        out[k] = sum(1 for j in range(last) if d[j] <= 1e-9 and dother[j] > 1e-9)
    return out


def a_sweep(mv: LocalMove) -> list[np.ndarray]:
    d = mv.down
    return list(d[::2]) + ([d[-1]] if (len(d) - 1) % 2 else [])


def result_dict(r: StaggeredResult) -> dict:
    return asdict(r)
