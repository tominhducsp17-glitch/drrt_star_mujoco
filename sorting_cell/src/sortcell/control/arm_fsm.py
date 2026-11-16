"""Điều phối cell M1: máy trạng thái từng tay, lập kế hoạch theo lượt, gắp và thả bằng script, pose thật từ sim.

Một lượt:
1. Mỗi tay chọn đích: đang mang khối → tư thế tiếp cận chỗ thả trong khay đúng màu; còn khối trong hàng đợi → tư thế
   chuẩn bị gắp khối gần TCP nhất (pose thật đọc từ sim lúc này); hết việc → HOME.
2. Chọn tập đích không va chạm giữa các tay (tay ưu tiên chọn trước, tay sau thử khối khác trong hàng đợi hoặc đứng yên).
3. Một truy vấn dRRT* cho cả 4 tay; chạy bằng mj_step, mọi tay cùng xuất phát, chờ nhau tới đích.
4. Pha cục bộ (GRASP + LIFT_CHECK, hoặc PLACE): các tay có quét cục bộ không va nhau và không va tay đang đứng yên chạy
   cùng lúc; tay xung đột chạy ở đợt sau; tay xung đột với một tay đứng yên thì hoãn: lượt sau lùi về HOME.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from drrt.execute import time_parameterize

from ..allocation.zones import assign_level0
from ..planning.service import PlannerService
from ..scene.builder import COLOR_NAMES, CUBE, Layout
from ..scene.sampling import CubeSpec
from ..skills.ik import HOME
from ..skills.scripted import ArmTools, LocalMove
from .executor import GRIP_CLOSE, GRIP_OPEN, CellSim, LineSeg, Step

VMAX, AMAX = 1.0, 2.0           # rad/s, rad/s² cho transit (như drrt.execute)
T_DOWN, T_UP = 1.2, 1.2         # s, đoạn đi xuống / lên 16 cm khi gắp
T_CLOSE, T_OPEN = 1.0, 0.5
T_PLACE_DOWN, T_PLACE_UP = 1.2, 1.0
LIFT_OK = 0.05
MAX_ATTEMPTS = 3
MAX_ROUNDS = 60


class Phase(str, Enum):
    IDLE = "IDLE"
    TRANSIT_TO_PICK = "TRANSIT_TO_PICK"
    GRASP = "GRASP"
    LIFT_CHECK = "LIFT_CHECK"
    TRANSIT_TO_PLACE = "TRANSIT_TO_PLACE"
    PLACE = "PLACE"
    ABORT = "ABORT"


@dataclass
class ArmState:
    k: int
    q: np.ndarray
    phase: Phase = Phase.IDLE
    queue: list[int] = field(default_factory=list)
    target: int | None = None          # khối đang nhắm tới hoặc đang mang
    holding: bool = False
    move: LocalMove | None = None      # chuyển động cục bộ ở đích của lượt này
    action: str | None = None          # "pick" | "place" | "home" | None
    deferred: bool = False
    slots: dict[int, int] = field(default_factory=dict)  # khay j → số khối đã thả


@dataclass
class CubeState:
    status: str = "on_table"           # on_table | assigned | held | placed | failed
    attempts: int = 0
    arm: int | None = None


@dataclass
class Event:
    t: float
    arm: int
    phase: str
    info: str = ""


@dataclass
class SceneResult:
    seed: int
    n_cubes: int
    placed_correct: int
    placed_wrong: int
    not_in_tray: int
    picks: int
    pick_success: int
    rounds: int
    sim_time: float
    t_last_place: float
    planning_wall: float
    plan_failures: int
    deferrals: int
    collision_steps: dict
    collision_pairs: dict
    max_tracking_error: float
    deadlock: bool
    events: list = field(default_factory=list)
    cube_status: list = field(default_factory=list)
    plan_times: list = field(default_factory=list)
    track_worst: tuple = ()

    @property
    def collisions(self) -> int:
        return sum(self.collision_steps.values())


class Cell:
    def __init__(self, L: Layout, planner: PlannerService, tools: list[ArmTools]):
        self.L, self.planner, self.tools = L, planner, tools
        self._place_cache: dict[tuple[int, int, int], LocalMove | None] = {}

    # ------------------------------------------------------------------ đích
    def place_moves(self, k: int, j: int, slot: int) -> list[LocalMove]:
        key = (k, j, slot % 5)
        if key not in self._place_cache:
            self._place_cache[key] = list(self.tools[k].place_options(self.L.tray_slots(k, j)[slot % 5]))
        return self._place_cache[key]

    def place_move(self, sim: CellSim, k: int, target: int, j: int, slot: int) -> LocalMove | None:
        """Cách thả đầu tiên mà đoạn đi xuống không quệt khối nào đã nằm trong khay."""
        for mv in self.place_moves(k, j, slot):
            if not sim.descent_touches(k, mv.down, target):
                return mv
        return None

    def pick_move(self, sim: CellSim, k: int, i: int) -> LocalMove | None:
        """Cách gắp đầu tiên (theo thứ tự hướng kẹp) mà đoạn đi xuống không quệt khối bên cạnh."""
        pos, yaw = sim.cube_pose(i)
        for mv in self.tools[k].pick_options(pos[:2], yaw):
            if not sim.descent_touches(k, mv.down, i):
                return mv
        return None

    def tcp(self, a: ArmState) -> np.ndarray:
        return self.tools[a.k].kin.fk(a.q)[0]

    def candidates(self, a: ArmState, sim: CellSim, cs: list[CubeState]) -> list[tuple[str, int | None, LocalMove | None, np.ndarray]]:
        """Các đích có thể của tay a, theo thứ tự ưu tiên: (hành động, khối, chuyển động cục bộ, q đích)."""
        if a.holding:
            j = COLOR_NAMES.index(self.cubes[a.target].color)
            for n in range(5):  # chỗ thả kế tiếp; nếu bị khối cũ chắn thì thử chỗ sau
                mv = self.place_move(sim, a.k, a.target, j, a.slots.get(j, 0) + n)
                if mv is not None:
                    a.slots[j] = a.slots.get(j, 0) + n
                    return [("place", a.target, mv, mv.q_top)]
            return []
        if a.deferred:
            return [("home", None, None, HOME)]
        out = []
        p = self.tcp(a)
        live = [i for i in a.queue if cs[i].status == "on_table"]
        live.sort(key=lambda i: np.linalg.norm(sim.cube_pose(i)[0][:2] - p[:2]))
        for i in live[:4]:
            mv = self.pick_move(sim, a.k, i)
            if mv is not None:
                out.append(("pick", i, mv, mv.q_top))
        if not out and not np.allclose(a.q, HOME):
            out.append(("home", None, None, HOME))
        return out

    # ------------------------------------------------------------------ một cảnh
    def run(self, cubes: list[CubeSpec], seed: int = 0, recorder=None, verbose: bool = False) -> SceneResult:
        L = self.L
        self.cubes = cubes
        sim = CellSim(L, cubes, recorder)
        sim.wait(0.3)  # khối nằm yên
        rng = np.random.default_rng(seed)
        cs = [CubeState() for _ in cubes]
        queues, orphans = assign_level0(L, cubes, lambda k, i: self.tools[k].pick((cubes[i].x, cubes[i].y),
                                                                                   cubes[i].yaw) is not None)
        for i in orphans:
            cs[i].status = "failed"
        arms = [ArmState(k, HOME.copy(), queue=queues[k]) for k in range(L.n_arms)]
        events: list[Event] = []
        log = lambda a, ph, info="": events.append(Event(round(sim.time, 3), a, ph, info))  # noqa: E731
        t_start = sim.time
        t_last_place = t_start
        planning_wall, plan_failures, deferrals, picks, pick_ok, max_err = 0.0, 0, 0, 0, 0, 0.0
        plan_times: list[float] = []
        deadlock = False
        rounds = 0
        while rounds < MAX_ROUNDS:
            busy = [a for a in arms if a.holding or any(cs[i].status == "on_table" for i in a.queue)]
            if not busy and all(np.allclose(a.q, HOME) for a in arms):
                break
            rounds += 1
            # 1–2. đích không va chạm giữa các tay
            order = sorted(arms, key=lambda a: (not a.holding, (a.k + rounds) % L.n_arms))
            goal_q = [a.q.copy() for a in arms]
            for a in arms:
                a.action, a.move = None, None
            for a in order:
                for act, i, mv, qg in self.candidates(a, sim, cs):
                    trial = list(goal_q)
                    trial[a.k] = qg
                    if self.planner.composite_free(trial):
                        goal_q[a.k] = qg
                        a.action, a.move = act, mv
                        if act == "pick":
                            a.target = i
                        break
            moving = [a for a in arms if a.action is not None]
            if not moving:
                deadlock = True
                log(-1, "DEADLOCK", "không tay nào có đích hợp lệ")
                break
            # 3. lập kế hoạch và chạy
            while True:
                goals = {a.k: goal_q[a.k] for a in arms if a.action is not None}
                res = self.planner.plan([a.q for a in arms], goals, {}, rng)
                planning_wall += res.planning_time
                plan_times.append(res.planning_time)
                if res.path is not None:
                    break
                plan_failures += 1
                log(-1, "PLAN_FAIL", res.reason)
                # bỏ đích của tay ưu tiên thấp nhất đang di chuyển rồi thử lại
                drop = [a for a in reversed(order) if a.action is not None and not np.allclose(goal_q[a.k], a.q)]
                if not drop:
                    break
                d0 = drop[0]
                goal_q[d0.k] = d0.q.copy()
                d0.action, d0.move = None, None
            if res.path is None or not any(a.action for a in arms):
                deadlock = True
                log(-1, "DEADLOCK", res.reason)
                break
            for a in arms:
                if a.action is not None and not np.allclose(goal_q[a.k], a.q):
                    a.phase = {"pick": Phase.TRANSIT_TO_PICK, "place": Phase.TRANSIT_TO_PLACE,
                               "home": Phase.IDLE}[a.action]
                    log(a.k, a.phase.value, f"khối {a.target}" if a.action != "home" else "về HOME")
                    if a.action == "pick":
                        cs[a.target].status, cs[a.target].arm = "assigned", a.k
            w0 = time.perf_counter()
            if len(res.path) > 1:
                traj = time_parameterize(res.path, VMAX, AMAX)
                max_err = max(max_err, sim.track(traj))
            if verbose:
                print(f"lượt {rounds}: " + ", ".join(f"tay {a.k} {a.action}" for a in arms if a.action) +
                      f" | lập kế hoạch {res.planning_time:.2f} s, {len(res.path)} đỉnh, "
                      f"mô phỏng {sim.time:.1f} s, chạy {time.perf_counter() - w0:.1f} s wall, "
                      f"va chạm {dict(sim.coll.steps)}", flush=True)
            for a in arms:
                a.q = goal_q[a.k].copy()
                if a.action == "home":
                    a.deferred = False
            # 4. pha cục bộ
            local = [a for a in arms if a.action in ("pick", "place")]
            remaining = list(local)
            while remaining:
                batch: list[ArmState] = []
                for a in remaining:
                    others_static = [b for b in arms if b is not a and b not in batch]
                    if self._sweep_ok(a, batch, others_static):
                        batch.append(a)
                if not batch:
                    for a in remaining:
                        a.deferred = True
                        deferrals += 1
                        if a.action == "pick":
                            cs[a.target].status = "on_table"
                        log(a.k, "DEFERRED", f"quét cục bộ va tay đang đứng yên; khối {a.target}")
                    break
                scripts = {}
                for a in batch:
                    seg_down = LineSeg(a.move.down, T_DOWN if a.action == "pick" else T_PLACE_DOWN)
                    seg_up = LineSeg(a.move.down[::-1], T_UP if a.action == "pick" else T_PLACE_UP)
                    if a.action == "pick":
                        sim.allowed[a.target] = a.k
                        a.phase = Phase.GRASP
                        log(a.k, "GRASP", f"khối {a.target}")
                        scripts[a.k] = [Step(seg_down, GRIP_OPEN), Step(None, GRIP_CLOSE, T_CLOSE),
                                        Step(seg_up, GRIP_CLOSE)]
                    else:
                        a.phase = Phase.PLACE
                        sim.carrying.discard(a.target)
                        log(a.k, "PLACE", f"khối {a.target}")
                        scripts[a.k] = [Step(seg_down, GRIP_CLOSE), Step(None, GRIP_OPEN, T_OPEN),
                                        Step(seg_up, GRIP_OPEN)]
                sim.run_scripts(scripts)
                for a in batch:
                    i = a.target
                    if a.action == "pick":
                        picks += 1
                        a.phase = Phase.LIFT_CHECK
                        rise = sim.cube_pose(i)[0][2] - (L.table_top + CUBE / 2)
                        if rise >= LIFT_OK:
                            pick_ok += 1
                            a.holding = True
                            cs[i].status = "held"
                            sim.carrying.add(i)
                            log(a.k, "LIFT_CHECK", f"khối {i} lên {rise*100:.1f} cm")
                        else:
                            cs[i].attempts += 1
                            cs[i].status = "on_table" if cs[i].attempts < MAX_ATTEMPTS else "failed"
                            sim.allowed.pop(i, None)
                            a.phase = Phase.ABORT
                            log(a.k, "ABORT", f"khối {i} không lên ({rise*100:.1f} cm), lần {cs[i].attempts}")
                    else:
                        j = COLOR_NAMES.index(cubes[i].color)
                        a.slots[j] = a.slots.get(j, 0) + 1
                        a.holding = False
                        cs[i].status = "placed"
                        sim.allowed.pop(i, None)
                        t_last_place = sim.time
                        a.phase = Phase.IDLE
                        a.target = None
                        log(a.k, "IDLE", f"thả khối {i}")
                remaining = [a for a in remaining if a not in batch]
            # khối do tay khác làm xê dịch vẫn on_table; khối rơi khỏi bàn thì bỏ
            for i, c in enumerate(cs):
                if c.status == "on_table" and sim.cube_pose(i)[0][2] < L.table_top - 0.05:
                    c.status = "failed"
                    log(-1, "CUBE_LOST", f"khối {i} rơi khỏi bàn")
        else:
            deadlock = True
        sim.wait(0.5)
        correct, wrong, out = self._verify(sim, cubes)
        return SceneResult(seed, len(cubes), correct, wrong, out, picks, pick_ok, rounds, sim.time - t_start,
                           t_last_place - t_start, planning_wall, plan_failures, deferrals, dict(sim.coll.steps),
                           {"|".join(k): v for k, v in sim.coll.pairs.most_common(10)}, max_err, deadlock,
                           [e.__dict__ for e in events], [c.status for c in cs], plan_times, sim.track_worst)

    def _sweep_ok(self, a: ArmState, batch: list[ArmState], static: list[ArmState]) -> bool:
        """Quét cục bộ của a không va (giữa các tay, lề an toàn của planner) với quét của các tay trong batch và với
        cấu hình đứng yên của các tay còn lại."""
        n = self.L.n_arms
        sweep = a.move.down[::2]
        for qa in sweep:
            base = [b.q for b in static]
            idx = [b.k for b in static]
            for combo in self._combos(batch):
                qs = [None] * n
                qs[a.k] = qa
                for k, q in zip(idx, base):
                    qs[k] = q
                for k, q in combo:
                    qs[k] = q
                if not self.planner.composite_free(qs):
                    return False
        return True

    @staticmethod
    def _combos(batch: list[ArmState]):
        """Mọi tổ hợp điểm quét của các tay trong batch (thưa, mỗi tay 6 điểm)."""
        if not batch:
            yield []
            return
        head, rest = batch[0], batch[1:]
        for q in head.move.down[::2]:
            for tail in Cell._combos(rest):
                yield [(head.k, q)] + tail

    def _verify(self, sim: CellSim, cubes: list[CubeSpec]) -> tuple[int, int, int]:
        L = self.L
        correct = wrong = out = 0
        for i, c in enumerate(cubes):
            p, _ = sim.cube_pose(i)
            hit = None
            for k in range(L.n_arms):
                yaw = L.base(k)[3]
                for j in range(3):
                    d = p - L.tray_center(k, j)
                    lx = np.cos(yaw) * d[0] + np.sin(yaw) * d[1]
                    ly = -np.sin(yaw) * d[0] + np.cos(yaw) * d[1]
                    if abs(lx) < L.tray_half and abs(ly) < L.tray_half and -0.01 < d[2] < L.tray_wall + CUBE:
                        hit = j
            if hit is None:
                out += 1
            elif COLOR_NAMES[hit] == c.color:
                correct += 1
            else:
                wrong += 1
        return correct, wrong, out
