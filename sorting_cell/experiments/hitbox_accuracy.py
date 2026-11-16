"""Độ chính xác của hitbox elip (Oberthür et al.) khi dự đoán tranh chấp giữa hai việc gắp (mục 11).

Việc (k, i) = pha vươn tới vật của tay k: HOME → tư thế chuẩn bị gắp vật i (đường ngắn nhất trên roadmap của tay k, rút
gọn, chia đoạn, định thời như khi thực thi ở M2) → đi xuống → đóng → đi lên. Đây là phần mà hitbox mô hình hoá.

Tranh chấp thật của cặp việc (k, i), (k', i') với k ≠ k', i ≠ i': cho hai tay chạy độc lập theo quỹ đạo riêng, tay k'
bắt đầu trễ δ giây so với tay k; nếu ở một thời điểm nào đó hai tay va nhau (model online: chỉ xét hai tay này, lề
CLEARANCE như planner) thì dRRT* hoặc vùng dành riêng buộc phải bắt một tay chờ hoặc đi vòng. Hai mức:
- cùng lúc: δ = 0 (hai việc được giao cùng lúc, ví dụ đầu cảnh);
- lệch giờ bất kỳ: có ít nhất một δ trên lưới DELTA_STEP trong khoảng hai việc còn chồng thời gian.
Chỉ xét phần hai việc chồng thời gian; ngoài khoảng của mình tay coi như không có mặt (trước đó nó ở xa hoặc vừa thả
xong, sau đó nó đi về khay).

Dự đoán: elip(đế k, vật i) và elip(đế k', vật i') giao nhau, hoặc elip này chứa vật kia (đúng như allocation.milp).

    MUJOCO_GL=egl .venv/bin/python experiments/hitbox_accuracy.py --scenes 50
"""
from __future__ import annotations

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from drrt.execute import time_parameterize

from sortcell.allocation.hitbox import Ellipse
from sortcell.control.arm_fsm import AMAX, T_CLOSE, T_DOWN, T_UP, VMAX
from sortcell.control.executor import minjerk
from sortcell.control.staggered import MAX_PIECE, subdivide
from sortcell.planning.models import PairChecker
from sortcell.planning.service import PlannerService
from sortcell.scene.builder import Layout
from sortcell.scene.sampling import sample_scene
from sortcell.skills.ik import HOME
from sortcell.skills.scripted import ArmTools

DT = 0.05            # bước thời gian khi so hai quỹ đạo: ở VMAX = 1 rad/s mỗi khớp dịch ≤ 0,05 rad (= RESOLUTION)
DELTA_STEP = 0.25
WS = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30]

_ctx = None


class Ctx:
    def __init__(self):
        self.L = Layout()
        self.pl = PlannerService(self.L)
        self.tools = [ArmTools(self.L, k, self.pl.spaces[k]) for k in range(self.L.n_arms)]
        self.pair = PairChecker(self.L)
        self.dist, self.pred = [], []
        for k, rm in enumerate(self.pl.rms):
            assert np.allclose(rm.q[0], HOME)
            rows, cols, w = [], [], []
            for u, v in rm.edges:
                d = float(np.linalg.norm(rm.q[u] - rm.q[v]))
                rows += [u, v]
                cols += [v, u]
                w += [d, d]
            g = csr_matrix((w, (rows, cols)), shape=(rm.n, rm.n))
            dist, pred = dijkstra(g, directed=False, indices=0, return_predecessors=True)
            self.dist.append(dist)
            self.pred.append(pred)
        m = self.pair.model
        arm_of = np.full(m.ngeom, -1)
        for g in range(m.ngeom):
            if m.geom_contype[g]:
                arm_of[g] = int(m.geom_contype[g]).bit_length() - 1
        self.arm_geoms = [np.flatnonzero(arm_of == k) for k in range(self.L.n_arms)]
        self.orig = (m.geom_contype.copy(), m.geom_conaffinity.copy())

    # ------------------------------------------------------------------ quỹ đạo một việc
    def home_path(self, k: int, q_goal: np.ndarray) -> list[np.ndarray] | None:
        rm = self.pl.rms[k]
        links = rm.connect(q_goal)
        if not links:
            return None
        v, _ = min(links, key=lambda x: self.dist[k][x[0]] + x[1])
        if not np.isfinite(self.dist[k][v]):
            return None
        chain = [v]
        while chain[-1] != 0:
            chain.append(int(self.pred[k][chain[-1]]))
        path = [rm.q[u] for u in chain[::-1]] + [q_goal]
        sp = self.pl.spaces[k]
        out, i = [path[0]], 0
        while i < len(path) - 1:  # rút gọn như PlannerService._shortcut (một tay)
            j = len(path) - 1
            while j > i + 1 and not sp.edge_free(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    def reach(self, k: int, xy, yaw) -> dict | None:
        mv = self.tools[k].pick(xy, yaw)
        if mv is None:
            return None
        path = self.home_path(k, mv.q_top)
        if path is None:
            return None
        path = subdivide(path, MAX_PIECE)
        times = time_parameterize(path, VMAX, AMAX).times
        ts = np.arange(0.0, times[-1] + T_DOWN + T_CLOSE + T_UP + DT, DT)
        down = np.asarray(mv.down)
        qs = []
        for t in ts:
            if t < times[-1]:
                j = int(np.searchsorted(times, t, side="right") - 1)
                s = minjerk((t - times[j]) / (times[j + 1] - times[j]))
                qs.append(path[j] + (path[j + 1] - path[j]) * s)
                continue
            u = t - times[-1]
            n = len(down) - 1
            if u < T_DOWN:
                s = minjerk(u / T_DOWN) * n
            elif u < T_DOWN + T_CLOSE:
                s = n
            else:
                s = (1 - minjerk((u - T_DOWN - T_CLOSE) / T_UP)) * n
            i = min(int(s), n - 1)
            qs.append(down[i] + (down[i + 1] - down[i]) * (s - i))
        return {"qs": np.array(qs), "T": float(ts[-1]), "transit": float(times[-1])}

    # ------------------------------------------------------------------ va chạm giữa hai tay
    def mask(self, k: int, k2: int):
        ct, ca = np.zeros_like(self.orig[0]), np.zeros_like(self.orig[1])
        ct[self.arm_geoms[k]], ca[self.arm_geoms[k]] = 1 << k, 1 << k2
        ct[self.arm_geoms[k2]], ca[self.arm_geoms[k2]] = 1 << k2, 1 << k
        return ct, ca

    def collide(self, k, qa, k2, qb) -> bool:
        m, d = self.pair.model, self.pair.data
        d.qpos[self.pair.idx[6 * k:6 * k + 6]] = qa
        d.qpos[self.pair.idx[6 * k2:6 * k2 + 6]] = qb
        mj.mj_kinematics(m, d)
        mj.mj_collision(m, d)
        return d.ncon > 0

    def conflict_offsets(self, k, A, k2, B) -> list[float]:
        """Các độ lệch δ (tay k2 bắt đầu sau tay k δ giây) mà hai tay va nhau."""
        m = self.pair.model
        m.geom_contype[:], m.geom_conaffinity[:] = self.mask(k, k2)
        qa, qb = A["qs"], B["qs"]
        hits = []
        n_off = int(np.floor(B["T"] / DELTA_STEP))
        deltas = np.arange(-n_off + 1, int(np.ceil(A["T"] / DELTA_STEP))) * DELTA_STEP
        for delta in deltas:
            shift = int(round(delta / DT))
            for t in range(max(0, shift), min(len(qa), shift + len(qb))):  # chỉ phần hai việc chồng thời gian
                if self.collide(k, qa[t], k2, qb[t - shift]):
                    hits.append(float(delta))
                    break
        m.geom_contype[:], m.geom_conaffinity[:] = self.orig
        return hits


def _init():
    global _ctx
    _ctx = Ctx()


def run_scene(seed: int) -> dict:
    c = _ctx
    L = c.L
    cubes = sample_scene(seed)
    reach = {}
    for k in range(L.n_arms):
        for i, cb in enumerate(cubes):
            r = c.reach(k, (cb.x, cb.y), cb.yaw)
            if r is not None:
                reach[k, i] = r
    pairs = []
    for (k, i), A in reach.items():
        for (k2, i2), B in reach.items():
            if k2 <= k or i2 == i:
                continue
            hits = c.conflict_offsets(k, A, k2, B)
            xy, xy2 = np.array([cubes[i].x, cubes[i].y]), np.array([cubes[i2].x, cubes[i2].y])
            pred = {}
            for w in WS:
                e1, e2 = Ellipse(L.base(k)[:2], xy, w), Ellipse(L.base(k2)[:2], xy2, w)
                pred[w] = bool(e1.intersects(e2) or e1.contains(xy2[None]).any() or e2.contains(xy[None]).any())
            n_off = int(np.floor(B["T"] / DELTA_STEP)) - 1 + int(np.ceil(A["T"] / DELTA_STEP))
            pairs.append({"k": k, "i": i, "k2": k2, "i2": i2, "sim": 0.0 in hits, "any": bool(hits),
                          "frac": len(hits) / n_off, "pred": pred,
                          "d_cubes": float(np.linalg.norm(xy - xy2)),
                          "TA": A["T"], "TB": B["T"]})
    return {"seed": seed, "n_tasks": len(reach), "pairs": pairs}


def confusion(pairs, truth: str, w: float) -> dict:
    tp = sum(p["pred"][w] and p[truth] for p in pairs)
    fp = sum(p["pred"][w] and not p[truth] for p in pairs)
    fn = sum(not p["pred"][w] and p[truth] for p in pairs)
    tn = sum(not p["pred"][w] and not p[truth] for p in pairs)
    n = tp + fp + fn + tn
    return {"tp": tp, "fp": fp, "fn": fn, "tn": tn,
            "precision": tp / (tp + fp) if tp + fp else None, "recall": tp / (tp + fn) if tp + fn else None,
            "accuracy": (tp + tn) / n, "predicted_rate": (tp + fp) / n, "true_rate": (tp + fn) / n}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, default=50)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--out", default="results/allocation/hitbox_accuracy.json")
    a = ap.parse_args()
    seeds = list(range(1000, 1000 + a.scenes))
    t = time.perf_counter()
    _init()
    with ProcessPoolExecutor(a.workers, initializer=_init) as ex:
        scenes = list(ex.map(run_scene, seeds))
    pairs = [p for s in scenes for p in s["pairs"]]
    out = {"n_scenes": len(scenes), "n_tasks": sum(s["n_tasks"] for s in scenes), "n_pairs": len(pairs),
           "dt": DT, "delta_step": DELTA_STEP, "wall_s": time.perf_counter() - t,
           "by_w": {str(w): {tr: confusion(pairs, tr, w) for tr in ("sim", "any")} for w in WS},
           "mean_conflict_frac_given_any": float(np.mean([p["frac"] for p in pairs if p["any"]])),
           }
    # tranh chấp thật theo khoảng cách giữa hai vật (để thấy hitbox sai ở đâu)
    bins = [0, 0.1, 0.2, 0.3, 0.4, 0.6, 2.0]
    out["by_distance"] = []
    for lo, hi in zip(bins, bins[1:]):
        sel = [p for p in pairs if lo <= p["d_cubes"] < hi]
        if sel:
            out["by_distance"].append({"range": [lo, hi], "n": len(sel),
                                       "any": float(np.mean([p["any"] for p in sel])),
                                       "sim": float(np.mean([p["sim"] for p in sel])),
                                       "pred_0.15": float(np.mean([p["pred"][0.15] for p in sel]))})
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, indent=1))
    Path(a.out).with_suffix(".pairs.json").write_text(json.dumps(pairs))
    print(json.dumps({k: v for k, v in out.items() if k != "by_w"}, indent=1))
    for w in WS:
        for tr in ("sim", "any"):
            cf = out["by_w"][str(w)][tr]
            print(f"W={w:.2f} {tr:3s}: P={cf['precision']:.3f} R={cf['recall']:.3f} acc={cf['accuracy']:.3f} "
                  f"dự đoán {cf['predicted_rate']:.3f} thật {cf['true_rate']:.3f}")


if __name__ == "__main__":
    main()
