"""Kiểm tra tầm với của bố cục (nghiệm thu M0) và vẽ bản đồ số tay với tới.

Ô (x, y) đạt với tay k khi ArmTools.pick có nghiệm cho MỌI góc khối trong YAWS (mỗi góc có 4 hướng kẹp tương đương).
Khay đạt khi ArmTools.place của tay sở hữu có nghiệm cho cả 5 chỗ thả. Kết quả: results/reach/.
"""
from __future__ import annotations

import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from sortcell.scene.builder import CUBE, Layout
from sortcell.skills.scripted import ArmTools

YAWS = np.radians([0.0, 22.5, 45.0, 67.5])
OUT = Path(__file__).resolve().parents[1] / "results/reach"


def job(args):
    k, cells, d_h = args
    L = Layout()
    t = ArmTools(L, k)
    reach = [all(t.pick(c, y, d_h) is not None for y in YAWS) for c in cells]
    trays = [[t.place(s) is not None for s in L.tray_slots(k, j)] for j in range(3)]
    return k, reach, trays


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--d-h", type=float, default=0.16)
    ap.add_argument("--workers", type=int, default=16)
    a = ap.parse_args()
    L = Layout()
    lim = L.table_half - CUBE / 2
    xs = np.round(np.arange(-np.floor(lim / a.step), np.floor(lim / a.step) + 1) * a.step, 4)
    cells = [(float(x), float(y)) for x in xs for y in xs]
    parts = a.workers // 4
    jobs = [(k, cells[p::parts], a.d_h) for k in range(4) for p in range(parts)]
    where = [list(range(len(cells)))[p::parts] for k in range(4) for p in range(parts)]
    reach = np.zeros((4, len(cells)), bool)
    trays = {}
    with ProcessPoolExecutor(a.workers) as ex:
        for w, (k, r, t) in zip(where, ex.map(job, jobs)):
            reach[k, w] = r
            trays[k] = t
    count = reach.sum(0)
    xy = np.array(cells)
    in_zone = np.all(np.abs(xy) <= L.zone_half + 1e-9, axis=1)
    summary = {
        "layout": L.__dict__, "step": a.step, "d_h": a.d_h, "n_cells": len(cells),
        "table_cells_ge1": int((count >= 1).sum()), "table_cells_ge2": int((count >= 2).sum()),
        "zone_cells": int(in_zone.sum()), "zone_cells_ge2": int((count[in_zone] >= 2).sum()),
        "zone_min_arms": int(count[in_zone].min()), "table_min_arms": int(count.min()),
        "zone_hist": {str(c): int((count[in_zone] == c).sum()) for c in range(5)},
        "table_hist": {str(c): int((count == c).sum()) for c in range(5)},
        "trays_ok": {str(k): [all(s) for s in v] for k, v in trays.items()},
        "tray_slots_ok": int(sum(sum(sum(s) for s in v) for v in trays.values())),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    tag = f"dh{int(a.d_h*100)}"
    (OUT / f"summary_{tag}.json").write_text(json.dumps(summary, indent=1))
    np.savez_compressed(OUT / f"reach_{tag}.npz", cells=xy, reach=reach)
    print(json.dumps({k: v for k, v in summary.items() if k != "layout"}, indent=1))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(xs)
    grid = count.reshape(n, n).T  # cells theo x ngoài, y trong
    fig, ax = plt.subplots(figsize=(6, 5.4))
    im = ax.imshow(grid, origin="lower", extent=[xs[0] - a.step / 2, xs[-1] + a.step / 2] * 2, cmap="viridis",
                   vmin=0, vmax=4)
    ax.add_patch(plt.Rectangle((-L.zone_half, -L.zone_half), 2 * L.zone_half, 2 * L.zone_half, fill=False, ec="w",
                               lw=1.5, ls="--"))
    for k in range(4):
        x, y, _, _ = L.base(k)
        ax.annotate(f"tay {k}", (np.clip(x, -0.6, 0.6), np.clip(y, -0.62, 0.62)), color="w", ha="center")
    fig.colorbar(im, ax=ax, label="số tay với tới (IK không va chạm, mọi góc khối)")
    ax.set_title(f"Bàn {2*L.table_half:.2f} m, đế cách tâm {L.base_dist} m, d_h = {a.d_h*100:.0f} cm")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(OUT / f"coverage_{tag}.png", dpi=120)


if __name__ == "__main__":
    main()
