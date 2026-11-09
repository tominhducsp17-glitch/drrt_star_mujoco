"""Đọc kết quả P2, in bảng và vẽ hình tương ứng Hình 11 của bài."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ALGOS = ["dRRT*", "ao-dRRT", "dRRT", "RRT*", "Implicit A*"]
# Thứ tự màu cố định của bảng màu tham chiếu; A* dùng xám vì đây là baseline tối ưu, không phải một dòng thuật toán cây.
COLORS = {"dRRT*": "#2a78d6", "ao-dRRT": "#eb6834", "dRRT": "#1baf7a", "RRT*": "#eda100", "Implicit A*": "#52514e"}
MARKERS = {"dRRT*": "o", "ao-dRRT": "s", "dRRT": "^", "RRT*": "D", "Implicit A*": "X"}
TEXT = "#0b0b0b"
MUTED = "#52514e"
GRID = "#e6e5e0"


def load(path: Path) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def pick(rows, algo, R):
    return [r for r in rows if r["algo"] == algo and int(r["R"]) == R]


def ratio(rows, field):
    out = [float(r[field]) / float(r["lower_bound"]) for r in rows if r.get(field) and r.get("lower_bound")]
    return np.array(out)


def times(rows):
    return np.array([float(r["first_time"]) for r in rows if r.get("first_time")])


def fmt(a, digits=2):
    return f"{a.mean():.{digits}f}" if len(a) else "-"


def table(rows, robots) -> str:
    lines = ["| R | Thuật toán | Thành công | Nghiệm đầu (s) | Cost đầu / mốc | Cost cuối / mốc | Vòng TB | Hết RAM |",
             "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for R in robots:
        for algo in ALGOS:
            sel = pick(rows, algo, R)
            if not sel:
                continue
            solved = sum(int(r["solved"]) for r in sel)
            iters = [int(r["iterations"]) for r in sel if r.get("iterations")]
            oom = sum(1 for r in sel if r.get("oom"))
            lines.append(f"| {R} | {algo} | {solved}/{len(sel)} | {fmt(times(sel), 3)} | {fmt(ratio(sel, 'first_cost'))} "
                         f"| {fmt(ratio(sel, 'final_cost'))} | {np.mean(iters) if iters else 0:.0f} | {oom} |")
    return "\n".join(lines)


def plot(rows, robots, out: Path) -> None:
    plt.rcParams.update({"font.size": 10, "text.color": TEXT, "axes.labelcolor": MUTED, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.edgecolor": GRID, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.3), constrained_layout=True)
    for algo in ALGOS:
        style = dict(color=COLORS[algo], marker=MARKERS[algo], lw=2, ms=6, markeredgecolor="white",
                     markeredgewidth=1, label=algo, zorder=5 if algo == "dRRT*" else 2)
        rs = [R for R in robots if pick(rows, algo, R)]
        success = [np.mean([int(r["solved"]) for r in pick(rows, algo, R)]) for R in rs]
        axes[0].plot(rs, success, **style)
        with_cost = [R for R in rs if len(ratio(pick(rows, algo, R), "final_cost"))]
        axes[1].plot(with_cost, [ratio(pick(rows, algo, R), "final_cost").mean() for R in with_cost], **style)
        with_time = [R for R in rs if len(times(pick(rows, algo, R)))]
        axes[2].plot(with_time, [times(pick(rows, algo, R)).mean() for R in with_time], **style)
    axes[0].set_title("Tỉ lệ thành công", loc="left", fontsize=11)
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].legend(frameon=False, fontsize=9, loc="lower left")
    axes[1].set_title("Chi phí cuối / mốc lạc quan (chỉ các lần giải được)", loc="left", fontsize=11)
    axes[1].axhline(1.0, color=MUTED, lw=1, ls="--")
    axes[2].set_title("Thời gian nghiệm đầu (giây, thang log)", loc="left", fontsize=11)
    axes[2].set_yscale("log")
    for ax in axes:
        ax.set_xlabel("Số robot R")
        ax.set_xticks(robots)
        ax.grid(axis="y", color=GRID, lw=0.8)
        ax.set_axisbelow(True)
    fig.savefig(out, dpi=160)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", type=Path, default=Path("results/p2"))
    args = ap.parse_args()
    rows = load(args.dir / "summary.csv")
    robots = sorted({int(r["R"]) for r in rows})
    report = table(rows, robots)
    (args.dir / "tables.md").write_text(report + "\n")
    print(report)
    plot(rows, robots, args.dir / "fig11.png")


if __name__ == "__main__":
    main()
