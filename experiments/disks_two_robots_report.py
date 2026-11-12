"""Đọc kết quả thí nghiệm 2 đĩa, in Bảng 1 và vẽ hình tương ứng Hình 10 của bài."""
from __future__ import annotations

import argparse
import collections
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Bảng màu tham chiếu của skill dataviz; màu đi theo thuật toán, không đi theo thứ hạng.
COLORS = {"dRRT*": "#2a78d6", "ao-dRRT": "#eb6834", "Implicit A*": "#52514e"}
TEXT = "#0b0b0b"
MUTED = "#52514e"
GRID = "#e6e5e0"


def load(path: Path) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def values(rows, algo, n, field):
    out = [float(r[field]) for r in rows if r["algo"] == algo and int(r["n"]) == n and r.get(field) not in ("", None)]
    return np.array(out)


def fmt(a: np.ndarray, digits: int = 3) -> str:
    return f"{a.mean():.{digits}f} ± {a.std():.{digits}f}" if len(a) else "-"


def table(rows: list[dict], sizes: list[int]) -> str:
    lines = ["| Đại lượng (giây) | " + " | ".join(f"n={n}" for n in sizes) + " |",
             "| --- |" + " --- |" * len(sizes)]

    def row(label, algo, field, digits=3):
        lines.append(f"| {label} | " + " | ".join(fmt(values(rows, algo, n, field), digits) for n in sizes) + " |")

    row("N²-PRM\\* dựng", "N2-PRM*", "build_time", 2)
    row("N²-PRM\\* truy vấn", "N2-PRM*", "first_time")
    row("2 PRM\\* cỡ n dựng", "dRRT*", "build_time")
    row("Implicit A\\* (tối ưu)", "Implicit A*", "first_time")
    row("ao-dRRT nghiệm đầu", "ao-dRRT", "first_time")
    row("dRRT\\* nghiệm đầu", "dRRT*", "first_time", 4)
    return "\n".join(lines)


def cost_table(rows: list[dict], sizes: list[int]) -> str:
    lines = ["| n | Tối ưu A\\* | dRRT\\* đầu | dRRT\\* 100k vòng | ao-dRRT đầu | ao-dRRT 100k vòng | N²-PRM\\* | Số lần giải dRRT\\*/ao |",
             "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for n in sizes:
        solved = f"{len(values(rows, 'dRRT*', n, 'final_cost'))}/{len(values(rows, 'ao-dRRT', n, 'final_cost'))}"
        lines.append(f"| {n} | {fmt(values(rows, 'Implicit A*', n, 'final_cost'), 2)} "
                     f"| {fmt(values(rows, 'dRRT*', n, 'first_cost'), 2)} | {fmt(values(rows, 'dRRT*', n, 'final_cost'), 2)} "
                     f"| {fmt(values(rows, 'ao-dRRT', n, 'first_cost'), 2)} | {fmt(values(rows, 'ao-dRRT', n, 'final_cost'), 2)} "
                     f"| {fmt(values(rows, 'N2-PRM*', n, 'final_cost'), 2)} | {solved} |")
    return "\n".join(lines)


def gap_table(rows: list[dict], sizes: list[int]) -> str:
    """Khoảng cách tới tối ưu của chính cặp roadmap đó sau 100k vòng: đây là kiểm chứng của Theorem 2."""
    optimum = {(int(r["n"]), int(r["pair"])): float(r["final_cost"]) for r in rows
               if r["algo"] == "Implicit A*" and r["final_cost"]}
    lines = ["| n | dRRT\\*: chênh TB so với tối ưu | dRRT\\*: số lần đạt đúng tối ưu | ao-dRRT: chênh TB | ao-dRRT: số lần đạt đúng tối ưu |",
             "| --- | --- | --- | --- | --- |"]
    for n in sizes:
        cells = []
        for algo in ("dRRT*", "ao-dRRT"):
            gaps = [float(r["final_cost"]) / optimum[(n, int(r["pair"]))] - 1 for r in rows
                    if r["algo"] == algo and int(r["n"]) == n and r["final_cost"] and (n, int(r["pair"])) in optimum]
            gaps = np.array(gaps)
            cells += [f"{gaps.mean() * 100:.2f}%", f"{int((gaps < 1e-6).sum())}/{len(gaps)}"]
        lines.append(f"| {n} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def plot(rows: list[dict], conv: list[dict], sizes: list[int], out: Path) -> None:
    plt.rcParams.update({"font.size": 10, "text.color": TEXT, "axes.labelcolor": MUTED, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.edgecolor": GRID, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, axes = plt.subplots(2, len(sizes), figsize=(4 * len(sizes), 6.6), sharex="row", constrained_layout=True,
                             gridspec_kw={"height_ratios": [3, 2]})
    series = collections.defaultdict(list)
    for r in conv:
        if r["cost"]:
            series[(r["algo"], int(r["n"]), int(r["checkpoint"]))].append(float(r["cost"]))
    for col, n in enumerate(sizes):
        ax = axes[0, col]
        for algo in ("dRRT*", "ao-dRRT"):
            xs = sorted(k[2] for k in series if k[0] == algo and k[1] == n)
            ys = [np.mean(series[(algo, n, x)]) for x in xs]
            ax.plot(np.array(xs) / 1000, ys, color=COLORS[algo], lw=2, marker="o", ms=4,
                    markeredgecolor="white", markeredgewidth=1, label=algo)
        opt = values(rows, "Implicit A*", n, "final_cost")
        ax.axhline(opt.mean(), color=COLORS["Implicit A*"], lw=1.5, ls="--", label="Tối ưu trên Ĝ (Implicit A*)")
        ax.set_title(f"Roadmap n = {n}", loc="left", fontsize=11, color=TEXT)
        ax.set_xlabel("Số vòng lặp (nghìn)")
        ax.grid(axis="y", color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        if col == 0:
            ax.set_ylabel("Chi phí trung bình (tổng độ dài)")
            ax.legend(frameon=False, fontsize=9)

        ax = axes[1, col]
        labels = ["dRRT*", "ao-dRRT", "Implicit A*"]
        data = [values(rows, a, n, "first_time") for a in labels]
        means = [d.mean() * 1000 for d in data]
        bars = ax.bar(labels, means, color=[COLORS[a] for a in labels], width=0.6)
        ax.set_yscale("log")
        ax.bar_label(bars, labels=[f"{m:.3g} ms" for m in means], padding=2, fontsize=9, color=TEXT)
        ax.set_ylim(top=max(means) * 8)
        ax.grid(axis="y", color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        if col == 0:
            ax.set_ylabel("Thời gian nghiệm đầu (ms, thang log)")
    fig.savefig(out, dpi=160)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", type=Path, default=Path("results/disks_two_robots"))
    args = ap.parse_args()
    rows = load(args.dir / "summary.csv")
    conv = load(args.dir / "convergence.csv")
    sizes = sorted({int(r["n"]) for r in rows})
    report = "\n\n".join([table(rows, sizes), cost_table(rows, sizes), gap_table(rows, sizes)])
    (args.dir / "tables.md").write_text(report + "\n")
    print(report)
    plot(rows, conv, sizes, args.dir / "fig10.png")


if __name__ == "__main__":
    main()
