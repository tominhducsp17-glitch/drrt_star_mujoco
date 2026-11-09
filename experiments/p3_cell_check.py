"""P3: kiểm tra cell UR10e trước khi lập kế hoạch.

1. Phép tách va chạm (offline từng tay + online giữa các tay) cho cùng kết quả với model đầy đủ.
2. Tốc độ kiểm tra và tỉ lệ cấu hình tự do.
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from drrt.mujoco_cell import ArmSpace, CellPairChecker, FullChecker, four_arm_cell, two_arm_cell


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=20000)
    ap.add_argument("--arms", type=int, default=2, choices=[2, 4])
    args = ap.parse_args()
    cell = two_arm_cell() if args.arms == 2 else four_arm_cell()
    rng = np.random.default_rng(0)
    arms = [ArmSpace(cell, k) for k in range(cell.n_arms)]
    pair = CellPairChecker(cell)
    full = FullChecker(cell)

    q = rng.uniform(-cell.joint_limit, cell.joint_limit, (args.samples, 6 * cell.n_arms))
    t0 = time.perf_counter()
    per_arm = np.array([[arm.config_free(row[6 * k:6 * k + 6]) for k, arm in enumerate(arms)] for row in q])
    t_arm = (time.perf_counter() - t0) / (args.samples * cell.n_arms) * 1e6
    t0 = time.perf_counter()
    inter = np.array([pair.config_free(row) for row in q])
    t_pair = (time.perf_counter() - t0) / args.samples * 1e6
    t0 = time.perf_counter()
    expected = np.array([not full.contacts(row) for row in q])
    t_full = (time.perf_counter() - t0) / args.samples * 1e6

    split = per_arm.all(axis=1) & inter
    mismatch = int((split != expected).sum())
    print(f"cell {cell.n_arms} tay, {args.samples} mẫu, khớp ±{cell.joint_limit:.2f}")
    print(f"  µs/lần: offline 1 tay {t_arm:.1f}, online {t_pair:.1f}, model đầy đủ {t_full:.1f}")
    for k in range(cell.n_arms):
        print(f"  tay {k} tự do với môi trường: {per_arm[:, k].mean():.3f}")
    both = per_arm.all(axis=1)
    print(f"  mọi tay tự do với môi trường: {both.mean():.3f}")
    print(f"  trong số đó, va chạm giữa các tay: {1 - inter[both].mean():.3f}")
    print(f"  tự do hoàn toàn: {split.mean():.3f}; lệch so với model đầy đủ: {mismatch}/{args.samples}")


if __name__ == "__main__":
    main()
