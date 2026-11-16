"""Phân việc mức 0 (mục 5.2): mỗi tay phụ trách phần bàn gần đế mình (chia theo hai đường chéo của bàn vuông).

Ở M1 chưa có bảng chi phí từ roadmap, nên "chi phí" là khoảng cách từ đế tới khối; vật sát ranh giới vì thế tự động giao
cho tay gần hơn. Nếu tay gần nhất không có nghiệm gắp (ArmTools.pick), giao cho tay gần kế tiếp.
"""
from __future__ import annotations

import numpy as np

from ..scene.builder import Layout


def assign_level0(L: Layout, cubes, can_pick) -> tuple[dict[int, list[int]], list[int]]:
    """cubes: danh sách CubeSpec; can_pick(arm, cube_index) -> bool. Trả (hàng đợi theo tay, khối không tay nào gắp được)."""
    queues: dict[int, list[int]] = {k: [] for k in range(L.n_arms)}
    orphans = []
    for i, c in enumerate(cubes):
        order = sorted(range(L.n_arms), key=lambda k: np.hypot(c.x - L.base(k)[0], c.y - L.base(k)[1]))
        for k in order:
            if can_pick(k, i):
                queues[k].append(i)
                break
        else:
            orphans.append(i)
    return queues, orphans
