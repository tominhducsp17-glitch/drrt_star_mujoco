"""Phân việc mức 1 (mục 5.2): mỗi khi có tay rảnh, chọn vật hợp lệ có chi phí nhỏ nhất.

chi phí(tay k, vật i) = thời gian tới vật (từ chỗ hiện tại của tay: HOME hoặc khay vừa thả)
                       + thời gian từ vật tới khay đúng màu
                       + PENALTY × số tay khác đang hoạt động mà hitbox của chúng giao hitbox (k, i) hoặc chứa vật i.
Vật hợp lệ: chưa giao, đang nằm trên bàn, tay k với tới được (chi phí hữu hạn). Ở M2 pose lấy từ sim nên luôn mới.
"""
from __future__ import annotations

import numpy as np

from ..scene.builder import COLOR_NAMES, Layout
from .costs import CostTable
from .hitbox import Ellipse

PENALTY = 3.0  # giây cho mỗi tay tranh chấp


def rank_level1(L: Layout, k: int, where: int | None, cands: list[tuple[int, np.ndarray, str]],
                table: CostTable, active: list[tuple[int, np.ndarray]], penalty: float = PENALTY) -> list[tuple[float, int]]:
    """where: None = HOME, j = khay j của tay k. cands: (vật, xy, màu). active: (tay khác, xy vật nó đang nhắm).
    Trả [(chi phí, vật)] tăng dần, bỏ vật không với tới."""
    base = L.base(k)[:2]
    others = [(Ellipse(L.base(a)[:2], xy), xy) for a, xy in active if a != k]
    out = []
    for i, xy, color in cands:
        t_to = table.t_home_to(xy) if where is None else table.t_tray_to(where, xy)
        t_back = table.t_tray_to(COLOR_NAMES.index(color), xy)
        c = t_to + t_back
        if not np.isfinite(c):
            continue
        mine = Ellipse(base, xy)
        n = sum(mine.intersects(e) or e.contains(np.asarray(xy)[None]).any() for e, _ in others)
        out.append((c + penalty * n, i))
    out.sort()
    return out
