"""Hậu xử lý ngoài bài: rút ngắn đường đi hợp bằng shortcut ngẫu nhiên, nối thẳng hai điểm trên đường nếu không va chạm."""
from __future__ import annotations

from typing import Callable, Sequence

import numpy as np


def path_length(path: Sequence[np.ndarray], R: int) -> float:
    return float(sum(np.linalg.norm((b - a).reshape(R, -1), axis=1).sum() for a, b in zip(path, path[1:])))


def shortcut(path: Sequence[np.ndarray], edge_free: Callable[[np.ndarray, np.ndarray], bool], R: int,
             rng: np.random.Generator, attempts: int = 300) -> list[np.ndarray]:
    """Chọn hai vị trí ngẫu nhiên dọc đường (theo chỉ số đỉnh, có nội suy), nối thẳng nếu ngắn hơn và không va chạm."""
    pts = [np.asarray(p, float) for p in path]
    for _ in range(attempts):
        if len(pts) < 3:
            break
        i, j = sorted(rng.uniform(0, len(pts) - 1, 2))
        a_idx, b_idx = int(i), int(j)
        if b_idx - a_idx < 1:
            continue
        a = pts[a_idx] + (pts[a_idx + 1] - pts[a_idx]) * (i - a_idx)
        b = pts[b_idx] + (pts[min(b_idx + 1, len(pts) - 1)] - pts[b_idx]) * (j - b_idx)
        old = [a] + pts[a_idx + 1:b_idx + 1] + [b]
        if path_length([a, b], R) >= path_length(old, R) - 1e-9:
            continue
        if edge_free(a, b):
            pts = pts[:a_idx + 1] + [a, b] + pts[b_idx + 1:]
            pts = [p for k, p in enumerate(pts) if k == 0 or np.max(np.abs(p - pts[k - 1])) > 1e-9]
    return pts
