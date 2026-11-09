"""Robot đĩa 2D giữa các vật cản chữ nhật (Mục 8.1), kiểm tra va chạm giải tích, không cần MuJoCo."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

Rect = tuple[float, float, float, float]  # xmin, ymin, xmax, ymax

# Đo từ ảnh render của Hình 9 (bài không cho toạ độ). Lưới 0,5 đơn vị = 32,4 px; hai chấm trong hình rơi đúng vào đường
# lưới ứng với (9, 9) và (4,5, 4,5), từ đó suy ra gốc toạ độ. Khung hình vì vậy là [-0,45; 9,45].
FIG9_RECTS: tuple[Rect, ...] = (
    (0.63, 6.59, 3.91, 7.33),
    (5.07, 4.01, 6.94, 8.90),
    (6.94, 6.23, 8.85, 6.71),
    (0.47, 0.93, 2.57, 3.02),
    (6.98, 0.63, 8.60, 2.22),
)
FIG9_BOUNDS = (-0.45, 9.45)


def _point_segment_dist2(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    t = 0.0 if denom == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    ex, ey = ax + t * dx - px, ay + t * dy - py
    return ex * ex + ey * ey


def _segments_cross(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float],
                    d: tuple[float, float]) -> bool:
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    return side(a, b, c) * side(a, b, d) < 0 and side(c, d, a) * side(c, d, b) < 0


def segment_rect_distance(a: Sequence[float], b: Sequence[float], rect: Rect) -> float:
    """Khoảng cách chính xác từ đoạn thẳng ab tới hình chữ nhật đặc."""
    xmin, ymin, xmax, ymax = rect
    a = (a[0], a[1])
    b = (b[0], b[1])
    if xmin <= a[0] <= xmax and ymin <= a[1] <= ymax:
        return 0.0
    if xmin <= b[0] <= xmax and ymin <= b[1] <= ymax:
        return 0.0
    corners = ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))
    best = math.inf
    for c, d in zip(corners, corners[1:] + corners[:1]):
        if _segments_cross(a, b, c, d):
            return 0.0
        best = min(best, _point_segment_dist2(*c, *a, *b), _point_segment_dist2(*a, *c, *d),
                   _point_segment_dist2(*b, *c, *d))
    return math.sqrt(best)


def perimeter_point(fraction: float, low: float = -0.2, high: float = 9.2) -> tuple[float, float]:
    """Điểm trên biên hình vuông [low, high]², đi ngược chiều kim đồng hồ theo tỉ lệ chu vi."""
    side = high - low
    s = (fraction % 1.0) * 4
    k = int(s)
    t = (s - k) * side
    return ((low + t, low), (high, low + t), (high - t, high), (low, high - t))[k]


def perimeter_swap_query(R: int, offset: float = 0.0) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """(s_i, t_i) của Mục 8.1: R robot xuất phát trên biên, mỗi robot tới phía đối diện."""
    return [(perimeter_point(offset + i / R), perimeter_point(offset + i / R + 0.5)) for i in range(R)]


@dataclass
class DiskSpace:
    """Không gian của một đĩa. Theo Mục 8.1, vật cản trong Hình 9 đã phình sẵn theo bán kính đĩa ("inflated by the
    radius"), nên mặc định tâm đĩa chỉ cần nằm ngoài hình vẽ (clearance = 0)."""

    rects: Sequence[Rect] = FIG9_RECTS
    clearance: float = 0.0
    bounds: tuple[float, float] = FIG9_BOUNDS

    def __post_init__(self) -> None:
        self.lower = np.full(2, self.bounds[0])
        self.upper = np.full(2, self.bounds[1])

    def config_free(self, q: Sequence[float]) -> bool:
        return self.edge_free(q, q)

    def edge_free(self, a: Sequence[float], b: Sequence[float]) -> bool:
        return all(segment_rect_distance(a, b, r) > self.clearance for r in self.rects)

    @property
    def free_area(self) -> float:
        return (self.bounds[1] - self.bounds[0]) ** 2 - sum((r[2] - r[0]) * (r[3] - r[1]) for r in self.rects)


@dataclass
class DiskPairChecker:
    """Va chạm giữa các đĩa khi cùng di chuyển thẳng, tính bằng điểm gần nhất của chuyển động tương đối."""

    radius: float = 0.2

    def __post_init__(self) -> None:
        self._min2 = (2 * self.radius) ** 2

    def edge_free(self, a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> bool:
        limit = self._min2
        R = len(a)
        for i in range(R):
            ai, bi = a[i], b[i]
            for j in range(i + 1, R):
                aj, bj = a[j], b[j]
                x0, y0 = ai[0] - aj[0], ai[1] - aj[1]
                x1, y1 = bi[0] - bj[0], bi[1] - bj[1]
                dx, dy = x1 - x0, y1 - y0
                denom = dx * dx + dy * dy
                t = 0.0 if denom == 0 else max(0.0, min(1.0, -(x0 * dx + y0 * dy) / denom))
                ex, ey = x0 + t * dx, y0 + t * dy
                if ex * ex + ey * ey < limit:
                    return False
        return True


@dataclass
class CompositeDiskSpace:
    """R đĩa như một robot hợp 2R chiều, để dựng N²-PRM* tường minh."""

    base: DiskSpace
    R: int = 2
    radius: float = 0.2

    def __post_init__(self) -> None:
        self.lower = np.tile(self.base.lower, self.R)
        self.upper = np.tile(self.base.upper, self.R)
        self._pairs = DiskPairChecker(self.radius)

    def _split(self, q: Sequence[float]) -> list[Sequence[float]]:
        return [q[2 * i:2 * i + 2] for i in range(self.R)]

    def config_free(self, q: Sequence[float]) -> bool:
        return self.edge_free(q, q)

    def edge_free(self, a: Sequence[float], b: Sequence[float]) -> bool:
        pa, pb = self._split(a), self._split(b)
        return all(self.base.edge_free(x, y) for x, y in zip(pa, pb)) and self._pairs.edge_free(pa, pb)
