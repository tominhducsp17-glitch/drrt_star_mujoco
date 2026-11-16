"""Lấy mẫu cảnh: N khối 5 cm, 3 màu, trong vùng đặt vật, tâm cách nhau tối thiểu MIN_DIST. Cố định theo seed."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .builder import COLOR_NAMES, CUBE, Layout

MIN_DIST = 0.08


@dataclass(frozen=True)
class CubeSpec:
    name: str
    color: str
    x: float
    y: float
    yaw: float  # [0, π/2): khối đối xứng 90°

    def as_tuple(self, L: Layout):
        return (self.name, (self.x, self.y, L.table_top + CUBE / 2), self.yaw, self.color)


def sample_scene(seed: int, L: Layout | None = None, n: int = 8, min_dist: float = MIN_DIST,
                 max_tries: int = 10000) -> list[CubeSpec]:
    L = L or Layout()
    rng = np.random.default_rng(seed)
    lim = L.zone_half - CUBE / 2 * math.sqrt(2)  # cả khối (kể cả khi xoay 45°) nằm trong vùng
    pts: list[np.ndarray] = []
    for _ in range(max_tries):
        p = rng.uniform(-lim, lim, 2)
        if all(np.linalg.norm(p - q) >= min_dist for q in pts):
            pts.append(p)
            if len(pts) == n:
                break
    else:
        raise RuntimeError("không đặt đủ khối")
    colors = rng.integers(0, len(COLOR_NAMES), n)
    yaws = rng.uniform(0, math.pi / 2, n)
    return [CubeSpec(f"cube{i}", COLOR_NAMES[c], float(p[0]), float(p[1]), float(w))
            for i, (p, c, w) in enumerate(zip(pts, colors, yaws))]


def eval_scenes(count: int = 50, L: Layout | None = None) -> list[list[CubeSpec]]:
    """Bộ cảnh đánh giá mục 2: seed 1000..1000+count-1."""
    return [sample_scene(1000 + i, L) for i in range(count)]


# ---------------------------------------------------------------------------------------------------- M4: khối sát nhau
CLUSTER_GAP = 0.02      # khe lớn nhất giữa hai khối trong một cụm
TOUCH_GAP = 0.0005      # khe nhỏ nhất (0,5 mm): coi như chạm, tránh lún vào nhau lúc bắt đầu mô phỏng


def square_corners(x: float, y: float, yaw: float, half: float = CUBE / 2) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return np.array([x, y]) + np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]]) * half @ R.T


def squares_gap(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    """Khe nhỏ nhất giữa hai hình vuông cạnh CUBE (x, y, yaw) theo định lý trục tách; âm nếu chồng nhau (độ lún theo
    trục tách tốt nhất)."""
    A, B = square_corners(*a), square_corners(*b)
    best = -np.inf
    for yaw in (a[2], b[2]):
        for ax in (np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])):
            pa, pb = A @ ax, B @ ax
            best = max(best, pb.min() - pa.max(), pa.min() - pb.max())
    return float(best)


def sample_cluster_scene(seed: int, L: Layout | None = None, n: int = 8, max_tries: int = 20000) -> list[CubeSpec]:
    """Cảnh có khối sát nhau (M4): vài cụm 2–4 khối, mỗi khối mới đặt cạnh một khối có sẵn trong cụm với khe 0,5–1,5 mm
    (nửa số lần) hoặc 0,5–20 mm,
    nửa số lần cùng góc (thẳng hàng theo một cạnh), nửa số lần góc ngẫu nhiên; phần còn lại là khối rời. Các cụm và khối
    rời cách nhau như cảnh thường (tâm ≥ MIN_DIST với mọi khối khác cụm). Màu ngẫu nhiên độc lập, nên có cả cặp cùng màu
    và khác màu."""
    L = L or Layout()
    rng = np.random.default_rng([seed, 4])
    lim = L.zone_half - CUBE / 2 * math.sqrt(2)
    for _ in range(max_tries):
        target = int(rng.integers(4, n + 1))       # số khối nằm trong cụm
        sizes = []
        while sum(sizes) < target:
            k = int(min(rng.integers(2, 5), target - sum(sizes)))
            if k < 2:
                break
            sizes.append(k)
        cubes: list[tuple[float, float, float]] = []
        group: list[int] = []
        ok = True
        for g, size in enumerate(sizes):
            members: list[tuple[float, float, float]] = []
            for _try in range(200):
                p = rng.uniform(-lim, lim, 2)
                if all(np.hypot(p[0] - c[0], p[1] - c[1]) >= MIN_DIST + CUBE for c in cubes):
                    members = [(float(p[0]), float(p[1]), float(rng.uniform(0, math.pi / 2)))]
                    break
            if not members:
                ok = False
                break
            tries = 0
            while len(members) < size and tries < 500:
                tries += 1
                ax_, ay_, ayaw = members[int(rng.integers(len(members)))]
                # nửa số cặp gần như chạm (0,5–1,5 mm), nửa còn lại khe tới 2 cm
                gap = float(rng.uniform(TOUCH_GAP, 0.0015) if rng.random() < 0.5 else rng.uniform(TOUCH_GAP, CLUSTER_GAP))
                if rng.random() < 0.5:     # cùng góc, áp vào một mặt
                    th = ayaw + int(rng.integers(4)) * math.pi / 2
                    yaw = ayaw
                    off = rng.uniform(-0.3, 0.3) * CUBE   # trượt dọc mặt tiếp xúc
                    d = np.array([math.cos(th), math.sin(th)]) * (CUBE + gap) + \
                        np.array([-math.sin(th), math.cos(th)]) * off
                    cand = (ax_ + d[0], ay_ + d[1], yaw)
                else:                      # góc ngẫu nhiên: đẩy ra theo hướng ngẫu nhiên tới khi khe = gap
                    th = rng.uniform(0, 2 * math.pi)
                    yaw = float(rng.uniform(0, math.pi / 2))
                    lo, hi = 0.0, 2 * CUBE
                    for _ in range(40):
                        mid = (lo + hi) / 2
                        c2 = (ax_ + mid * math.cos(th), ay_ + mid * math.sin(th), yaw)
                        if squares_gap((ax_, ay_, ayaw), c2) < gap:
                            lo = mid
                        else:
                            hi = mid
                    cand = (ax_ + hi * math.cos(th), ay_ + hi * math.sin(th), yaw)
                if max(abs(cand[0]), abs(cand[1])) > lim:
                    continue
                if any(squares_gap(cand, m) < TOUCH_GAP * 0.99 for m in members):
                    continue
                if any(np.hypot(cand[0] - c[0], cand[1] - c[1]) < MIN_DIST + CUBE / 2 for c in cubes):
                    continue
                members.append((float(cand[0]), float(cand[1]), float(cand[2]) % (math.pi / 2)))
            if len(members) < size:
                ok = False
                break
            cubes += members
            group += [g] * len(members)
        if not ok:
            continue
        while len(cubes) < n:
            placed = False
            for _try in range(500):
                p = rng.uniform(-lim, lim, 2)
                if all(np.hypot(p[0] - c[0], p[1] - c[1]) >= MIN_DIST for c in cubes):
                    cubes.append((float(p[0]), float(p[1]), float(rng.uniform(0, math.pi / 2))))
                    group.append(-1)
                    placed = True
                    break
            if not placed:
                break
        if len(cubes) < n:
            continue
        colors = rng.integers(0, len(COLOR_NAMES), n)
        return [CubeSpec(f"cube{i}", COLOR_NAMES[c], x, y, yaw)
                for i, ((x, y, yaw), c) in enumerate(zip(cubes, colors))]
    raise RuntimeError("không dựng được cảnh có cụm khối")


def cluster_scenes(count: int = 50, L: Layout | None = None) -> list[list[CubeSpec]]:
    """Bộ cảnh đánh giá M4 (khối sát nhau): seed 2000..2000+count-1."""
    return [sample_cluster_scene(2000 + i, L) for i in range(count)]


# ---------------------------------------------------------------------------------------------------- M4: thả thêm khối
PARK = (2.5, 2.5)        # chỗ đỗ khối chưa thả: trên sàn, ngoài tầm nhìn camera trên cao


@dataclass(frozen=True)
class Arrival:
    t: float             # thời điểm thả (s mô phỏng, tính từ lúc cell bắt đầu)
    cube: int            # chỉ số khối trong danh sách khối của cảnh
    x: float             # chỗ thả mong muốn; nếu không trống thì chọn chỗ trống khác lúc thả
    y: float
    yaw: float


def sample_arrival_scene(seed: int, L: Layout | None = None, n0: int = 8, n_new: int = 4,
                         t_range: tuple[float, float] = (5.0, 40.0)) -> tuple[list[CubeSpec], list[Arrival]]:
    """Cảnh thường n0 khối + n_new khối thả thêm vào lúc ngẫu nhiên trong t_range. Khối chưa thả nằm ở PARK."""
    L = L or Layout()
    base = sample_scene(seed, L, n=n0)
    rng = np.random.default_rng([seed, 9])
    lim = L.zone_half - CUBE / 2 * math.sqrt(2)
    cubes, arrivals = list(base), []
    times = np.sort(rng.uniform(*t_range, n_new))
    for j, t in enumerate(times):
        i = n0 + j
        color = COLOR_NAMES[int(rng.integers(len(COLOR_NAMES)))]
        cubes.append(CubeSpec(f"cube{i}", color, PARK[0] + 0.1 * j, PARK[1], 0.0))
        p = rng.uniform(-lim, lim, 2)
        arrivals.append(Arrival(float(t), i, float(p[0]), float(p[1]), float(rng.uniform(0, math.pi / 2))))
    return cubes, arrivals


def scene_for_seed(seed: int, L: Layout | None = None) -> tuple[list[CubeSpec], list[Arrival]]:
    """Seed 2000–2999: khối sát nhau; 3000–3999: cảnh thường + thả thêm 4 khối giữa chừng; còn lại: cảnh thường."""
    if 2000 <= seed < 3000:
        return sample_cluster_scene(seed, L), []
    if 3000 <= seed < 4000:
        return sample_arrival_scene(seed, L)
    return sample_scene(seed, L), []
