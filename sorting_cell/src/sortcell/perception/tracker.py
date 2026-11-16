"""Bản ghi vật từ các lần phát hiện của camera trên cao (mục 5.1, theo dõi và che khuất).

- Mỗi vật một bản ghi: màu, pose, thời điểm thấy rõ lần cuối, trạng thái.
- Ghép phát hiện với bản ghi cùng màu gần nhất trong cổng GATE (ghép tham lam theo khoảng cách tăng dần).
- Chỉ cập nhật pose từ phát hiện thấy rõ (diện tích gần đủ); phát hiện bị che một phần chỉ xác nhận vật còn đó.
- Phát hiện thấy rõ không ghép được → bản ghi mới.
- Bản ghi trên bàn mà ở chỗ của nó camera thấy mặt bàn trống (không bị che) LOST_FRAMES lần liên tiếp → bỏ (vật đã bị đẩy
  đi; nếu nó còn trên bàn thì sẽ có bản ghi mới ở chỗ mới).
- Vật đang được gắp, đang mang hoặc đã thả không cập nhật từ camera (bộ điều khiển đổi trạng thái).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..scene.builder import CUBE, Layout
from .overhead import Detection, OverheadCamera

GATE = 0.04
LOST_FRAMES = 3
EMPTY_TOL = 0.006   # m: điểm ở chỗ bản ghi cao hơn mặt bàn không quá chừng này = chỗ trống


def yaw_diff(a: float, b: float) -> float:
    """Hiệu góc theo đối xứng 90°, trong [-π/4, π/4)."""
    return (a - b + math.pi / 4) % (math.pi / 2) - math.pi / 4


@dataclass
class Track:
    id: int
    color: str
    xy: np.ndarray
    yaw: float
    t_seen: float
    t_first: float
    state: str = "on_table"        # on_table | busy (đang được gắp/mang/thả) | done | lost
    n_full: int = 1
    n_partial: int = 0
    empty_streak: int = 0
    history: list = field(default_factory=list)


class Tracker:
    def __init__(self, L: Layout):
        self.L = L
        self.tracks: list[Track] = []
        self._next = 0

    def live(self) -> list[Track]:
        return [t for t in self.tracks if t.state == "on_table"]

    def get(self, tid: int) -> Track:
        return self.tracks[tid]

    def update(self, dets: list[Detection], t: float, cam: OverheadCamera | None = None, d=None,
               points: np.ndarray | None = None) -> None:
        live = self.live()
        pairs = []
        for di, det in enumerate(dets):
            for tr in live:
                if tr.color != det.color:
                    continue
                dist = float(np.linalg.norm(tr.xy - det.xy))
                if dist < GATE:
                    pairs.append((dist, di, tr.id))
        pairs.sort()
        used_d, used_t = set(), set()
        for dist, di, tid in pairs:
            if di in used_d or tid in used_t:
                continue
            used_d.add(di)
            used_t.add(tid)
            tr, det = self.tracks[tid], dets[di]
            tr.empty_streak = 0
            if det.full:
                tr.xy, tr.yaw, tr.t_seen = det.xy.copy(), det.yaw, t
                tr.n_full += 1
            else:
                tr.n_partial += 1
        # vật bận/đã xong: phát hiện trùng chỗ của chúng không tạo bản ghi mới (ví dụ khối vừa bị nhấc lên một chút)
        others = [tr for tr in self.tracks if tr.state == "busy"]
        for di, det in enumerate(dets):
            if di in used_d or not det.full:
                continue
            if any(tr.color == det.color and np.linalg.norm(tr.xy - det.xy) < GATE for tr in others):
                continue
            self.tracks.append(Track(self._next, det.color, det.xy.copy(), det.yaw, t, t))
            self._next += 1
        # chỗ trống
        if cam is not None and points is not None:
            for tr in live:
                if tr.id in used_t:
                    continue
                u, v = cam.project(d, [*tr.xy, self.L.table_top + CUBE])
                if not (0 <= u < cam.W and 0 <= v < cam.H):
                    continue
                z = points[v, u, 2]
                if abs(z - self.L.table_top) < EMPTY_TOL:
                    tr.empty_streak += 1
                    if tr.empty_streak >= LOST_FRAMES:
                        tr.state = "lost"
                else:
                    tr.empty_streak = 0
