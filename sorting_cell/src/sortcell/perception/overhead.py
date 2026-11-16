"""Phát hiện khối từ camera RGB-D trên cao (mục 5.1).

1. Depth → điểm 3D trong toạ độ cell (thông số camera lấy từ model: fovy, vị trí, hướng). Depth của MuJoCo là khoảng cách
   dọc trục quang (không phải dọc tia).
2. Chỉ giữ điểm trên mặt bàn ở độ cao mặt trên của khối (dải ±BAND): bỏ khay (thấp hơn), cánh tay phía trên, và mặt bên của
   khối (để tâm và góc không bị lệch do phối cảnh).
3. HSV theo ngưỡng từng màu; mở hình thái 3×3; thành phần liên thông.
4. Mỗi thành phần: minAreaRect trên toạ độ xy (mm) của các điểm → tâm, góc (khối đối xứng 90° nên quy về [0°, 90°)), cạnh.
   Diện tích so với diện tích mặt trên kỳ vọng (theo độ sâu) cho biết khối bị che một phần hay không.
5. (M4) Thành phần có diện tích > 1,5 mặt: các khối cùng màu dính nhau; tách bằng hình học (split_component).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import mujoco as mj
import numpy as np

from ..scene.builder import COLOR_NAMES, CUBE, Layout
from ..scene.sampling import squares_gap

BAND = 0.010           # m quanh độ cao mặt trên khối
FULL_FRAC = 0.85       # diện tích trong [85%, 115%] kỳ vọng: thấy rõ, được cập nhật pose
FULL_MAX = 1.15        # lớn hơn mà chưa tới SPLIT_FRAC: có thể là hai khối dính nhau, một khối bị che: không chắc
MIN_FRAC = 0.15        # nhỏ hơn: bỏ
MAX_FRAC = 1.6         # lớn hơn: không phải một khối, thử tách thành nhiều khối (M4)
SPLIT_FRAC = 1.5       # diện tích > 1,5 lần một mặt: vùng gồm round(diện tích) khối dính nhau
MAX_SPLIT = 6
SPLIT_COVER = 0.92     # tách được chấp nhận khi các hình vuông phủ ≥ 92% điểm của vùng
SPLIT_FILL = 0.85      # và mỗi hình vuông có ≥ 85% diện tích nằm trong vùng
SPLIT_TOL = 0.003      # m: điểm cách cạnh hình vuông không quá chừng này vẫn tính là nằm trong
# Ngưỡng HSV của OpenCV (H trong [0, 180)). Lấy từ ảnh render: mặt trên khối đỏ H 0–1, xanh lá 61–63, xanh dương 110–114,
# S ≥ 61, V ≥ 143. Phần xanh của cánh tay UR (H ≈ 101–104) và khay nằm ngoài dải độ cao.
HSV_RANGES = {
    "red": [((0, 60, 90), (8, 255, 255)), ((172, 60, 90), (179, 255, 255))],
    "green": [((50, 60, 90), (75, 255, 255))],
    "blue": [((106, 60, 90), (120, 255, 255))],
}
assert tuple(HSV_RANGES) == COLOR_NAMES


@dataclass
class Detection:
    color: str
    xy: np.ndarray          # tâm (m)
    z: float                # độ cao mặt trên (m)
    yaw: float              # [0, π/2)
    area_frac: float        # diện tích / diện tích mặt trên kỳ vọng
    side: tuple[float, float]
    split: int = 1          # > 1: tách ra từ một vùng gồm `split` khối dính nhau (M4)

    @property
    def full(self) -> bool:
        return FULL_FRAC <= self.area_frac <= FULL_MAX


def _square_dist(pts: np.ndarray, sq: tuple[float, float, float], side: float = CUBE) -> np.ndarray:
    """Khoảng cách có dấu (chuẩn vô cùng) từ điểm tới hình vuông cạnh `side`: âm = ở trong."""
    x, y, yaw = sq
    c, s = math.cos(yaw), math.sin(yaw)
    d = pts - [x, y]
    lx, ly = d[:, 0] * c + d[:, 1] * s, -d[:, 0] * s + d[:, 1] * c
    return np.maximum(np.abs(lx), np.abs(ly)) - side / 2


def _fit_squares(pts: np.ndarray, sqs: list, iters: int = 20) -> list:
    """Lặp: gán mỗi điểm cho hình vuông "chứa nó nhiều nhất", đặt lại tâm = trung bình điểm được gán (mặt trên nhìn thấy
    trọn thì đúng là tâm) và góc = minAreaRect của các điểm đó."""
    for _ in range(iters):
        D = np.stack([_square_dist(pts, q) for q in sqs], 1)
        own = D.argmin(1)
        new = []
        for k, q in enumerate(sqs):
            P = pts[own == k]
            if len(P) < 10:
                new.append(q)
                continue
            (_, _), (_, _), ang = cv2.minAreaRect((P * 1000).astype(np.float32))
            cx, cy = P.mean(0)
            new.append((float(cx), float(cy), math.radians(ang) % (math.pi / 2)))
        if all(np.allclose(a, b, atol=1e-5) for a, b in zip(new, sqs)):
            break
        sqs = new
    return sqs


def _edge_fit(edge: np.ndarray, sq: tuple[float, float, float], side: float,
              others: list | None = None, keep_center: bool = False) -> tuple[float, float, float]:
    """Khớp bình phương tối thiểu pose (x, y, yaw) của hình vuông cạnh `side` vào các điểm biên của vùng (tâm pixel biên,
    toạ độ thật). Chỉ dùng điểm biên gần cạnh của hình vuông này (≤ 4 mm) và không gần hình vuông khác (bỏ đường tiếp giáp
    giữa hai khối dính nhau). Độ chính xác cỡ một phần pixel, tốt hơn minAreaRect trên pixel (lệch 1 hàng pixel ≈ 3°)."""
    from scipy.optimize import least_squares
    d0 = np.abs(_square_dist(edge, sq, side))
    use = d0 < 0.004
    for q in others or []:
        use &= np.abs(_square_dist(edge, q, side)) > 0.004
    P = edge[use]
    if len(P) < 20:
        return sq
    res = least_squares(lambda v: _square_dist(P, (v[0], v[1], v[2]), side), np.array(sq, float),
                        loss="soft_l1", f_scale=0.002)
    x, y, a = res.x
    if np.hypot(x - sq[0], y - sq[1]) > 0.005 or abs((a - sq[2] + math.pi / 4) % (math.pi / 2) - math.pi / 4) > math.radians(8):
        return sq  # khớp chạy xa: giữ nghiệm cũ
    if keep_center:
        return sq[0], sq[1], float(a) % (math.pi / 2)
    return float(x), float(y), float(a) % (math.pi / 2)


def split_component(color: str, pts3: np.ndarray, k: int, frac: float, starts: int = 6,
                    edge: np.ndarray | None = None, px: float = 0.0) -> list[Detection]:
    """Tách một vùng màu gồm k khối dính nhau bằng hình học (M4, không learning): biết mỗi khối có mặt trên là hình vuông
    cạnh CUBE, tìm k hình vuông phủ vùng tốt nhất (khởi đầu k-means nhiều lần, rồi _fit_squares). Chấp nhận khi phủ
    ≥ SPLIT_COVER điểm của vùng và mỗi hình vuông có ≥ SPLIT_FILL diện tích trong vùng, không chồng lên nhau; khi đó mỗi
    khối là một phát hiện "thấy rõ". Không thì trả các phát hiện "bị che một phần" (tracker không cập nhật, không tạo bản
    ghi mới, nên khối chưa được gắp)."""
    pts = pts3[:, :2].astype(np.float64)
    z = float(np.median(pts3[:, 2]))
    per_sq = len(pts) / frac                      # số điểm của một mặt trên trọn vẹn
    rng = np.random.default_rng(len(pts))
    best, best_score = None, -math.inf
    for s in range(starts):
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-5)
        cv2.setRNGSeed(int(rng.integers(1 << 30)))
        _, lab, cent = cv2.kmeans(pts.astype(np.float32), k, None, crit, 1, cv2.KMEANS_PP_CENTERS)
        sqs = []
        for j in range(k):
            P = pts[lab.ravel() == j]
            ang = cv2.minAreaRect((P * 1000).astype(np.float32))[2] if len(P) >= 5 else 0.0
            sqs.append((float(cent[j][0]), float(cent[j][1]), math.radians(ang) % (math.pi / 2)))
        sqs = _fit_squares(pts, sqs)
        D = np.stack([_square_dist(pts, q) for q in sqs], 1)
        cover = float(np.mean(D.min(1) <= SPLIT_TOL))
        fill = [float(np.sum(D[:, j] <= SPLIT_TOL) / per_sq) for j in range(k)]
        overlap = max((-squares_gap(a, b) for i, a in enumerate(sqs) for b in sqs[i + 1:]), default=0.0)
        score = cover * min(fill) - max(overlap - SPLIT_TOL, 0) * 100
        if not math.isfinite(score):
            score = -1e9
        if best is None or score > best_score:
            best, best_score = (sqs, cover, fill, overlap), score
    sqs, cover, fill, overlap = best
    ok = cover >= SPLIT_COVER and min(fill) >= SPLIT_FILL and overlap <= 2 * SPLIT_TOL
    if ok and edge is not None:
        sqs = [_edge_fit(edge, q, CUBE - px, [o for o in sqs if o is not q]) for q in sqs]
    out = []
    for q, f in zip(sqs, fill):
        out.append(Detection(color, np.array(q[:2]), z, q[2], float(min(f, 1.0)) if ok else min(float(f), FULL_FRAC - 0.01),
                             (CUBE, CUBE), split=k))
    return out


class OverheadCamera:
    def __init__(self, m: mj.MjModel, L: Layout, width: int = 640, height: int = 480, name: str = "overhead",
                 split: bool = True):
        self.m, self.L = m, L
        self.split = split  # False: bỏ vùng gồm nhiều khối dính nhau (so sánh ở M4)
        self.W, self.H = width, height
        self.cid = m.camera(name).id
        self.name = name
        self.f = height / 2 / math.tan(math.radians(m.cam_fovy[self.cid]) / 2)
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        self._ux = (u + 0.5 - width / 2) / self.f
        self._vy = -(v + 0.5 - height / 2) / self.f
        self._r = None
        self._split_cache: dict = {}   # vùng không đổi giữa hai khung hình (cùng thống kê pixel): dùng lại kết quả tách

    def _renderer(self) -> mj.Renderer:
        if self._r is None:
            self._r = mj.Renderer(self.m, self.H, self.W)
        return self._r

    def capture(self, d: mj.MjData) -> tuple[np.ndarray, np.ndarray]:
        r = self._renderer()
        r.update_scene(d, self.name)
        rgb = r.render().copy()
        r.enable_depth_rendering()
        r.update_scene(d, self.name)
        dep = r.render().copy()
        r.disable_depth_rendering()
        return rgb, dep

    def segmentation(self, d: mj.MjData, scene_option: mj.MjvOption | None = None) -> np.ndarray:
        """Chỉ dùng để đánh giá: id geom ở mỗi pixel (-1 nếu không phải geom)."""
        r = self._renderer()
        r.enable_segmentation_rendering()
        r.update_scene(d, self.name, scene_option=scene_option)
        seg = r.render().copy()
        r.disable_segmentation_rendering()
        ids = seg[..., 0].astype(int)
        ids[seg[..., 1] != mj.mjtObj.mjOBJ_GEOM] = -1
        return ids

    def points(self, d: mj.MjData, depth: np.ndarray) -> np.ndarray:
        R = d.cam_xmat[self.cid].reshape(3, 3)
        cam = np.stack([self._ux * depth, self._vy * depth, -depth], -1)
        return d.cam_xpos[self.cid] + cam @ R.T

    def project(self, d: mj.MjData, p) -> tuple[int, int]:
        R = d.cam_xmat[self.cid].reshape(3, 3)
        c = R.T @ (np.asarray(p, float) - d.cam_xpos[self.cid])
        u = self.W / 2 + self.f * c[0] / -c[2] - 0.5
        v = self.H / 2 - self.f * c[1] / -c[2] - 0.5
        return int(round(u)), int(round(v))

    def expected_px(self, depth: float) -> float:
        return (CUBE * self.f / depth) ** 2

    def detect(self, d: mj.MjData, rgb: np.ndarray, depth: np.ndarray) -> tuple[list[Detection], np.ndarray]:
        """Trả các phát hiện và mảng điểm 3D (để tracker kiểm tra chỗ trống)."""
        L = self.L
        P = self.points(d, depth)
        z_top = L.table_top + CUBE
        lim = L.table_half
        roi = ((np.abs(P[..., 2] - z_top) < BAND) & (np.abs(P[..., 0]) < lim) & (np.abs(P[..., 1]) < lim))
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        kernel = np.ones((3, 3), np.uint8)
        out = []
        cam_z = d.cam_xpos[self.cid][2]
        exp = self.expected_px(cam_z - z_top)
        for color, ranges in HSV_RANGES.items():
            mask = np.zeros(rgb.shape[:2], np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask &= roi.astype(np.uint8) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            for c in range(1, n):
                area = stats[c, cv2.CC_STAT_AREA]
                frac = area / exp
                if frac < MIN_FRAC:
                    continue
                comp = lab == c
                pts = P[comp]
                edge = self._edge_points(comp, P)
                px = (cam_z - z_top) / self.f          # cỡ một pixel trên mặt trên khối (m)
                k = int(round(frac)) if frac > SPLIT_FRAC else 1
                if k == 1 and frac <= MAX_FRAC:
                    (cx, cy), (w, h), ang = cv2.minAreaRect((pts[:, :2] * 1000).astype(np.float32))
                    sq = (cx / 1000, cy / 1000, math.radians(ang) % (math.pi / 2))
                    if FULL_FRAC <= frac <= FULL_MAX:
                        sq = _edge_fit(edge, sq, CUBE - px)
                    out.append(Detection(color, np.array(sq[:2]), float(np.median(pts[:, 2])), sq[2], float(frac),
                                         (w / 1000, h / 1000)))
                    continue
                if k > MAX_SPLIT or not self.split:
                    continue
                key = (color, k, tuple(int(v) for v in stats[c]))
                hit = self._split_cache.get(key)
                if hit is None:
                    if len(self._split_cache) > 256:
                        self._split_cache.clear()
                    hit = self._split_cache[key] = split_component(color, pts, k, frac, edge=edge, px=px)
                out += hit
        return out, P

    @staticmethod
    def _edge_points(comp: np.ndarray, P: np.ndarray) -> np.ndarray:
        """Toạ độ thật (xy) của các pixel biên của một vùng (đường bao ngoài, mọi điểm)."""
        cs, _ = cv2.findContours(comp.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not cs:
            return np.zeros((0, 2))
        uv = np.concatenate([c_[:, 0, :] for c_ in cs])
        return P[uv[:, 1], uv[:, 0], :2]

    def close(self) -> None:
        if self._r is not None:
            self._r.close()
            self._r = None
