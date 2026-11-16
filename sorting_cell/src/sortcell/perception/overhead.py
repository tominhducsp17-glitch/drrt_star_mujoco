"""Phát hiện khối từ camera RGB-D trên cao (mục 5.1).

1. Depth → điểm 3D trong toạ độ cell (thông số camera lấy từ model: fovy, vị trí, hướng). Depth của MuJoCo là khoảng cách
   dọc trục quang (không phải dọc tia).
2. Chỉ giữ điểm trên mặt bàn ở độ cao mặt trên của khối (dải ±BAND): bỏ khay (thấp hơn), cánh tay phía trên, và mặt bên của
   khối (để tâm và góc không bị lệch do phối cảnh).
3. HSV theo ngưỡng từng màu; mở hình thái 3×3; thành phần liên thông.
4. Mỗi thành phần: minAreaRect trên toạ độ xy (mm) của các điểm → tâm, góc (khối đối xứng 90° nên quy về [0°, 90°)), cạnh.
   Diện tích so với diện tích mặt trên kỳ vọng (theo độ sâu) cho biết khối bị che một phần hay không.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import mujoco as mj
import numpy as np

from ..scene.builder import COLOR_NAMES, CUBE, Layout

BAND = 0.010           # m quanh độ cao mặt trên khối
FULL_FRAC = 0.85       # diện tích ≥ 85% kỳ vọng: thấy rõ, được cập nhật pose
MIN_FRAC = 0.15        # nhỏ hơn: bỏ
MAX_FRAC = 1.6         # lớn hơn: không phải một khối
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

    @property
    def full(self) -> bool:
        return self.area_frac >= FULL_FRAC


class OverheadCamera:
    def __init__(self, m: mj.MjModel, L: Layout, width: int = 640, height: int = 480, name: str = "overhead"):
        self.m, self.L = m, L
        self.W, self.H = width, height
        self.cid = m.camera(name).id
        self.name = name
        self.f = height / 2 / math.tan(math.radians(m.cam_fovy[self.cid]) / 2)
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        self._ux = (u + 0.5 - width / 2) / self.f
        self._vy = -(v + 0.5 - height / 2) / self.f
        self._r = None

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
                if frac < MIN_FRAC or frac > MAX_FRAC:
                    continue
                pts = P[lab == c]
                (cx, cy), (w, h), ang = cv2.minAreaRect((pts[:, :2] * 1000).astype(np.float32))
                out.append(Detection(color, np.array([cx, cy]) / 1000, float(np.median(pts[:, 2])),
                                     math.radians(ang) % (math.pi / 2), float(frac), (w / 1000, h / 1000)))
        return out, P

    def close(self) -> None:
        if self._r is not None:
            self._r.close()
            self._r = None
