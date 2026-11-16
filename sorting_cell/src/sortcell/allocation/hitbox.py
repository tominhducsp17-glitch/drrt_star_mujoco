"""Hitbox elip của Oberthür et al. (IFAC 2025): elip bao đoạn nối đế tay với vật, dùng để dự đoán tranh chấp.

Elip có hai tiêu điểm là đế tay và vật (mặt phẳng ngang), bán trục lớn = nửa khoảng cách + W, bán trục nhỏ = W. W là
nửa bề rộng dải mà cánh tay và gripper chiếm khi vươn tới vật. Hai lựa chọn tranh chấp khi elip của chúng giao nhau.
"""
from __future__ import annotations

import math

import numpy as np

W = 0.15
_T = np.linspace(0, 2 * math.pi, 48, endpoint=False)


class Ellipse:
    def __init__(self, base_xy, obj_xy, w: float = W):
        p, q = np.asarray(base_xy, float), np.asarray(obj_xy, float)
        self.c = (p + q) / 2
        d = float(np.linalg.norm(q - p))
        self.a = d / 2 + w
        self.b = w
        self.th = math.atan2(q[1] - p[1], q[0] - p[0]) if d > 1e-9 else 0.0
        self.R = np.array([[math.cos(self.th), -math.sin(self.th)], [math.sin(self.th), math.cos(self.th)]])

    def contains(self, pts) -> np.ndarray:
        local = (np.atleast_2d(pts) - self.c) @ self.R
        return (local[:, 0] / self.a) ** 2 + (local[:, 1] / self.b) ** 2 <= 1.0

    def boundary(self) -> np.ndarray:
        return self.c + np.column_stack([self.a * np.cos(_T), self.b * np.sin(_T)]) @ self.R.T

    def intersects(self, other: "Ellipse") -> bool:
        """Xấp xỉ bằng 48 điểm biên mỗi elip và tâm: đủ cho kích thước ở đây (sai lệch dưới 1 cm)."""
        return bool(self.contains(other.boundary()).any() or other.contains(self.boundary()).any()
                    or self.contains(other.c[None]).any() or other.contains(self.c[None]).any())
