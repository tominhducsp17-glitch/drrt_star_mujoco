"""Mô hình thế giới mà bộ điều phối dùng: vật nào đang có, ở đâu, màu gì.

- TruthWorld: pose thật từ sim (M1, M2). Mã vật = chỉ số khối trong sim.
- PerceivedWorld: camera trên cao + tracker (M3). Mã vật = mã bản ghi của tracker; bộ điều phối không biết chỉ số khối
  thật. `truth()` ghép bản ghi với khối thật gần nhất CHỈ để đánh giá (phân loại va chạm tay–khối, ghi log sai số), không
  dùng cho quyết định.

Cả hai cùng giao diện: update(), ids(), pose(), color(), check_cubes(), truth(), holding().
PerceivedWorld cũng ghi lại, ở mỗi khung hình, sai số pose của các bản ghi và độ che khuất thật của từng khối (từ ảnh
phân đoạn) để đánh giá perception khi cell đang chạy.
"""
from __future__ import annotations

import math

import mujoco as mj
import numpy as np

from ..perception.overhead import OverheadCamera
from ..perception.tracker import Tracker, yaw_diff
from ..scene.builder import CUBE, Layout
from ..scene.sampling import CubeSpec
from .executor import CellSim

DRIVER_HELD = 0.70     # rad: khớp driver của gripper đóng hẳn khi trống (0,8); kẹp khối 5 cm thì nhỏ hơn nhiều
TRUTH_GATE = 0.05
FAR = np.array([10.0, 10.0, -1.0])


class TruthWorld:
    perceived = False

    def __init__(self, sim: CellSim, cubes: list[CubeSpec]):
        self.sim, self.cubes = sim, cubes

    def update(self, force: bool = False) -> None:
        pass

    def ids(self) -> list[int]:
        return list(range(len(self.cubes)))

    def lost(self, oid: int) -> bool:
        return False

    def pose(self, oid: int) -> tuple[np.ndarray, float]:
        return self.sim.cube_pose(oid)

    def color(self, oid: int) -> str:
        return self.cubes[oid].color

    def truth(self, oid: int) -> int | None:
        return oid

    def set_busy(self, oid: int, busy: bool) -> None:
        pass

    def set_done(self, oid: int) -> None:
        pass

    def check_cubes(self, target: int | None):
        """(vị trí giả định của các khối cho kiểm tra đoạn đi xuống, chỉ số khối đích); None = dùng trạng thái sim."""
        return None, target

    def holding(self, k: int, oid: int) -> bool:
        """Sau khi nhấc: tay k có đang giữ vật không (bộ điều khiển dùng để quyết định)."""
        rise = self.sim.cube_pose(oid)[0][2] - (self.sim.L.table_top + CUBE / 2)
        return rise >= 0.05


class PerceivedWorld:
    perceived = True

    def __init__(self, sim: CellSim, cubes: list[CubeSpec], period: float = 0.2, log_eval: bool = True):
        self.sim, self.cubes, self.L = sim, cubes, sim.L
        self.cam = OverheadCamera(sim.m, sim.L)
        self.tracker = Tracker(sim.L)
        self.period = period
        self.next_t = 0.0
        self.log_eval = log_eval
        self.frames: list[dict] = []
        self._driver = [[sim.m.joint(f"a{k}_g_{s}_driver_joint").qposadr[0] for s in ("right", "left")]
                        for k in range(sim.L.n_arms)]
        self._cube_qadr = [sim.m.jnt_qposadr[sim.m.body_jntadr[b]] for b in sim.cube_body]
        self._cube_geoms = [sim.m.geom(c.name + "_geom").id for c in cubes]
        self._opt_cubes = None
        if log_eval:  # ảnh phân đoạn không có tay: số pixel của khối khi không bị che
            opt = mj.MjvOption()
            opt.geomgroup[:] = 0
            opt.geomgroup[0] = 1
            self._opt_cubes = opt

    # ------------------------------------------------------------------ cập nhật
    def update(self, force: bool = False) -> None:
        sim = self.sim
        if not force and sim.time < self.next_t - 1e-9:
            return
        self.next_t = sim.time + self.period
        rgb, dep = self.cam.capture(sim.d)
        dets, P = self.cam.detect(sim.d, rgb, dep)
        self.tracker.update(dets, sim.time, self.cam, sim.d, P)
        if self.log_eval:
            self._log_frame(dets)

    def _log_frame(self, dets) -> None:
        sim, L = self.sim, self.L
        seg = self.cam.segmentation(sim.d)
        # số pixel khi không bị che: chỉ hiện nhóm geom 0 (bàn, bệ, khay, khối; không có tay), không gì nằm trên khối
        seg0 = self.cam.segmentation(sim.d, self._opt_cubes)
        cubes = []
        for i, g in enumerate(self._cube_geoms):
            pos, yaw = sim.cube_pose(i)
            on_table = abs(pos[2] - (L.table_top + CUBE / 2)) < 0.01 and max(abs(pos[0]), abs(pos[1])) < L.table_half
            if not on_table:
                continue
            full = int((seg0 == g).sum())
            vis = int((seg == g).sum())
            cubes.append({"i": i, "vis": vis / full if full else 0.0})
        tracks = []
        for tr in self.tracker.tracks:
            if tr.state != "on_table":
                continue
            j = self._nearest(tr.xy)
            if j is None:
                tracks.append({"id": tr.id, "truth": None})
                continue
            pos, yaw = sim.cube_pose(j)
            tracks.append({"id": tr.id, "truth": j, "color_ok": tr.color == self.cubes[j].color,
                           "err_xy": float(np.linalg.norm(tr.xy - pos[:2])),
                           "err_yaw": abs(yaw_diff(tr.yaw, yaw)), "age": sim.time - tr.t_seen})
        self.frames.append({"t": round(sim.time, 3), "cubes": cubes, "tracks": tracks,
                            "dets": [{"color": d.color, "full": d.full, "frac": round(d.area_frac, 3)} for d in dets]})

    # ------------------------------------------------------------------ truy vấn
    def ids(self) -> list[int]:
        return [t.id for t in self.tracker.tracks]

    def lost(self, oid: int) -> bool:
        return self.tracker.get(oid).state == "lost"

    def pose(self, oid: int) -> tuple[np.ndarray, float]:
        tr = self.tracker.get(oid)
        return np.array([tr.xy[0], tr.xy[1], self.L.table_top + CUBE / 2]), tr.yaw

    def color(self, oid: int) -> str:
        return self.tracker.get(oid).color

    def set_busy(self, oid: int, busy: bool) -> None:
        tr = self.tracker.get(oid)
        if tr.state in ("on_table", "busy"):
            tr.state = "busy" if busy else "on_table"

    def set_done(self, oid: int) -> None:
        self.tracker.get(oid).state = "done"

    def _nearest(self, xy) -> int | None:
        best, bi = TRUTH_GATE, None
        for i in range(len(self.cubes)):
            pos, _ = self.sim.cube_pose(i)
            if abs(pos[2] - (self.L.table_top + CUBE / 2)) > 0.02:
                continue
            dd = float(np.linalg.norm(pos[:2] - xy))
            if dd < best:
                best, bi = dd, i
        return bi

    def truth(self, oid: int) -> int | None:
        """Chỉ để đánh giá."""
        return self._nearest(self.tracker.get(oid).xy)

    def check_cubes(self, target: int | None):
        """Khối cho kiểm tra đoạn đi xuống đặt theo bản ghi (không dùng pose thật): bản ghi trên bàn lần lượt lấy các thân
        khối của model kiểm tra, thân thừa đưa ra xa. Trả (qpos theo thân, chỉ số thân của đích)."""
        live = [t for t in self.tracker.tracks if t.state in ("on_table", "busy")]
        out, tgt = {}, None
        for b in range(len(self.cubes)):
            if b < len(live):
                tr = live[b]
                pos = np.array([tr.xy[0], tr.xy[1], self.L.table_top + CUBE / 2])
                quat = np.array([math.cos(tr.yaw / 2), 0, 0, math.sin(tr.yaw / 2)])
                if tr.id == target:
                    tgt = b
            else:
                pos, quat = FAR + [b, 0, 0], np.array([1.0, 0, 0, 0])
            out[self._cube_qadr[b]] = np.concatenate([pos, quat])
        return out, tgt

    def holding(self, k: int, oid: int) -> bool:
        q = self.sim.d.qpos[self._driver[k]]
        return bool(np.mean(q) < DRIVER_HELD)

    def close(self) -> None:
        self.cam.close()
