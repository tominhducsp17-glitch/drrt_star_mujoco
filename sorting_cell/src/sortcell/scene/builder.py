"""Dựng cell: 4 tay UR10e + Robotiq 2F-85, bàn, bệ, khay, khối, camera.

Theo quy ước của drrt.mujoco_cell: tên mọi thứ thuộc tay k có tiền tố `a{k}_`, khớp tay mang tên trong JOINTS, nên
drrt.mujoco_cell._qpos_indices và _arm_of dùng được trực tiếp.

Hai loại model:
- "exec": model thực thi cho mj_step, đúng như vật lý;
- "plan": như exec nhưng mỗi gripper mang thêm một khối ảo cỡ khối thật giữa hai má kẹp (vật có thể đang mang), và không
  có khối thật. Model này dùng cho roadmap và kiểm tra va chạm của planner.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import mujoco as mj
import numpy as np

from drrt.mujoco_cell import JOINTS, UR10E

ROOT = Path(__file__).resolve().parents[3]
GRIPPER_XML = ROOT / "third_party/robotiq_2f85/2f85.xml"
TCP = "tcp"
CUBE = 0.05
# Từ mặt bích tới tâm má kẹp khi kẹp khối 5 cm. Đo bằng experiments/measure_gripper.py: tâm má kẹp ở 0,142 m khi mở
# hẳn, 0,155 m khi đóng hẳn.
TCP_OFFSET = 0.150
ARM_ACTUATORS = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")
GRIP_ACT = "g_fingers_actuator"
COLORS = {"red": (0.85, 0.15, 0.15, 1), "green": (0.15, 0.7, 0.2, 1), "blue": (0.15, 0.3, 0.85, 1)}
COLOR_NAMES = tuple(COLORS)


@dataclass(frozen=True)
class Layout:
    """Bàn vuông ở tâm, mỗi cạnh một tay. Ba khay của mỗi tay ở phía sau đế. Kích thước chốt bằng
    experiments/reach_check.py (xem mục 12 của kế hoạch)."""

    base_dist: float = 0.85     # tâm bàn → đế tay
    table_half: float = 0.65    # bàn 1,30 × 1,30 m
    table_top: float = 0.75
    base_height: float = 0.75   # đế tay ngang mặt bàn
    pedestal_half: float = 0.15
    tray_half: float = 0.15     # khay 0,30 × 0,30 m
    tray_wall: float = 0.06
    tray_top: float = 0.60      # mặt đáy khay
    tray_radius: float = 0.55   # đế tay → tâm khay
    tray_angles_deg: tuple[float, float, float] = (115.0, 180.0, 245.0)  # 0° = hướng vào tâm bàn; thứ tự = COLOR_NAMES
    zone_half: float = 0.35     # vùng đặt vật 0,70 × 0,70 m ở giữa bàn
    n_arms: int = 4

    def base(self, k: int) -> tuple[float, float, float, float]:
        yaw = math.pi / 2 * k
        return (-self.base_dist * math.cos(yaw), -self.base_dist * math.sin(yaw), self.base_height, yaw)

    def tray_center(self, k: int, j: int) -> np.ndarray:
        x, y, _, yaw = self.base(k)
        t = yaw + math.radians(self.tray_angles_deg[j])
        return np.array([x + self.tray_radius * math.cos(t), y + self.tray_radius * math.sin(t), self.tray_top])

    def tray_centers(self, k: int) -> list[np.ndarray]:
        return [self.tray_center(k, j) for j in range(3)]

    def tray_slots(self, k: int, j: int) -> list[np.ndarray]:
        """Chỗ thả trong khay: tâm rồi 4 góc (cách tâm 6,5 cm), theo hệ của khay."""
        c = self.tray_center(k, j)
        yaw = self.base(k)[3]
        R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        offs = [(0, 0), (0.065, 0.065), (-0.065, 0.065), (-0.065, -0.065), (0.065, -0.065)]
        return [c + np.append(R @ np.array(o), 0.0) for o in offs]


@dataclass(frozen=True)
class GripperCfg:
    pad_friction: tuple[float, float, float] | None = None  # None: giữ của Menagerie
    force_limit: float | None = None                       # None: giữ của Menagerie (5)
    # Menagerie đặt cone=elliptic, impratio=10 trong <option> của 2f85.xml; attach KHÔNG mang theo option của model con
    # (MuJoCo giữ giá trị của model cha), nên phải đặt lại ở model cha. Thiếu nó khối trượt nhanh hơn ~18 lần (mục 12).
    cone_elliptic: bool = True
    impratio: float = 10.0


def _contact_options(spec: mj.MjSpec, cfg: GripperCfg) -> None:
    if cfg.cone_elliptic:
        spec.option.cone = mj.mjtCone.mjCONE_ELLIPTIC
        spec.option.impratio = cfg.impratio


def arm_spec(cfg: GripperCfg, phantom: bool, camera: bool, joint_limit: float = math.pi) -> mj.MjSpec:
    spec = mj.MjSpec.from_file(str(UR10E))
    _contact_options(spec, cfg)
    for name in JOINTS:
        spec.joint(name).range = [-joint_limit, joint_limit]
    for b in spec.bodies:
        b.gravcomp = 1.0  # bộ điều khiển UR bù trọng lực; vật được gắp vẫn chịu trọng lực
    g = mj.MjSpec.from_file(str(GRIPPER_XML))
    for geom in g.geoms:
        if geom.name.endswith(("pad1", "pad2")) and cfg.pad_friction is not None:
            geom.friction = list(cfg.pad_friction)
    if cfg.force_limit is not None:
        g.actuator("fingers_actuator").forcerange = [-cfg.force_limit, cfg.force_limit]
    for b in g.bodies:
        b.gravcomp = 1.0
    spec.site("attachment_site").attach_body(g.body("base_mount"), "g_", "")
    w3 = spec.body("wrist_3_link")
    tcp_y = 0.1 + TCP_OFFSET  # site mặt bích ở y = 0,1 trong wrist_3_link, trục gripper là +y của wrist_3_link
    w3.add_site(name=TCP, pos=[0, tcp_y, 0], quat=[-1, 1, 0, 0], size=[0.006, 0, 0], rgba=[1, 0, 0, 1], group=5)
    if phantom:
        w3.add_geom(name="held_phantom", type=mj.mjtGeom.mjGEOM_BOX, size=[CUBE / 2] * 3, pos=[0, tcp_y, 0],
                    quat=[-1, 1, 0, 0], rgba=[1, 0.6, 0, 0.3], mass=0, contype=2, conaffinity=0, group=3)
    if camera:
        _wrist_camera(w3, tcp_y)
    return spec


def _wrist_camera(w3, tcp_y: float, side: float = 0.085, back: float = 0.09, look: float = 0.10):
    """Camera cổ tay: cách trục gripper 9 cm theo phương vuông góc với mặt phẳng đóng mở của má kẹp (má kẹp mở theo x
    của TCP, camera lệch theo z của wrist_3_link = −y của TCP), nhìn về điểm nằm dưới TCP 10 cm theo trục gripper."""
    pos = np.array([0.0, back, side])
    fwd = np.array([0.0, tcp_y + look, 0.0]) - pos
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0.0, 1.0, 0.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    quat = np.zeros(4)
    mj.mju_mat2Quat(quat, np.column_stack([right, up, -fwd]).flatten())  # camera MuJoCo nhìn theo −z
    w3.add_camera(name="wrist_cam", pos=pos.tolist(), quat=quat.tolist(), fovy=60)


def base_spec(cfg: GripperCfg, offwidth: int = 1280, offheight: int = 960) -> mj.MjSpec:
    spec = mj.MjSpec()
    spec.option.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST  # attach giữ integrator của model cha
    _contact_options(spec, cfg)
    spec.visual.global_.offwidth = offwidth
    spec.visual.global_.offheight = offheight
    spec.visual.headlight.ambient = [0.25, 0.25, 0.25]
    spec.visual.headlight.diffuse = [0.3, 0.3, 0.3]
    w = spec.worldbody
    w.add_light(pos=[0, 0, 3.5], dir=[0, 0, -1], diffuse=[0.45, 0.45, 0.45], castshadow=False)
    w.add_geom(name="floor", type=mj.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], rgba=[0.92, 0.92, 0.9, 1])
    return spec


def add_environment(spec: mj.MjSpec, L: Layout) -> None:
    w = spec.worldbody
    h = L.table_top / 2
    w.add_geom(name="table", type=mj.mjtGeom.mjGEOM_BOX, size=[L.table_half, L.table_half, h], pos=[0, 0, h],
               rgba=[0.45, 0.44, 0.42, 1])
    w.add_geom(name="zone", type=mj.mjtGeom.mjGEOM_BOX, size=[L.zone_half, L.zone_half, 0.0005],
               pos=[0, 0, L.table_top + 0.0005], rgba=[0.55, 0.54, 0.52, 1], contype=0, conaffinity=0)
    for k in range(L.n_arms):
        x, y, z, yaw = L.base(k)
        w.add_geom(name=f"pedestal{k}", type=mj.mjtGeom.mjGEOM_BOX, size=[L.pedestal_half, L.pedestal_half, z / 2],
                   pos=[x, y, z / 2], rgba=[0.3, 0.3, 0.32, 1])
        for j in range(3):
            _tray(w, f"tray{k}_{j}", L.tray_center(k, j), yaw, L, COLORS[COLOR_NAMES[j]])


NO_FLY_MARGIN = 0.05   # vùng cấm rộng hơn vùng đặt vật mỗi bên (khối ở mép vùng, khối bị đẩy ra)
NO_FLY_TOP = 0.005     # nóc vùng cấm cao hơn mặt trên khối


def add_no_fly(spec: mj.MjSpec, L: Layout) -> None:
    """Chỉ trong model lập kế hoạch: hộp vô hình phủ vùng đặt vật, từ mặt bàn tới mặt trên khối + NO_FLY_TOP. Planner không
    biết vị trí từng khối trên bàn, nên cấm thân tay và vật đang mang đi thấp qua vùng này (cộng lề CLEARANCE của model
    lập kế hoạch). Gắp và thả là chuyển động cục bộ, không qua planner, nên không bị ảnh hưởng."""
    h = (CUBE + NO_FLY_TOP) / 2
    spec.worldbody.add_geom(name="no_fly", type=mj.mjtGeom.mjGEOM_BOX,
                            size=[L.zone_half + NO_FLY_MARGIN, L.zone_half + NO_FLY_MARGIN, h],
                            pos=[0, 0, L.table_top + h], rgba=[1, 0, 0, 0], group=3)


def _tray(w, name, c, yaw, L: Layout, rgba):
    hx = hy = L.tray_half
    t = 0.01
    b = w.add_body(name=name, pos=[c[0], c[1], 0], quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    stand = c[2] - 0.01
    b.add_geom(name=name + "_stand", type=mj.mjtGeom.mjGEOM_BOX, size=[hx, hy, stand / 2], pos=[0, 0, stand / 2],
               rgba=[0.5, 0.5, 0.52, 1])
    b.add_geom(name=name + "_floor", type=mj.mjtGeom.mjGEOM_BOX, size=[hx, hy, 0.005], pos=[0, 0, c[2] - 0.005],
               rgba=list(rgba))
    zc = c[2] + L.tray_wall / 2
    for s in (-1, 1):
        b.add_geom(name=f"{name}_wx{s}", type=mj.mjtGeom.mjGEOM_BOX, size=[t / 2, hy, L.tray_wall / 2],
                   pos=[s * (hx - t / 2), 0, zc], rgba=list(rgba))
        b.add_geom(name=f"{name}_wy{s}", type=mj.mjtGeom.mjGEOM_BOX, size=[hx, t / 2, L.tray_wall / 2],
                   pos=[0, s * (hy - t / 2), zc], rgba=list(rgba))


def add_cube(spec: mj.MjSpec, name: str, pos, yaw: float, rgba, mass: float = 0.1):
    b = spec.worldbody.add_body(name=name, pos=list(pos), quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    b.add_freejoint(name=name + "_free")
    b.add_geom(name=name + "_geom", type=mj.mjtGeom.mjGEOM_BOX, size=[CUBE / 2] * 3, rgba=list(rgba), mass=mass)
    return b


def overhead_height(L: Layout, fovy_deg: float = 60.0, margin: float = 0.05) -> float:
    """Camera nhìn thẳng xuống, đủ cao để chiều hẹp của ảnh (dọc, 480 px) chứa trọn bàn cộng lề."""
    return L.table_top + (L.table_half + margin) / math.tan(math.radians(fovy_deg) / 2)


def build_spec(L: Layout, kind: str = "exec", cfg: GripperCfg | None = None, arms=None, environment: bool = True,
               cubes=(), cameras: bool = True) -> mj.MjSpec:
    """cubes: danh sách (tên, (x, y, z), yaw, màu). Chỉ dùng cho kind="exec"."""
    if kind not in ("exec", "plan"):
        raise ValueError(kind)
    cfg = cfg or GripperCfg()
    spec = base_spec(cfg)
    if environment:
        add_environment(spec, L)
        if kind == "plan":
            add_no_fly(spec, L)
    for k in (range(L.n_arms) if arms is None else arms):
        x, y, z, yaw = L.base(k)
        frame = spec.worldbody.add_frame(pos=[x, y, z], quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        spec.attach(arm_spec(cfg, phantom=kind == "plan", camera=cameras), frame=frame, prefix=f"a{k}_")
    if kind == "exec":
        for name, pos, yaw, color in cubes:
            add_cube(spec, name, pos, yaw, COLORS[color])
    if cameras:
        spec.worldbody.add_camera(name="overhead", pos=[0, 0, overhead_height(L)], quat=[1, 0, 0, 0], fovy=60)
    return spec


def arm_of_body(name: str) -> int:
    """Tay sở hữu body (theo tiền tố a{k}_), -1 nếu không thuộc tay nào."""
    if name.startswith("a") and "_" in name and name[1:name.index("_")].isdigit():
        return int(name[1:name.index("_")])
    return -1


def arm_of_geom(m: mj.MjModel, g: int) -> int:
    return arm_of_body(mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "")
