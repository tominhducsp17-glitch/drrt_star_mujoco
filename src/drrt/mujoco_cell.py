"""Cell nhiều tay UR10e trong MuJoCo và các checker va chạm cho planner.

Va chạm được tách đúng như bài: roadmap của từng tay chỉ xét tay đó với môi trường (kể cả tự va chạm), còn online chỉ
xét giữa các tay. Mỗi loại dùng một model riêng, dựng từ cùng một mô tả cell:

- model offline của tay i: chỉ có tay i và môi trường;
- model online: mọi tay, môi trường tắt va chạm, tay i chỉ va chạm với tay khác nhờ mặt nạ bit contype/conaffinity;
- model thực thi: đầy đủ, không có lề an toàn, dùng cho mj_step.

Lề an toàn (clearance) được cài bằng geom margin. MuJoCo **cộng** margin của hai geom trong cặp (đã kiểm chứng), nên
offline đặt toàn bộ lề trên geom môi trường, còn online chia đôi cho geom của mỗi tay.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import mujoco as mj
import numpy as np

UR10E = Path(__file__).resolve().parents[2] / "third_party/universal_robots_ur10e/ur10e.xml"
JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
TIP = "tool_tip"
DOF = 6


@dataclass
class Box:
    name: str
    pos: tuple[float, float, float]
    half: tuple[float, float, float]
    rgba: tuple[float, float, float, float] = (0.55, 0.45, 0.35, 1)


@dataclass
class Cylinder:
    name: str
    pos: tuple[float, float, float]
    radius: float
    half_height: float
    rgba: tuple[float, float, float, float] = (0.85, 0.35, 0.2, 1)


@dataclass
class CellSpec:
    """Vị trí đế (x, y, z, yaw) của từng tay và vật cản môi trường."""

    bases: Sequence[tuple[float, float, float, float]]
    obstacles: Sequence[Box | Cylinder] = ()
    tool_length: float = 0.15
    tool_radius: float = 0.035
    joint_limit: float = math.pi

    @property
    def n_arms(self) -> int:
        return len(self.bases)


def two_arm_cell(spacing: float = 1.5, pole: bool = True) -> CellSpec:
    """Hai tay đối diện qua bàn gá, mỗi tay trên một bệ cao 0,6 m. Tuỳ chọn một cột giữa bàn, ý tưởng từ Hình 19."""
    table_top = 0.75
    obstacles: list[Box | Cylinder] = [
        Box("table", (0.0, 0.0, table_top / 2), (0.35, 0.6, table_top / 2)),
    ]
    for k, x in enumerate((-spacing / 2, spacing / 2)):
        obstacles.append(Box(f"pedestal{k}", (x, 0.0, 0.3), (0.15, 0.15, 0.3), (0.3, 0.3, 0.32, 1)))
    if pole:
        obstacles.append(Cylinder("pole", (0.0, 0.0, table_top + 0.3), 0.04, 0.3))
    return CellSpec(bases=[(-spacing / 2, 0.0, 0.6, 0.0), (spacing / 2, 0.0, 0.6, math.pi)], obstacles=obstacles)


def four_arm_cell(spacing: float = 1.5) -> CellSpec:
    """Bốn tay quanh một bàn vuông, mỗi cạnh một tay."""
    obstacles: list[Box | Cylinder] = [Box("table", (0.0, 0.0, 0.375), (0.35, 0.35, 0.375))]
    bases = []
    for k in range(4):
        yaw = math.pi / 2 * k
        x, y = -spacing / 2 * math.cos(yaw), -spacing / 2 * math.sin(yaw)
        bases.append((x, y, 0.6, yaw))
        obstacles.append(Box(f"pedestal{k}", (x, y, 0.3), (0.15, 0.15, 0.3), (0.3, 0.3, 0.32, 1)))
    return CellSpec(bases=bases, obstacles=obstacles)


def _arm_spec(cell: CellSpec) -> mj.MjSpec:
    """Một tay UR10e kèm dụng cụ hình trụ ở mặt bích và site ở đầu dụng cụ."""
    spec = mj.MjSpec.from_file(str(UR10E))
    for name in JOINTS:
        spec.joint(name).range = [-cell.joint_limit, cell.joint_limit]
    flange = spec.body("wrist_3_link")
    tool = flange.add_body(name="tool", pos=[0, 0.1, 0], quat=[-1, 1, 0, 0])  # trục z của tool = trục mặt bích
    tool.add_geom(name="tool_geom", type=mj.mjtGeom.mjGEOM_CYLINDER, size=[cell.tool_radius, cell.tool_length / 2, 0],
                  pos=[0, 0, cell.tool_length / 2], rgba=[0.9, 0.75, 0.2, 1], mass=0.3)
    tool.add_site(name=TIP, pos=[0, 0, cell.tool_length], size=[0.01, 0, 0])
    return spec


def build(cell: CellSpec, arms: Sequence[int] | None = None, environment: bool = True) -> mj.MjSpec:
    """Ghép các tay được chọn (mặc định tất cả) và môi trường. Tên của tay k có tiền tố `a{k}_`."""
    arms = range(cell.n_arms) if arms is None else arms
    spec = mj.MjSpec()
    spec.option.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST  # attach giữ integrator của model cha
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 720
    world = spec.worldbody
    world.add_light(pos=[0, 0, 3.5], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
    world.add_geom(name="floor", type=mj.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], rgba=[0.92, 0.92, 0.9, 1])
    if environment:
        for ob in cell.obstacles:
            if isinstance(ob, Box):
                world.add_geom(name=ob.name, type=mj.mjtGeom.mjGEOM_BOX, size=list(ob.half), pos=list(ob.pos),
                               rgba=list(ob.rgba))
            else:
                world.add_geom(name=ob.name, type=mj.mjtGeom.mjGEOM_CYLINDER, size=[ob.radius, ob.half_height, 0],
                               pos=list(ob.pos), rgba=list(ob.rgba))
    for k in arms:
        x, y, z, yaw = cell.bases[k]
        frame = world.add_frame(pos=[x, y, z], quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        spec.attach(_arm_spec(cell), frame=frame, prefix=f"a{k}_")
    return spec


def _arm_of(model: mj.MjModel, geom: int) -> int:
    name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom]) or ""
    return int(name[1:name.index("_")]) if name.startswith("a") and "_" in name and name[1].isdigit() else -1


def _qpos_indices(model: mj.MjModel, arm: int) -> np.ndarray:
    return np.array([model.jnt_qposadr[model.joint(f"a{arm}_{j}").id] for j in JOINTS])


def offline_model(cell: CellSpec, arm: int, clearance: float) -> mj.MjModel:
    """Tay `arm` với môi trường. Lề an toàn chỉ đặt trên geom môi trường nên không ảnh hưởng tự va chạm."""
    m = build(cell, [arm]).compile()
    for g in range(m.ngeom):
        if m.geom_contype[g] or m.geom_conaffinity[g]:
            m.geom_margin[g] = clearance if _arm_of(m, g) < 0 else 0.0
    return m


def online_model(cell: CellSpec, clearance: float) -> mj.MjModel:
    """Mọi tay, chỉ va chạm giữa các tay khác nhau. Mỗi geom của tay mang nửa lề an toàn."""
    m = build(cell, environment=False).compile()
    everyone = (1 << cell.n_arms) - 1
    for g in range(m.ngeom):
        if not (m.geom_contype[g] or m.geom_conaffinity[g]):
            continue
        arm = _arm_of(m, g)
        if arm < 0:
            m.geom_contype[g] = m.geom_conaffinity[g] = 0  # sàn
            continue
        m.geom_contype[g] = 1 << arm
        m.geom_conaffinity[g] = everyone & ~(1 << arm)
        m.geom_margin[g] = clearance / 2
    return m


def execution_model(cell: CellSpec) -> mj.MjModel:
    return build(cell).compile()


def steps_between(a: np.ndarray, b: np.ndarray, resolution: float) -> int:
    return max(1, math.ceil(float(np.max(np.abs(b - a))) / resolution))


class ArmSpace:
    """RobotSpace của một tay (roadmap.RobotSpace): giới hạn khớp, va chạm với môi trường và tự va chạm."""

    def __init__(self, cell: CellSpec, arm: int, clearance: float = 0.02, resolution: float = 0.05):
        self.cell = cell
        self.arm = arm
        self.model = offline_model(cell, arm, clearance)
        self.data = mj.MjData(self.model)
        self.idx = _qpos_indices(self.model, arm)
        self.tip = self.model.site(f"a{arm}_{TIP}").id
        self.lower = np.full(DOF, -cell.joint_limit)
        self.upper = np.full(DOF, cell.joint_limit)
        self.resolution = resolution
        self.checks = 0

    def config_free(self, q: Sequence[float]) -> bool:
        self.checks += 1
        d = self.data
        d.qpos[self.idx] = q
        mj.mj_kinematics(self.model, d)
        mj.mj_collision(self.model, d)
        return d.ncon == 0

    def edge_free(self, a: Sequence[float], b: Sequence[float]) -> bool:
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = steps_between(a, b, self.resolution)
        # Kiểm tra theo thứ tự chia đôi để gặp va chạm sớm hơn duyệt tuần tự.
        for k in _bisection_order(n):
            if not self.config_free(a + (b - a) * (k / n)):
                return False
        return True

    def tip_pose(self, q: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        d = self.data
        d.qpos[self.idx] = q
        mj.mj_kinematics(self.model, d)
        return d.site_xpos[self.tip].copy(), d.site_xmat[self.tip].reshape(3, 3).copy()

    def free_fraction(self, samples: int, rng: np.random.Generator) -> float:
        q = rng.uniform(self.lower, self.upper, (samples, DOF))
        return float(np.mean([self.config_free(row) for row in q]))


def _bisection_order(n: int) -> list[int]:
    """0..n theo thứ tự đầu mút trước, rồi trung điểm, rồi các phần tư..."""
    order, seen = [0, n], {0, n}
    width = n
    while width > 1:
        half = width // 2 if width % 2 == 0 else (width + 1) // 2
        for start in range(0, n, width):
            k = start + half
            if k < n and k not in seen:
                seen.add(k)
                order.append(k)
        width = half
    order.extend(k for k in range(n + 1) if k not in seen)
    return order


class CellPairChecker:
    """PairwiseChecker của tensor_graph: va chạm giữa các tay khi cùng nội suy thẳng trong không gian khớp."""

    def __init__(self, cell: CellSpec, clearance: float = 0.02, resolution: float = 0.05):
        self.model = online_model(cell, clearance)
        self.data = mj.MjData(self.model)
        self.idx = np.concatenate([_qpos_indices(self.model, k) for k in range(cell.n_arms)])
        self.resolution = resolution
        self.checks = 0

    def config_free(self, q: np.ndarray) -> bool:
        self.checks += 1
        d = self.data
        d.qpos[self.idx] = q
        mj.mj_kinematics(self.model, d)
        mj.mj_collision(self.model, d)
        return d.ncon == 0

    def edge_free(self, a, b) -> bool:
        a = np.concatenate([np.asarray(x, float) for x in a])
        b = np.concatenate([np.asarray(x, float) for x in b])
        n = steps_between(a, b, self.resolution)
        return all(self.config_free(a + (b - a) * (k / n)) for k in _bisection_order(n))


@dataclass
class FullChecker:
    """Kiểm tra trên model đầy đủ (mọi tay, môi trường, cùng lề an toàn), dùng để xác nhận phép tách ở trên là đúng."""

    cell: CellSpec
    clearance: float = 0.02
    model: mj.MjModel = field(init=False)

    def __post_init__(self) -> None:
        self.model = build(self.cell).compile()
        for g in range(self.model.ngeom):
            if self.model.geom_contype[g] or self.model.geom_conaffinity[g]:
                self.model.geom_margin[g] = self.clearance  # tổng lề 2c đủ rộng; lọc lại theo dist bên dưới
        self.data = mj.MjData(self.model)
        self.idx = np.concatenate([_qpos_indices(self.model, k) for k in range(self.cell.n_arms)])

    def contacts(self, q: np.ndarray) -> list[tuple[int, int]]:
        """(tay của geom1, tay của geom2), -1 là môi trường. Chỉ trả các cặp thật sự nằm trong lề an toàn tương ứng."""
        d = self.data
        d.qpos[self.idx] = q
        mj.mj_kinematics(self.model, d)
        mj.mj_collision(self.model, d)
        out = []
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = _arm_of(self.model, c.geom1), _arm_of(self.model, c.geom2)
            # Tự va chạm dùng lề 0; môi trường và giữa các tay dùng lề clearance.
            if c.dist >= (0.0 if a == b and a >= 0 else self.clearance):
                continue
            out.append((a, b))
        return out
