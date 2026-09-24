"""Delta robot design parameters and catalog lookup.

A :class:`DeltaDesign` holds everything the kinematics, dynamics, URDF generator and
simulator need.  Part specs (motors, gearboxes, materials, tools) come from
``data/catalog.json`` — the same file the web simulator fetches.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Any, Dict, List, Optional

_CATALOG: Optional[Dict[str, Any]] = None
CATALOG_PATH = Path(__file__).resolve().parent / "data" / "catalog.json"

# fittings (kg) — shared with assets/js/delta/design.js
HUB_MASS = 0.02          # upper-arm hub clamped on the motor shaft
ELBOW_MASS = 0.01        # elbow cross-bar fittings (plus spacing * forearm density)
BALL_CUP_MASS = 0.004    # one ball-joint cup
G = 9.81


def catalog() -> Dict[str, Any]:
    """Return the parsed part catalog (cached)."""
    global _CATALOG
    if _CATALOG is None:
        _CATALOG = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))
    return _CATALOG


@dataclass
class DeltaDesign:
    base_radius: float = 0.1          # R : base centre -> motor axis [m]
    effector_radius: float = 0.035    # r : effector centre -> ball-joint pair [m]
    upper_arm: float = 0.13           # L [m]
    forearm: float = 0.3              # l : parallelogram rod length [m]
    forearm_spacing: float = 0.05     # w : distance between the two rods [m]
    theta_min: float = -0.7           # motor angle limits [rad], 0 = horizontal, + = down
    theta_max: float = 1.5
    ball_joint_limit: float = 0.6     # max rod swing out of the arm plane [rad]
    upper_arm_material: str = "al_tube"
    forearm_material: str = "cfrp_tube"
    motor: str = "dxl_xm430_w350"
    gearbox: str = "none"
    gear_ratio: float = 1.0
    tool: str = "gripper_servo"
    payload: float = 0.2              # [kg]
    effector_mass: float = 0.06       # bare moving plate [kg]

    # ------------------------------------------------------------ construction
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "DeltaDesign":
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in names})

    @classmethod
    def preset(cls, name: str) -> "DeltaDesign":
        presets = catalog()["presets"]
        if name not in presets:
            raise KeyError("unknown preset %r (choose from %s)" % (name, ", ".join(presets)))
        return cls.from_dict(presets[name]["design"])

    @classmethod
    def load(cls, path: str) -> "DeltaDesign":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(data.get("design", data))

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def copy(self, **changes: Any) -> "DeltaDesign":
        d = self.to_dict()
        d.update(changes)
        return DeltaDesign.from_dict(d)

    # ------------------------------------------------------------ short names
    @property
    def R(self) -> float:
        return self.base_radius

    @property
    def r(self) -> float:
        return self.effector_radius

    @property
    def L(self) -> float:
        return self.upper_arm

    @property
    def l(self) -> float:  # noqa: E743 - standard symbol in delta literature
        return self.forearm

    # ------------------------------------------------------------ catalog specs
    @property
    def motor_spec(self) -> Dict[str, Any]:
        return catalog()["motors"][self.motor]

    @property
    def gearbox_spec(self) -> Dict[str, Any]:
        return catalog()["gearboxes"][self.gearbox]

    @property
    def tool_spec(self) -> Dict[str, Any]:
        return catalog()["tools"][self.tool]

    @property
    def ratio(self) -> float:
        """Effective reduction (integrated servos have none outside the case)."""
        return 1.0 if self.motor_spec.get("integrated_gear") else float(self.gear_ratio)

    @property
    def efficiency(self) -> float:
        return 1.0 if self.motor_spec.get("integrated_gear") else float(self.gearbox_spec["efficiency"])

    @property
    def tool_length(self) -> float:
        return float(self.tool_spec["length"])

    # ------------------------------------------------------------ masses
    def _density(self, material: str) -> float:
        return float(catalog()["materials"][material]["linear_density"])

    @property
    def upper_arm_mass(self) -> float:
        """Tube + hub, centre of mass taken at L/2."""
        return self.upper_arm * self._density(self.upper_arm_material) + HUB_MASS

    @property
    def elbow_mass(self) -> float:
        """Cross-bar at the elbow (lumped at distance L)."""
        return self.forearm_spacing * self._density(self.forearm_material) + ELBOW_MASS

    @property
    def forearm_pair_mass(self) -> float:
        """Two rods plus four ball cups."""
        return 2.0 * self.forearm * self._density(self.forearm_material) + 4 * BALL_CUP_MASS

    @property
    def moving_plate_mass(self) -> float:
        """Plate + tool + payload + half of every forearm pair (Codourey-style lumping)."""
        return (self.effector_mass + float(self.tool_spec["mass"]) + self.payload
                + 3 * 0.5 * self.forearm_pair_mass)

    @property
    def arm_inertia(self) -> float:
        """Inertia about the motor axis seen at the joint (output side) [kg m^2]."""
        L = self.upper_arm
        link = (self.upper_arm_mass * L * L / 3.0
                + (self.elbow_mass + 0.5 * self.forearm_pair_mass) * L * L)
        return link + self.drive_inertia

    @property
    def drive_inertia(self) -> float:
        """Rotor + gearbox inertia reflected to the output shaft (N^2 J)."""
        m = self.motor_spec
        if m.get("integrated_gear"):
            return float(m.get("output_inertia", 0.0))
        n = self.ratio
        return (float(m["rotor_inertia"]) + float(self.gearbox_spec["inertia"])) * n * n

    @property
    def gravity_moment(self) -> float:
        """k in tau_gravity = k * g * cos(theta) [kg m]."""
        L = self.upper_arm
        return self.upper_arm_mass * L / 2.0 + (self.elbow_mass + 0.5 * self.forearm_pair_mass) * L

    @property
    def home_theta(self) -> float:
        """Motor angle used for the home pose (all three arms equal)."""
        return max(self.theta_min, min(self.theta_max, 0.35))

    # ------------------------------------------------------------ validation
    def validate(self) -> List[str]:
        """Return a list of human readable problems (empty = ok)."""
        cat = catalog()
        errs: List[str] = []
        for name in ("base_radius", "effector_radius", "upper_arm", "forearm", "forearm_spacing"):
            if not getattr(self, name) > 0:
                errs.append("%s 는 0보다 커야 합니다" % name)
        if self.motor not in cat["motors"]:
            errs.append("알 수 없는 모터: %s" % self.motor)
        if self.gearbox not in cat["gearboxes"]:
            errs.append("알 수 없는 감속기: %s" % self.gearbox)
        if self.tool not in cat["tools"]:
            errs.append("알 수 없는 툴: %s" % self.tool)
        for mat in (self.upper_arm_material, self.forearm_material):
            if mat not in cat["materials"]:
                errs.append("알 수 없는 재질: %s" % mat)
        if self.theta_min >= self.theta_max:
            errs.append("theta_min < theta_max 이어야 합니다")
        if self.forearm <= self.upper_arm:
            errs.append("forearm(l)이 upper_arm(L)보다 짧으면 작업영역이 매우 좁아집니다 (보통 l ≈ 2~2.5 L)")
        if self.effector_radius >= self.base_radius:
            errs.append("effector_radius(r)는 base_radius(R)보다 작아야 합니다")
        if self.payload < 0:
            errs.append("payload는 음수가 될 수 없습니다")
        if not errs and self.motor_spec.get("integrated_gear") and self.gearbox != "none":
            errs.append("일체형 서보(%s)에는 외부 감속기를 달지 않습니다 — gearbox는 무시됩니다" % self.motor)
        if not errs and not self.motor_spec.get("integrated_gear"):
            ratios = self.gearbox_spec["ratios"]
            if self.gear_ratio not in ratios:
                errs.append("%s 의 감속비는 %s 중에서 고르세요" % (self.gearbox, ratios))
        return errs


def summary(design: DeltaDesign) -> Dict[str, float]:
    """Derived numbers shown in the simulator's design tab."""
    return {
        "upper_arm_mass": design.upper_arm_mass,
        "elbow_mass": design.elbow_mass,
        "forearm_pair_mass": design.forearm_pair_mass,
        "moving_plate_mass": design.moving_plate_mass,
        "arm_inertia": design.arm_inertia,
        "drive_inertia": design.drive_inertia,
        "gravity_moment": design.gravity_moment,
        "ratio": design.ratio,
        "efficiency": design.efficiency,
        "static_torque_horizontal": design.gravity_moment * G,
    }


def deg(x: float) -> float:
    return math.degrees(x)
