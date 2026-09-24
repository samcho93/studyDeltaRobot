"""High level robot API — the same student code runs on every backend.

    from deltarobot import DeltaRobot
    robot = DeltaRobot("edu_dynamixel")          # preset | DeltaDesign | dict | "design.json"
    robot.home()
    robot.move_to(0.05, 0.0, -0.30)               # TCP position [m], straight line
    robot.arch_to(0.10, 0.05, -0.33, height=0.03) # pick-and-place door motion
    robot.tool_on(); robot.wait(0.2); robot.tool_off()

Backends: "record" (Playground / dry run), "websim" (PC -> web simulator), "serial"
(Arduino firmware), "ros2" (delta_robot driver), "auto" (record).
"""

from __future__ import annotations

import math
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

from . import dynamics as dyn
from . import kinematics as kin
from . import trajectory as traj
from .design import DeltaDesign
from .scene import Scene

Vec = Tuple[float, float, float]
DT = 0.01   # command stream period [s]


class WorkspaceError(ValueError):
    """Target (or a point on the path) is outside the reachable workspace."""


class EmergencyStop(RuntimeError):
    """The simulator or driver requested an emergency stop."""


def _make_design(design: Union[str, DeltaDesign, Dict[str, Any], None]) -> DeltaDesign:
    if design is None:
        return DeltaDesign()
    if isinstance(design, DeltaDesign):
        return design
    if isinstance(design, dict):
        return DeltaDesign.from_dict(design.get("design", design))
    if isinstance(design, str) and design.endswith(".json"):
        return DeltaDesign.load(design)
    return DeltaDesign.preset(design)


def _make_backend(name: Union[str, Any], **kw: Any):
    if not isinstance(name, str):
        return name
    if name == "auto":
        name = "record"
    if name == "record":
        from .backends.record import RecordBackend
        return RecordBackend(**kw)
    if name == "websim":
        from .backends.websim import WebSimBackend
        return WebSimBackend(**kw)
    if name == "serial":
        from .backends.serial_backend import SerialBackend
        return SerialBackend(**kw)
    if name == "ros2":
        from .backends.ros2 import Ros2Backend
        return Ros2Backend(**kw)
    raise ValueError("unknown backend %r (record | websim | serial | ros2)" % name)


class DeltaRobot:
    """Delta robot controller: plans trajectories, checks limits, streams joint angles."""

    def __init__(self, design: Union[str, DeltaDesign, Dict[str, Any], None] = None,
                 backend: Union[str, Any] = "auto", scene: Optional[Dict[str, Any]] = None,
                 **backend_options: Any):
        self.backend = _make_backend(backend, **backend_options)
        hello = self.backend.open(self) or {}
        # a connected simulator may dictate the design / scene it is showing
        from_hello = design is None and bool(hello.get("design"))
        if from_hello:
            design = hello["design"]
        self.design = _make_design(design)
        problems = self.design.validate()
        hard = [p for p in problems if "무시" not in p and "짧으면" not in p]
        if hard:
            raise ValueError("설계 오류: " + "; ".join(hard))
        hello_scene = hello.get("scene")
        if scene is None and hello_scene:
            same = from_hello or (hello.get("design") or {}) == self.design.to_dict()
            # the simulator's scene is sized for its own design; rebuild it for another design
            scene = hello_scene if same else hello_scene.get("kind", "pick_place")
        if scene is None:
            from .scene import default_scene
            scene = default_scene(self.design, "pick_place")
        elif isinstance(scene, str):
            from .scene import default_scene
            scene = default_scene(self.design, scene)
        self.scene = Scene(scene)
        self.speed = 0.25 * (self.design.upper_arm + self.design.forearm) / 0.43   # m/s
        self.accel = 10.0 * self.speed                                              # m/s^2
        self.profile = "scurve"
        self.joint_speed = 3.0      # rad/s  (move_joints)
        self.joint_accel = 30.0     # rad/s^2
        self.tool_state = 0
        self.q: Vec = (self.design.home_theta,) * 3   # type: ignore[assignment]
        self.backend.start(self)

    # ------------------------------------------------------------ properties
    @property
    def time(self) -> float:
        return self.backend.now()

    @property
    def joints(self) -> Vec:
        return tuple(self.q)  # type: ignore[return-value]

    @property
    def position(self) -> Vec:
        """Current TCP position (tool tip) [m]."""
        return self.fk(*self.q)

    @property
    def effector(self) -> Vec:
        return kin.fk(self.design, self.q)

    # ------------------------------------------------------------ kinematics (TCP)
    def _eff(self, x: float, y: float, z: float) -> Vec:
        return (x, y, z + self.design.tool_length)

    def ik(self, x: float, y: float, z: float) -> Vec:
        """TCP position -> motor angles [rad] (raises WorkspaceError)."""
        rep = kin.limit_report(self.design, self._eff(x, y, z))
        if not rep["ok"]:
            raise WorkspaceError(self._explain((x, y, z), rep["problems"]))
        return rep["theta"]  # type: ignore[return-value]

    def fk(self, t1: float, t2: float, t3: float) -> Vec:
        p = kin.fk(self.design, (t1, t2, t3))
        return (p[0], p[1], p[2] - self.design.tool_length)

    def reachable(self, x: float, y: float, z: float) -> bool:
        return kin.reachable(self.design, self._eff(x, y, z))

    def _explain(self, p: Sequence[float], problems: List[str]) -> str:
        names = {"unreachable": "팔 길이로 닿지 않음", "theta": "모터 각도 한계", "ball": "볼조인트 각도 한계",
                 "elbow": "팔꿈치 각도 한계"}
        why = sorted({names.get(pr.rstrip("123"), pr) for pr in problems})
        return "작업영역 밖: (%.3f, %.3f, %.3f) m — %s" % (p[0], p[1], p[2], ", ".join(why))

    # ------------------------------------------------------------ settings
    def set_speed(self, speed: Optional[float] = None, accel: Optional[float] = None) -> None:
        """Cartesian speed [m/s] and acceleration [m/s^2] for later moves."""
        if speed is not None:
            if speed <= 0:
                raise ValueError("speed must be > 0")
            self.speed = float(speed)
        if accel is not None:
            if accel <= 0:
                raise ValueError("accel must be > 0")
            self.accel = float(accel)

    def set_profile(self, name: str) -> None:
        if name not in traj.PROFILES:
            raise ValueError("profile: %s" % ", ".join(traj.PROFILES))
        self.profile = name

    # ------------------------------------------------------------ motion
    def _run_cartesian(self, path: traj.Path, speed: Optional[float], accel: Optional[float],
                       profile: Optional[str]) -> None:
        self.backend.check()
        samples = traj.sample_path(path, profile or self.profile, speed or self.speed,
                                   accel or self.accel, DT, self.time)
        stream: List[Tuple[float, Vec]] = []
        for t, pe in samples:
            rep = kin.limit_report(self.design, pe)
            if not rep["ok"]:
                tcp = (pe[0], pe[1], pe[2] - self.design.tool_length)
                raise WorkspaceError("경로 중간 " + self._explain(tcp, rep["problems"]))
            stream.append((t, rep["theta"]))  # type: ignore[arg-type]
        self._send(stream)

    def _send(self, stream: List[Tuple[float, Vec]]) -> None:
        if not stream:
            return
        self.backend.check_stream(stream)
        self.backend.stream(stream, self.tool_state)
        self.q = tuple(stream[-1][1])  # type: ignore[assignment]

    def move_to(self, x: float, y: float, z: float, speed: Optional[float] = None,
                accel: Optional[float] = None, profile: Optional[str] = None) -> None:
        """Straight-line TCP move to (x, y, z) [m]."""
        self.ik(x, y, z)
        self._run_cartesian(traj.line_path(self.effector, self._eff(x, y, z)), speed, accel, profile)

    def move_path(self, points: Sequence[Sequence[float]], speed: Optional[float] = None,
                  accel: Optional[float] = None, profile: Optional[str] = None) -> None:
        """Polyline through TCP points with ONE speed profile (no stop at the corners)."""
        path = traj.Path()
        prev = self.effector
        for pt in points:
            self.ik(*pt)
            e = self._eff(*pt)
            path.line(prev, e)
            prev = e
        self._run_cartesian(path, speed, accel, profile)

    def move_by(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, **kw: Any) -> None:
        x, y, z = self.position
        self.move_to(x + dx, y + dy, z + dz, **kw)

    def arch_to(self, x: float, y: float, z: float, height: float = 0.03, radius: float = 0.0,
                speed: Optional[float] = None, accel: Optional[float] = None,
                profile: Optional[str] = None) -> None:
        """Pick-and-place motion: rise `height` above the higher end, travel, descend."""
        self.ik(x, y, z)
        path = traj.arch_path(self.effector, self._eff(x, y, z), height, radius)
        self._run_cartesian(path, speed, accel, profile)

    def move_joints(self, t1: float, t2: float, t3: float, degrees: bool = False,
                    speed: Optional[float] = None, accel: Optional[float] = None,
                    profile: Optional[str] = None) -> None:
        """Joint-space move (all motors start and stop together)."""
        q1 = tuple(math.radians(v) for v in (t1, t2, t3)) if degrees else (t1, t2, t3)
        for i, v in enumerate(q1):
            if not self.design.theta_min <= v <= self.design.theta_max:
                raise WorkspaceError("모터 %d 각도 %.1f° 가 한계 [%.1f°, %.1f°] 밖입니다" % (
                    i + 1, math.degrees(v), math.degrees(self.design.theta_min), math.degrees(self.design.theta_max)))
        try:
            kin.fk(self.design, q1)
        except kin.Unreachable as e:
            raise WorkspaceError("이 관절각 조합은 조립될 수 없습니다: %s" % e) from None
        self.backend.check()
        stream = traj.sample_joint(self.q, q1, profile or self.profile, speed or self.joint_speed,
                                   accel or self.joint_accel, DT, self.time)
        self._send(stream)

    def move_joint_to(self, x: float, y: float, z: float, **kw: Any) -> None:
        """Move to a TCP position with joint interpolation (curved path, usually faster)."""
        q = self.ik(x, y, z)
        self.move_joints(*q, **kw)

    def home(self) -> None:
        h = self.design.home_theta
        self.move_joints(h, h, h)

    def wait(self, seconds: float) -> None:
        self.backend.check()
        self.backend.wait(max(0.0, float(seconds)))

    # ------------------------------------------------------------ tool
    def tool_on(self) -> Optional[str]:
        """Suction on / gripper close / magnet on. Returns the grabbed part id (sim)."""
        self.backend.check()
        self.tool_state = 1
        got = self.scene.grab(self.position, self.time, self.design.tool_spec)
        self.backend.set_tool(1)
        return got

    def tool_off(self) -> Optional[str]:
        self.backend.check()
        self.tool_state = 0
        got = self.scene.release(self.position, self.time)
        self.backend.set_tool(0)
        return got

    suction_on = grip = magnet_on = pen_down = tool_on
    suction_off = release = magnet_off = pen_up = tool_off

    @property
    def holding(self) -> Optional[str]:
        return self.scene.held["id"] if self.scene.held else None

    # ------------------------------------------------------------ scene shortcuts
    def parts(self) -> List[Dict[str, Any]]:
        """Parts visible right now (conveyor parts move with time)."""
        return self.scene.parts(self.time)

    def bins(self) -> List[Dict[str, Any]]:
        return self.scene.bins()

    # ------------------------------------------------------------ analysis
    def analyze(self, safety_factor: float = 1.2) -> Dict[str, Any]:
        """Motor check of everything recorded so far (record backend only)."""
        frames = getattr(self.backend, "frames", None)
        if not frames:
            raise RuntimeError("analyze()는 record 백엔드에서 기록된 동작에만 쓸 수 있습니다")
        data = dyn.analyze(self.design, frames)
        return dyn.evaluate(self.design, data, safety_factor)

    def close(self) -> None:
        self.backend.close()

    def __enter__(self) -> "DeltaRobot":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return "DeltaRobot(motor=%s, tool=%s, backend=%s)" % (
            self.design.motor, self.design.tool, type(self.backend).__name__)


def in_pyodide() -> bool:
    return "pyodide" in sys.modules
