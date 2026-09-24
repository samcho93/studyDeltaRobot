"""ROS-independent logic shared by the delta_robot nodes.

Everything here is plain Python on top of the ``deltarobot`` library so it can be unit
tested without a ROS 2 installation (tests/test_ros2_core.py).  Do NOT import rclpy here.
"""

from __future__ import annotations

import json
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from deltarobot import kinematics as kin
from deltarobot import trajectory as traj
from deltarobot import urdf as durdf
from deltarobot.backends.base import max_joint_speed
from deltarobot.design import DeltaDesign

Vec = Tuple[float, float, float]
JOINTS: Tuple[str, str, str] = ("motor1_joint", "motor2_joint", "motor3_joint")
DT = 0.01               # commander stream period [s] (same as deltarobot.robot.DT)
SERIAL_MAX_RATE = 50.0  # the Arduino firmware must not be flooded [Hz]
STATE_MAX_RATE = 50.0   # web bridge state messages [Hz]


class PlanError(ValueError):
    """A goal (or a sample on the path) violates a workspace / joint / speed limit."""


# ---------------------------------------------------------------- design helpers
def load_design(preset: str = "edu_dynamixel", design_file: str = "") -> DeltaDesign:
    """Design from a JSON file (if given) or a catalog preset; raises ValueError if invalid."""
    design = DeltaDesign.load(design_file) if design_file else DeltaDesign.preset(preset)
    hard = [p for p in design.validate() if "무시" not in p and "짧으면" not in p]
    if hard:
        raise ValueError("invalid design: " + "; ".join(hard))
    return design


def joint_speed_limit(design: DeltaDesign) -> float:
    """Max output-shaft speed [rad/s] = motor max_speed / reduction ratio."""
    return float(design.motor_spec["max_speed"]) / design.ratio


def default_speed(design: DeltaDesign) -> float:
    """Default Cartesian speed [m/s] (same formula as DeltaRobot)."""
    return 0.25 * (design.upper_arm + design.forearm) / 0.43


def home_joints(design: DeltaDesign) -> Vec:
    h = design.home_theta
    return (h, h, h)


def tcp_from_joints(design: DeltaDesign, q: Sequence[float]) -> Vec:
    """TCP (tool tip) = effector centre - (0, 0, tool length). Raises kin.Unreachable."""
    p = kin.fk(design, q)
    return (p[0], p[1], p[2] - design.tool_length)


def joint_state_lists(design: DeltaDesign, q: Sequence[float], tool: int = 0) -> Tuple[List[str], List[float]]:
    """Names / positions for /joint_states: active + passive + virtual effector (+ gripper finger) joints."""
    js = durdf.joint_state(design, q, tool)
    names = list(js)
    return names, [float(js[n]) for n in names]


def extract_motor_positions(names: Sequence[str], positions: Sequence[float]) -> Optional[Vec]:
    """Pick motor1..3 from a JointState (any order); None if one is missing or not finite."""
    idx = {n: i for i, n in enumerate(names)}
    out: List[float] = []
    for j in JOINTS:
        i = idx.get(j)
        if i is None or i >= len(positions):
            return None
        v = float(positions[i])
        if not math.isfinite(v):
            return None
        out.append(v)
    return (out[0], out[1], out[2])


def check_joints(design: DeltaDesign, q: Sequence[float]) -> Optional[str]:
    """None if the motor angles are within limits and can be assembled, else a reason."""
    if len(q) != 3 or not all(math.isfinite(v) for v in q):
        return "need 3 finite motor angles"
    for i, v in enumerate(q):
        if v < design.theta_min - 1e-9 or v > design.theta_max + 1e-9:
            return "motor%d angle %.3f rad outside [%.3f, %.3f]" % (
                i + 1, v, design.theta_min, design.theta_max)
    try:
        kin.fk(design, q)
    except kin.Unreachable as e:
        return "forward kinematics failed: %s" % e
    return None


# ---------------------------------------------------------------- driver: mock motors
class RateLimitedJoints:
    """Three motors that track a target with a per-joint velocity limit.

    * ``set_target`` clamps to the theta limits and rejects combinations whose FK fails.
    * ``step(dt)`` moves each joint at most ``vmax * dt`` toward the target.
    * ``freeze()`` (e-stop) makes the current position the target and ignores new targets
      until ``release()``.
    """

    def __init__(self, design: DeltaDesign, q0: Optional[Sequence[float]] = None,
                 vmax: Optional[float] = None):
        self.design = design
        self.vmax = float(vmax) if vmax is not None else joint_speed_limit(design)
        start = tuple(q0) if q0 is not None else home_joints(design)
        self.q: Vec = (float(start[0]), float(start[1]), float(start[2]))
        self.target: Vec = self.q
        self.frozen = False

    def clamp(self, q: Sequence[float]) -> Vec:
        lo, hi = self.design.theta_min, self.design.theta_max
        return (min(hi, max(lo, float(q[0]))), min(hi, max(lo, float(q[1]))), min(hi, max(lo, float(q[2]))))

    def set_target(self, q: Sequence[float]) -> Optional[str]:
        """Accept a new target; returns None on success or the reason it was rejected."""
        if self.frozen:
            return "emergency stop active"
        if len(q) != 3 or not all(math.isfinite(float(v)) for v in q):
            return "need 3 finite motor angles"
        qc = self.clamp(q)
        try:
            kin.fk(self.design, qc)
        except kin.Unreachable as e:
            return "forward kinematics failed: %s" % e
        self.target = qc
        return None

    def step(self, dt: float) -> Vec:
        if dt <= 0:
            return self.q
        lim = self.vmax * dt
        self.q = tuple(c + max(-lim, min(lim, t - c))  # type: ignore[assignment]
                       for c, t in zip(self.q, self.target))
        return self.q

    def freeze(self) -> None:
        self.frozen = True
        self.target = self.q

    def release(self) -> None:
        self.frozen = False
        self.target = self.q

    @property
    def settled(self) -> bool:
        return all(abs(a - b) < 1e-9 for a, b in zip(self.q, self.target))


class Throttle:
    """Allow an action at most ``rate`` times per second (monotonic time in seconds)."""

    def __init__(self, rate: float):
        self.period = 1.0 / rate if rate > 0 else 0.0
        self._last: Optional[float] = None

    def ready(self, now: float) -> bool:
        if self._last is None or now - self._last >= self.period - 1e-9:
            self._last = now
            return True
        return False

    def reset(self) -> None:
        self._last = None


class Watchdog:
    """Reports once when no command arrived for ``timeout`` seconds."""

    def __init__(self, timeout: float = 1.0):
        self.timeout = timeout
        self._last: Optional[float] = None
        self._reported = False

    def feed(self, now: float) -> None:
        self._last = now
        self._reported = False

    def expired(self, now: float) -> bool:
        """True exactly once per silence period (never before the first command)."""
        if self._last is None or self._reported:
            return False
        if now - self._last > self.timeout:
            self._reported = True
            return True
        return False


# ---------------------------------------------------------------- serial line protocol
def serial_joint_line(q: Sequence[float]) -> str:
    """'J d1 d2 d3' in degrees (same format as deltarobot.backends.serial_backend)."""
    return "J %.2f %.2f %.2f" % tuple(math.degrees(v) for v in q)


def serial_tool_line(on: bool) -> str:
    return "T %d" % (1 if on else 0)


SERIAL_ESTOP = "E"
SERIAL_RESET = "R"
SERIAL_STATUS = "?"


def parse_status_line(line: str) -> Optional[Tuple[Vec, int]]:
    """Parse the firmware reply 'OK d1 d2 d3 tool' -> ((rad, rad, rad), tool)."""
    parts = line.strip().split()
    if len(parts) != 5 or parts[0] != "OK":
        return None
    try:
        deg = [float(v) for v in parts[1:4]]
        tool = int(parts[4])
    except ValueError:
        return None
    return (math.radians(deg[0]), math.radians(deg[1]), math.radians(deg[2])), tool


# ---------------------------------------------------------------- commander: planning
def plan_line(design: DeltaDesign, q_now: Sequence[float], goal_tcp: Sequence[float],
              profile: str = "scurve", speed: Optional[float] = None, accel: Optional[float] = None,
              dt: float = DT, check_speed: bool = True) -> List[Vec]:
    """Straight TCP line from the pose at ``q_now`` to ``goal_tcp`` -> motor angle samples.

    Every sample is checked with kinematics.limit_report; the joint speed of the sampled
    stream is checked against the motor limit.  Raises PlanError with a reason.
    """
    if len(goal_tcp) != 3 or not all(math.isfinite(float(v)) for v in goal_tcp):
        raise PlanError("goal must be 3 finite numbers")
    if profile not in traj.PROFILES:
        raise PlanError("profile must be one of %s" % (traj.PROFILES,))
    v = float(speed) if speed else default_speed(design)
    a = float(accel) if accel else 10.0 * v
    if v <= 0 or a <= 0:
        raise PlanError("speed and accel must be > 0")
    tl = design.tool_length
    goal_eff = (float(goal_tcp[0]), float(goal_tcp[1]), float(goal_tcp[2]) + tl)
    rep = kin.limit_report(design, goal_eff)
    if not rep["ok"]:
        raise PlanError("goal (%.3f, %.3f, %.3f) outside workspace: %s"
                        % (goal_tcp[0], goal_tcp[1], goal_tcp[2], ", ".join(rep["problems"])))  # type: ignore[arg-type]
    try:
        start = kin.fk(design, q_now)
    except kin.Unreachable as e:
        raise PlanError("current joints are not a valid pose: %s" % e) from None
    samples = traj.sample_path(traj.line_path(start, goal_eff), profile, v, a, dt)
    out: List[Vec] = []
    for _t, pe in samples:
        rep = kin.limit_report(design, pe)
        if not rep["ok"]:
            raise PlanError("path point (%.3f, %.3f, %.3f) outside workspace: %s"
                            % (pe[0], pe[1], pe[2] - tl, ", ".join(rep["problems"])))  # type: ignore[arg-type]
        th = rep["theta"]
        out.append((float(th[0]), float(th[1]), float(th[2])))  # type: ignore[index]
    if check_speed and out:
        w = max_joint_speed(list(zip([dt * (k + 1) for k in range(len(out))], out)), q_now, 0.0)
        lim = joint_speed_limit(design)
        if w > lim:
            raise PlanError("joint speed %.2f rad/s exceeds motor limit %.2f rad/s — lower 'speed'" % (w, lim))
    return out


# ---------------------------------------------------------------- web bridge protocol
def design_message(design: DeltaDesign, scene: Dict[str, Any]) -> Dict[str, Any]:
    return {"type": "design", "design": design.to_dict(), "scene": scene}


def state_message(t: float, q: Sequence[float], tool: bool) -> Dict[str, Any]:
    return {"type": "state", "t": round(float(t), 4), "q": [round(float(v), 6) for v in q],
            "tool": 1 if tool else 0}


def parse_session(raw: str) -> Optional[Dict[str, Any]]:
    """/delta/session payload -> {"design": dict|None, "scene": dict} (None if malformed)."""
    try:
        data = json.loads(raw)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict) or not isinstance(data.get("scene"), dict):
        return None
    design = data.get("design") if isinstance(data.get("design"), dict) else None
    return {"design": design, "scene": data["scene"]}


def parse_client_message(raw: Any) -> Optional[Dict[str, Any]]:
    """JSON object with a string 'type', else None."""
    if isinstance(raw, (bytes, bytearray)):
        try:
            raw = raw.decode("utf-8")
        except UnicodeDecodeError:
            return None
    try:
        msg = json.loads(raw)
    except (TypeError, ValueError):
        return None
    if not isinstance(msg, dict) or not isinstance(msg.get("type"), str):
        return None
    return msg


def goal_from_message(msg: Dict[str, Any]) -> Optional[Vec]:
    """{"type":"goal","xyz":[x,y,z]} -> (x, y, z) TCP [m]; None if malformed."""
    xyz = msg.get("xyz")
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        return None
    try:
        vals = [float(v) for v in xyz]
    except (TypeError, ValueError):
        return None
    if any(isinstance(v, bool) for v in xyz) or not all(math.isfinite(v) for v in vals):
        return None
    return (vals[0], vals[1], vals[2])
