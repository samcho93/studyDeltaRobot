"""Rotary delta robot kinematics (pure Python — runs unchanged in Pyodide).

Conventions (CLAUDE.md §2):
  * base frame: origin at the base centre, z up, the effector works at z < 0
  * arm i sits at azimuth PHI[i] = 0, 120, 240 deg
  * motor angle theta: 0 = upper arm horizontal, positive = arm swings down
  * elbow in the arm frame: (R + L cos(theta), 0, -L sin(theta))
  * all positions here are the *effector centre*; the robot API adds the tool length

Mirrored in assets/js/delta/kinematics.js — keep both in sync (tests/test_js_parity.py).
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

Vec = Tuple[float, float, float]

PHI = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
ELBOW_MIN = math.radians(20.0)    # interior elbow angle limits (folded / stretched)
ELBOW_MAX = math.radians(165.0)


class Unreachable(ValueError):
    """Raised when a position cannot be reached by the arms."""


# ---------------------------------------------------------------- small vector helpers
def _sub(a: Sequence[float], b: Sequence[float]) -> Vec:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Sequence[float], b: Sequence[float]) -> Vec:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Sequence[float], k: float) -> Vec:
    return (a[0] * k, a[1] * k, a[2] * k)


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Sequence[float], b: Sequence[float]) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a: Sequence[float]) -> float:
    return math.sqrt(_dot(a, a))


def det3(m: Sequence[Sequence[float]]) -> float:
    return _dot(m[0], _cross(m[1], m[2]))


def inv3(m: Sequence[Sequence[float]]) -> List[List[float]]:
    d = det3(m)
    if abs(d) < 1e-15:
        raise ZeroDivisionError("singular matrix")
    c0, c1, c2 = _cross(m[1], m[2]), _cross(m[2], m[0]), _cross(m[0], m[1])
    # inverse = adj / det, adj columns are the cross products above
    return [[c0[0] / d, c1[0] / d, c2[0] / d],
            [c0[1] / d, c1[1] / d, c2[1] / d],
            [c0[2] / d, c1[2] / d, c2[2] / d]]


def matvec(m: Sequence[Sequence[float]], v: Sequence[float]) -> Vec:
    return (_dot(m[0], v), _dot(m[1], v), _dot(m[2], v))


def transpose(m: Sequence[Sequence[float]]) -> List[List[float]]:
    return [[m[j][i] for j in range(3)] for i in range(3)]


def matmul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    bt = transpose(b)
    return [[_dot(a[i], bt[j]) for j in range(3)] for i in range(3)]


def _sym_eigvals(a: Sequence[Sequence[float]]) -> List[float]:
    """Eigenvalues of a symmetric 3x3 matrix (closed form, Smith 1961)."""
    p1 = a[0][1] ** 2 + a[0][2] ** 2 + a[1][2] ** 2
    q = (a[0][0] + a[1][1] + a[2][2]) / 3.0
    if p1 < 1e-30:
        return sorted([a[0][0], a[1][1], a[2][2]])
    p2 = (a[0][0] - q) ** 2 + (a[1][1] - q) ** 2 + (a[2][2] - q) ** 2 + 2 * p1
    p = math.sqrt(p2 / 6.0)
    b = [[(a[i][j] - (q if i == j else 0.0)) / p for j in range(3)] for i in range(3)]
    rr = max(-1.0, min(1.0, det3(b) / 2.0))
    phi = math.acos(rr) / 3.0
    e1 = q + 2 * p * math.cos(phi)
    e3 = q + 2 * p * math.cos(phi + 2 * math.pi / 3)
    e2 = 3 * q - e1 - e3
    return sorted([e3, e2, e1])


# ---------------------------------------------------------------- geometry
def _R(d):  # noqa: N802
    return d.base_radius


def arm_axis(i: int) -> Vec:
    """Unit radial direction of arm i."""
    return (math.cos(PHI[i]), math.sin(PHI[i]), 0.0)


def motor_point(design, i: int) -> Vec:
    return _scale(arm_axis(i), design.base_radius)


def elbow(design, i: int, theta: float) -> Vec:
    """Elbow position of arm i in the base frame."""
    rho = design.base_radius + design.upper_arm * math.cos(theta)
    c, s = math.cos(PHI[i]), math.sin(PHI[i])
    return (rho * c, rho * s, -design.upper_arm * math.sin(theta))


def elbow_derivative(design, i: int, theta: float) -> Vec:
    """d(elbow)/d(theta)."""
    c, s = math.cos(PHI[i]), math.sin(PHI[i])
    L = design.upper_arm
    return (-L * math.sin(theta) * c, -L * math.sin(theta) * s, -L * math.cos(theta))


def ball_joint(design, i: int, p: Sequence[float]) -> Vec:
    """Centre of the effector-side ball-joint pair of arm i for effector position p."""
    return _add(p, _scale(arm_axis(i), design.effector_radius))


# ---------------------------------------------------------------- inverse kinematics
def ik_arm(design, i: int, p: Sequence[float]) -> float:
    """Motor angle of arm i for effector position p (raises Unreachable)."""
    c, s = math.cos(PHI[i]), math.sin(PHI[i])
    x, y, z = p
    xa = x * c + y * s                 # position in the arm frame
    ya = -x * s + y * c
    a = xa + design.effector_radius - design.base_radius
    L, l = design.upper_arm, design.forearm
    # (a - L cos t)^2 + ya^2 + (z + L sin t)^2 = l^2  ->  z sin t - a cos t = K
    K = (l * l - L * L - a * a - ya * ya - z * z) / (2.0 * L)
    rho = math.hypot(a, z)
    if rho < 1e-12 or abs(K) > rho:
        raise Unreachable("arm %d cannot reach (%.4f, %.4f, %.4f)" % (i + 1, x, y, z))
    alpha = math.atan2(a, z)
    asn = math.asin(K / rho)
    t1 = alpha + asn
    t2 = alpha + math.pi - asn
    # choose the elbow-out solution (larger cos)
    t = t1 if math.cos(t1) > math.cos(t2) else t2
    return math.atan2(math.sin(t), math.cos(t))


def ik(design, p: Sequence[float]) -> Tuple[float, float, float]:
    """Inverse kinematics: effector centre -> (theta1, theta2, theta3) [rad]."""
    return (ik_arm(design, 0, p), ik_arm(design, 1, p), ik_arm(design, 2, p))


# ---------------------------------------------------------------- forward kinematics
def fk(design, theta: Sequence[float]) -> Vec:
    """Forward kinematics: (theta1..3) -> effector centre (lower intersection).

    Each arm constrains the effector centre to a sphere of radius l around
    C_i = elbow_i - r * u_i ; the three spheres are intersected (trilateration).
    """
    l = design.forearm
    c = [_sub(elbow(design, i, theta[i]), _scale(arm_axis(i), design.effector_radius)) for i in range(3)]
    ex_v = _sub(c[1], c[0])
    d = _norm(ex_v)
    if d < 1e-12:
        raise Unreachable("degenerate sphere centres")
    ex = _scale(ex_v, 1.0 / d)
    t = _sub(c[2], c[0])
    i_ = _dot(ex, t)
    ey_v = _sub(t, _scale(ex, i_))
    ey_n = _norm(ey_v)
    if ey_n < 1e-12:
        raise Unreachable("sphere centres are collinear")
    ey = _scale(ey_v, 1.0 / ey_n)
    ez = _cross(ex, ey)
    j = _dot(ey, t)
    # equal radii (l) simplify the classic trilateration formulas
    x = d / 2.0
    y = (i_ * i_ + j * j - 2.0 * i_ * x) / (2.0 * j)
    h2 = l * l - x * x - y * y
    if h2 < 0:
        raise Unreachable("no forward kinematic solution for theta=%s" % (tuple(round(v, 4) for v in theta),))
    h = math.sqrt(h2)
    base = _add(c[0], _add(_scale(ex, x), _scale(ey, y)))
    p1 = _add(base, _scale(ez, h))
    p2 = _sub(base, _scale(ez, h))
    return p1 if p1[2] < p2[2] else p2


# ---------------------------------------------------------------- jacobian / velocity
def jacobian_parts(design, theta: Sequence[float], p: Optional[Sequence[float]] = None):
    """Return (Jx, Jtheta_diag) with Jx * v = diag(Jtheta) * theta_dot."""
    if p is None:
        p = fk(design, theta)
    jx, jt = [], []
    for i in range(3):
        s = _sub(ball_joint(design, i, p), elbow(design, i, theta[i]))
        jx.append(list(s))
        jt.append(_dot(s, elbow_derivative(design, i, theta[i])))
    return jx, jt


def jacobian(design, theta: Sequence[float], p: Optional[Sequence[float]] = None) -> List[List[float]]:
    """J such that v = J * theta_dot (3x3, rows = x, y, z)."""
    jx, jt = jacobian_parts(design, theta, p)
    inv = inv3(jx)
    return [[inv[r][c] * jt[c] for c in range(3)] for r in range(3)]


def joint_velocity(design, theta: Sequence[float], v: Sequence[float],
                   p: Optional[Sequence[float]] = None) -> Vec:
    """theta_dot for a Cartesian velocity v (inverse jacobian, no matrix inverse needed)."""
    jx, jt = jacobian_parts(design, theta, p)
    return tuple(_dot(jx[i], v) / jt[i] for i in range(3))  # type: ignore[return-value]


def singular_values(j: Sequence[Sequence[float]]) -> List[float]:
    jtj = matmul(transpose(j), j)
    return [math.sqrt(max(0.0, e)) for e in _sym_eigvals(jtj)]


def condition_number(design, theta: Sequence[float], p: Optional[Sequence[float]] = None) -> float:
    """sigma_max / sigma_min of J (1 = isotropic, large = near singular)."""
    try:
        sv = singular_values(jacobian(design, theta, p))
    except (ZeroDivisionError, Unreachable):
        return float("inf")
    return sv[-1] / sv[0] if sv[0] > 1e-12 else float("inf")


# ---------------------------------------------------------------- passive joints / limits
def forearm_direction(design, i: int, theta: float, p: Sequence[float]) -> Vec:
    s = _sub(ball_joint(design, i, p), elbow(design, i, theta))
    return _scale(s, 1.0 / _norm(s))


def passive_angles(design, theta: Sequence[float], p: Optional[Sequence[float]] = None) -> List[Tuple[float, float]]:
    """(pitch, yaw) of each forearm relative to its upper arm — used by the URDF.

    forearm frame = Rz(phi) Ry(theta) Ry(pitch) Rz(yaw); its x axis is the rod direction.
    """
    if p is None:
        p = fk(design, theta)
    out = []
    for i in range(3):
        d = forearm_direction(design, i, theta[i], p)
        # d_local = Ry(-theta) Rz(-phi) d
        c, s = math.cos(PHI[i]), math.sin(PHI[i])
        dx, dy, dz = d[0] * c + d[1] * s, -d[0] * s + d[1] * c, d[2]
        ct, st = math.cos(theta[i]), math.sin(theta[i])
        lx = ct * dx - st * dz          # Ry(-t): x' = cos t x - sin t z
        lz = st * dx + ct * dz          #          z' = sin t x + cos t z
        ly = dy
        yaw = math.asin(max(-1.0, min(1.0, ly)))
        pitch = math.atan2(-lz, lx)
        out.append((pitch, yaw))
    return out


def elbow_angle(design, i: int, theta: float, p: Sequence[float]) -> float:
    """Interior angle at the elbow between (motor - elbow) and (ball - elbow)."""
    e = elbow(design, i, theta)
    a = _sub(motor_point(design, i), e)
    b = _sub(ball_joint(design, i, p), e)
    cosv = _dot(a, b) / (_norm(a) * _norm(b))
    return math.acos(max(-1.0, min(1.0, cosv)))


def limit_report(design, p: Sequence[float]) -> Dict[str, object]:
    """Check a position: reachability, motor range, ball joints, elbow angle.

    Returns {"ok": bool, "theta": (..)|None, "problems": [str, ...], "ball": [..], "elbow": [..]}.
    """
    problems: List[str] = []
    try:
        th = ik(design, p)
    except Unreachable:
        return {"ok": False, "theta": None, "problems": ["unreachable"], "ball": [], "elbow": []}
    ball, elb = [], []
    pa = passive_angles(design, th, p)
    for i in range(3):
        if th[i] < design.theta_min - 1e-9 or th[i] > design.theta_max + 1e-9:
            problems.append("theta%d" % (i + 1))
        yaw = pa[i][1]
        ball.append(abs(yaw))
        if abs(yaw) > design.ball_joint_limit:
            problems.append("ball%d" % (i + 1))
        ea = elbow_angle(design, i, th[i], p)
        elb.append(ea)
        if ea < ELBOW_MIN or ea > ELBOW_MAX:
            problems.append("elbow%d" % (i + 1))
    return {"ok": not problems, "theta": th, "problems": problems, "ball": ball, "elbow": elb}


def reachable(design, p: Sequence[float]) -> bool:
    return bool(limit_report(design, p)["ok"])


# ---------------------------------------------------------------- workspace
def workspace_bounds(design) -> Tuple[float, float, float]:
    """(radius, z_top, z_bottom) of a box that surely contains the workspace."""
    reach = design.base_radius - design.effector_radius + design.upper_arm + design.forearm
    return reach, 0.0, -(design.upper_arm + design.forearm)


def workspace_points(design, step: float = 0.0) -> List[Vec]:
    """Grid-sample reachable effector positions (step default: reach/14)."""
    rad, ztop, zbot = workspace_bounds(design)
    h = step or rad / 14.0
    pts: List[Vec] = []
    n = int(rad / h)
    nz = int((ztop - zbot) / h)
    for iz in range(nz + 1):
        z = ztop - iz * h
        for ix in range(-n, n + 1):
            for iy in range(-n, n + 1):
                p = (ix * h, iy * h, z)
                if p[0] * p[0] + p[1] * p[1] > rad * rad:
                    continue
                if reachable(design, p):
                    pts.append(p)
    return pts


def max_radius_at(design, z: float, rmax: float, n_dir: int = 24, tol: float = 5e-4) -> float:
    """Largest radius rho such that the whole circle (rho, z) is reachable (0 if centre is not)."""
    if not reachable(design, (0.0, 0.0, z)):
        return 0.0

    def ok(rho: float) -> bool:
        for k in range(n_dir):
            a = 2 * math.pi * k / n_dir
            if not reachable(design, (rho * math.cos(a), rho * math.sin(a), z)):
                return False
        return True

    # march out, then bisect
    step = rmax / 20.0
    lo = 0.0
    hi = None
    rho = step
    while rho <= rmax:
        if ok(rho):
            lo = rho
            rho += step
        else:
            hi = rho
            break
    if hi is None:
        return lo
    while hi - lo > tol:
        mid = (lo + hi) / 2
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo


def work_cylinder(design, height: float, nz: int = 40) -> Dict[str, object]:
    """Best upright cylinder of the given height inside the workspace.

    Returns {"diameter", "height", "z_top", "z_bottom", "profile": [(z, radius), ...]}.
    """
    rad, ztop, zbot = workspace_bounds(design)
    dz = (ztop - zbot) / nz
    profile = []
    for k in range(nz + 1):
        z = ztop - k * dz
        profile.append((z, max_radius_at(design, z, rad)))
    win = max(1, int(round(height / dz)))
    best = (0.0, 0)
    for k in range(0, len(profile) - win):
        m = min(r for _, r in profile[k:k + win + 1])
        if m > best[0]:
            best = (m, k)
    rbest, k0 = best
    z_top = profile[k0][0]
    return {"diameter": 2 * rbest, "height": win * dz, "z_top": z_top,
            "z_bottom": z_top - win * dz, "profile": profile}
