"""Time scaling profiles and paths (Cartesian line, pick-and-place arch, joint space).

A *profile* maps time to distance along a path: s(t), s'(t), s''(t) for 0 <= t <= T.
Profiles: "trapezoid", "scurve" (sin^2 acceleration), "quintic", "cycloidal".
Mirrored in assets/js/delta/trajectory.js.
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

Vec = Tuple[float, float, float]
PROFILES = ("trapezoid", "scurve", "quintic", "cycloidal")


class Profile:
    """Rest-to-rest time scaling over distance d with speed/accel limits."""

    def __init__(self, kind: str, d: float, vmax: float, amax: float):
        if kind not in PROFILES:
            raise ValueError("profile must be one of %s" % (PROFILES,))
        if vmax <= 0 or amax <= 0:
            raise ValueError("vmax and amax must be positive")
        self.kind, self.d, self.vmax, self.amax = kind, max(0.0, d), vmax, amax
        d = self.d
        if d == 0:
            self.T = 0.0
            return
        if kind == "trapezoid":
            ta = vmax / amax
            if amax * ta * ta >= d:           # triangular
                ta = math.sqrt(d / amax)
                self.vp = amax * ta
                self.ta, self.tc = ta, 0.0
            else:
                self.vp = vmax
                self.ta, self.tc = ta, (d - vmax * ta) / vmax
            self.T = 2 * self.ta + self.tc
        elif kind == "scurve":
            # a(t) = A sin^2(pi t / ta): mean accel A/2, reaches v = A ta / 2
            A = amax
            ta = 2 * vmax / A
            if A * ta * ta / 2 >= d:          # distance of accel+decel phases = v * ta
                ta = math.sqrt(2 * d / A)
                self.vp = A * ta / 2
                self.ta, self.tc = ta, 0.0
            else:
                self.vp = vmax
                self.ta, self.tc = ta, (d - vmax * ta) / vmax
            self.A = A
            self.T = 2 * self.ta + self.tc
        elif kind == "quintic":
            self.T = max(15 * d / (8 * vmax), math.sqrt(10 * d / (math.sqrt(3) * amax)))
        else:  # cycloidal
            self.T = max(2 * d / vmax, math.sqrt(2 * math.pi * d / amax))

    def at(self, t: float) -> Tuple[float, float, float]:
        """(s, s_dot, s_ddot) at time t (clamped to [0, T])."""
        d, T = self.d, self.T
        if T <= 0:
            return 0.0, 0.0, 0.0
        t = min(max(t, 0.0), T)
        k = self.kind
        if k == "trapezoid":
            ta, tc, vp = self.ta, self.tc, self.vp
            a = vp / ta
            if t < ta:
                return 0.5 * a * t * t, a * t, a
            if t < ta + tc:
                return 0.5 * a * ta * ta + vp * (t - ta), vp, 0.0
            u = T - t
            return d - 0.5 * a * u * u, a * u, -a if t < T else 0.0
        if k == "scurve":
            ta, tc, vp, A = self.ta, self.tc, self.vp, self.A
            A = 2 * vp / ta                     # actual peak accel for this move

            def acc_phase(u: float) -> Tuple[float, float, float]:
                w = math.pi / ta
                s = A * (u * u / 4 - (1 - math.cos(2 * w * u)) / (8 * w * w))
                v = A * (u / 2 - math.sin(2 * w * u) / (4 * w))
                a = A * math.sin(w * u) ** 2
                return s, v, a

            if t < ta:
                return acc_phase(t)
            s_a = acc_phase(ta)[0]
            if t < ta + tc:
                return s_a + vp * (t - ta), vp, 0.0
            s, v, a = acc_phase(T - t)
            return d - s, v, -a
        if k == "quintic":
            x = t / T
            s = d * (10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5)
            v = d * (30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4) / T
            a = d * (60 * x - 180 * x ** 2 + 120 * x ** 3) / (T * T)
            return s, v, a
        x = t / T
        s = d * (x - math.sin(2 * math.pi * x) / (2 * math.pi))
        v = d * (1 - math.cos(2 * math.pi * x)) / T
        a = d * 2 * math.pi * math.sin(2 * math.pi * x) / (T * T)
        return s, v, a


# ---------------------------------------------------------------- paths
def _lerp(a: Sequence[float], b: Sequence[float], u: float) -> Vec:
    return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u)


def _dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


class Path:
    """Piecewise path made of straight lines and circular arcs, parametrised by arc length."""

    def __init__(self) -> None:
        self.segs: List[tuple] = []   # ("line", p0, p1, len) | ("arc", centre, u, w, radius, angle, len)
        self.length = 0.0

    def line(self, p0: Sequence[float], p1: Sequence[float]) -> "Path":
        n = _dist(p0, p1)
        if n > 1e-12:
            self.segs.append(("line", tuple(p0), tuple(p1), n))
            self.length += n
        return self

    def arc(self, centre: Sequence[float], u: Sequence[float], w: Sequence[float],
            radius: float, angle: float) -> "Path":
        """Arc: centre + radius*(cos(a) u + sin(a) w), a from 0 to angle."""
        n = radius * abs(angle)
        if n > 1e-12:
            self.segs.append(("arc", tuple(centre), tuple(u), tuple(w), radius, angle, n))
            self.length += n
        return self

    def point(self, s: float) -> Vec:
        s = min(max(s, 0.0), self.length)
        for seg in self.segs:
            n = seg[-1]
            if s <= n or seg is self.segs[-1]:
                if seg[0] == "line":
                    return _lerp(seg[1], seg[2], s / n if n else 0.0)
                _, c, u, w, rad, ang, _n = seg
                a = ang * (s / n)
                ca, sa = math.cos(a), math.sin(a)
                return (c[0] + rad * (ca * u[0] + sa * w[0]),
                        c[1] + rad * (ca * u[1] + sa * w[1]),
                        c[2] + rad * (ca * u[2] + sa * w[2]))
            s -= n
        return tuple(self.segs[-1][2]) if self.segs else (0.0, 0.0, 0.0)  # type: ignore[return-value]


def line_path(p0: Sequence[float], p1: Sequence[float]) -> Path:
    return Path().line(p0, p1)


def arch_path(p0: Sequence[float], p1: Sequence[float], height: float, radius: float = 0.0) -> Path:
    """Pick-and-place 'door' path: up from p0, across, down to p1, rounded corners.

    The travel height is max(p0.z, p1.z) + height. radius defaults to height/2.
    """
    ztop = max(p0[2], p1[2]) + height
    a = (p0[0], p0[1], ztop)
    b = (p1[0], p1[1], ztop)
    horiz = _dist(a, b)
    path = Path()
    if horiz < 1e-9:
        return path.line(p0, a).line(a, p1) if height > 0 else path.line(p0, p1)
    rad = radius or height / 2.0
    rad = max(0.0, min(rad, ztop - p0[2], ztop - p1[2], horiz / 2.0))
    hdir = ((b[0] - a[0]) / horiz, (b[1] - a[1]) / horiz, 0.0)
    up = (0.0, 0.0, 1.0)
    # corner 1 at a: coming up, leaving along hdir
    c1 = (a[0] + hdir[0] * rad, a[1] + hdir[1] * rad, ztop - rad)
    path.line(p0, (a[0], a[1], ztop - rad))
    if rad > 0:
        path.arc(c1, (-hdir[0], -hdir[1], 0.0), up, rad, math.pi / 2)
    path.line((a[0] + hdir[0] * rad, a[1] + hdir[1] * rad, ztop),
              (b[0] - hdir[0] * rad, b[1] - hdir[1] * rad, ztop))
    c2 = (b[0] - hdir[0] * rad, b[1] - hdir[1] * rad, ztop - rad)
    if rad > 0:
        path.arc(c2, up, hdir, rad, math.pi / 2)
    path.line((b[0], b[1], ztop - rad), p1)
    return path


def sample_path(path: Path, profile_kind: str, vmax: float, amax: float, dt: float,
                t0: float = 0.0) -> List[Tuple[float, Vec]]:
    """Sample (t, point) along a path; always includes the final point."""
    prof = Profile(profile_kind, path.length, vmax, amax)
    out: List[Tuple[float, Vec]] = []
    n = max(1, int(math.ceil(prof.T / dt)))
    for k in range(1, n + 1):
        t = min(prof.T, k * dt)
        out.append((t0 + t, path.point(prof.at(t)[0])))
    return out


def sample_joint(q0: Sequence[float], q1: Sequence[float], profile_kind: str, wmax: float,
                 alpha: float, dt: float, t0: float = 0.0) -> List[Tuple[float, Vec]]:
    """Synchronised joint-space move: the largest joint move sets the timing."""
    dq = [q1[i] - q0[i] for i in range(3)]
    big = max(abs(v) for v in dq)
    prof = Profile(profile_kind, big, wmax, alpha)
    out: List[Tuple[float, Vec]] = []
    n = max(1, int(math.ceil(prof.T / dt)))
    for k in range(1, n + 1):
        t = min(prof.T, k * dt)
        u = prof.at(t)[0] / big if big > 0 else 1.0
        out.append((t0 + t, (q0[0] + dq[0] * u, q0[1] + dq[1] * u, q0[2] + dq[2] * u)))
    return out
