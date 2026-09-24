"""Simplified rigid-body dynamics for motor sizing (Codourey-style lumped model).

Per arm i (output side, tau > 0 pushes the arm down):

    tau_i = I_arm * theta_dd_i - k_g * g * cos(theta_i) + (J^T * m_p * (a - g_vec))_i

  I_arm   upper arm (rod, L^2/3) + elbow + half forearm pair (at L) + reflected rotor/gearbox
  k_g     gravity moment of the upper arm, elbow and half forearm pair
  m_p     moving plate: plate + tool + payload + half of every forearm pair
  J       v = J theta_dot   (so the joint torques for a plate force F are J^T F)

Motor side:  tau_m = tau / (N * eta),  omega_m = theta_dot * N.
Mirrored in assets/js/delta/dynamics.js.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

from . import kinematics as kin
from .design import G, DeltaDesign


def joint_torques(design: DeltaDesign, theta: Sequence[float], theta_dd: Sequence[float],
                  p: Optional[Sequence[float]] = None,
                  a: Sequence[float] = (0.0, 0.0, 0.0)) -> List[float]:
    """Output-shaft torques [N m] for one state."""
    if p is None:
        p = kin.fk(design, theta)
    J = kin.jacobian(design, theta, p)
    m = design.moving_plate_mass
    f = (m * a[0], m * a[1], m * (a[2] + G))          # force the arms must apply to the plate
    jtf = [J[0][c] * f[0] + J[1][c] * f[1] + J[2][c] * f[2] for c in range(3)]
    I = design.arm_inertia
    k = design.gravity_moment
    return [I * theta_dd[i] - k * G * math.cos(theta[i]) + jtf[i] for i in range(3)]


def static_torques(design: DeltaDesign, p: Sequence[float]) -> List[float]:
    th = kin.ik(design, p)
    return joint_torques(design, th, (0.0, 0.0, 0.0), p)


def available_torque(design: DeltaDesign, omega_motor: float) -> float:
    """Peak torque the motor can deliver at shaft speed omega (speed-torque model)."""
    m = design.motor_spec
    w = abs(omega_motor)
    wmax = float(m["max_speed"])
    peak = float(m["peak_torque"])
    kind = m["kind"]
    if w >= wmax:
        return 0.0
    if kind in ("rc_servo", "smart_servo"):
        return peak * (1.0 - w / wmax)                 # DC-motor line: stall -> no-load speed
    if kind == "stepper":
        tmax = float(m.get("torque_at_max_speed", 0.25 * peak))
        return peak - (peak - tmax) * w / wmax          # pull-out curve (linearised)
    return peak                                          # servo drives hold peak up to max speed


STEPPER_MARGIN = 1.5   # steppers lose steps silently — keep 50 % reserve


def evaluate(design: DeltaDesign, samples: Sequence[Dict[str, object]],
             safety_factor: float = 1.2) -> Dict[str, object]:
    """Check a sampled trajectory against the motor.

    samples: dicts with keys t, theta, theta_d, tau (output side, from :func:`analyze`).
    Returns utilisation numbers and a list of problems (Korean messages).
    """
    m = design.motor_spec
    n, eta = design.ratio, design.efficiency
    peak_m = [0.0, 0.0, 0.0]
    wmax_m = [0.0, 0.0, 0.0]
    sq = [0.0, 0.0, 0.0]
    worst_ratio = 0.0
    total_t = 0.0
    prev_t = None
    for s in samples:
        t = float(s["t"])  # type: ignore[arg-type]
        dt = 0.0 if prev_t is None else t - prev_t
        prev_t = t
        total_t += dt
        for i in range(3):
            tm = abs(s["tau"][i]) / (n * eta)  # type: ignore[index]
            wm = abs(s["theta_d"][i]) * n       # type: ignore[index]
            peak_m[i] = max(peak_m[i], tm)
            wmax_m[i] = max(wmax_m[i], wm)
            sq[i] += tm * tm * dt
            avail = available_torque(design, wm)
            if m["kind"] == "stepper":
                avail /= STEPPER_MARGIN
            ratio = tm * safety_factor / avail if avail > 0 else float("inf")
            worst_ratio = max(worst_ratio, ratio)
    rms = [math.sqrt(v / total_t) if total_t > 0 else 0.0 for v in sq]
    problems: List[str] = []
    peak_ratio = max(peak_m) * safety_factor / float(m["peak_torque"])
    rms_ratio = max(rms) * safety_factor / float(m["rated_torque"])
    speed_ratio = max(wmax_m) / float(m["max_speed"])
    if worst_ratio > 1.0:
        problems.append("속도-토크 곡선 초과: 해당 속도에서 낼 수 있는 토크보다 큰 토크가 필요합니다")
    if peak_ratio > 1.0:
        problems.append("피크 토크 초과 (안전율 %.1f 포함)" % safety_factor)
    if rms_ratio > 1.0:
        problems.append("RMS(연속) 토크가 정격을 넘습니다 — 과열 위험")
    if speed_ratio > 1.0:
        problems.append("모터 최고 속도 초과")
    return {
        "peak_torque_motor": peak_m,
        "rms_torque_motor": rms,
        "max_speed_motor": wmax_m,
        "peak_ratio": peak_ratio,
        "rms_ratio": rms_ratio,
        "speed_ratio": speed_ratio,
        "curve_ratio": worst_ratio,
        "inertia_ratio": inertia_ratio(design),
        "ok": not problems,
        "problems": problems,
    }


def inertia_ratio(design: DeltaDesign) -> float:
    """Load inertia reflected to the motor / rotor inertia (0 for integrated servos)."""
    m = design.motor_spec
    if m.get("integrated_gear") or float(m["rotor_inertia"]) <= 0:
        return 0.0
    n = design.ratio
    load = design.arm_inertia - design.drive_inertia
    # plate mass seen at the joint near home: m_p * (dp/dtheta)^2 ~ m_p * L^2 / 3 (rough)
    load += design.moving_plate_mass * design.upper_arm ** 2 / 3.0
    return load / (n * n) / float(m["rotor_inertia"])


def analyze(design: DeltaDesign, frames: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    """Add velocities, accelerations and torques to a list of frames {t, q}.

    Derivatives use central differences (one-sided at the ends).
    """
    n = len(frames)
    out: List[Dict[str, object]] = []
    if n == 0:
        return out
    ts = [float(f["t"]) for f in frames]  # type: ignore[arg-type]
    qs = [list(f["q"]) for f in frames]   # type: ignore[arg-type]
    ps = [kin.fk(design, q) for q in qs]

    def deriv(vals, k):
        if n == 1:
            return [0.0] * len(vals[0])
        lo, hi = max(0, k - 1), min(n - 1, k + 1)
        dt = ts[hi] - ts[lo]
        if dt <= 0:
            return [0.0] * len(vals[0])
        return [(vals[hi][j] - vals[lo][j]) / dt for j in range(len(vals[0]))]

    qd = [deriv(qs, k) for k in range(n)]
    v = [deriv(ps, k) for k in range(n)]
    qdd = [deriv(qd, k) for k in range(n)]
    acc = [deriv(v, k) for k in range(n)]
    for k in range(n):
        tau = joint_torques(design, qs[k], qdd[k], ps[k], acc[k])
        out.append({"t": ts[k], "theta": qs[k], "theta_d": qd[k], "theta_dd": qdd[k],
                    "p": ps[k], "v": v[k], "a": acc[k], "tau": tau,
                    "tool": frames[k].get("tool", 0)})
    return out
