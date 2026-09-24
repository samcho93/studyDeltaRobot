import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from deltarobot import DeltaDesign, DeltaRobot, WorkspaceError, catalog  # noqa: E402
from deltarobot import dynamics as dyn  # noqa: E402
from deltarobot import kinematics as kin  # noqa: E402
from deltarobot import trajectory as tr  # noqa: E402
from deltarobot import urdf  # noqa: E402

PRESETS = list(catalog()["presets"])


@pytest.mark.parametrize("preset", PRESETS)
def test_presets_valid(preset):
    assert DeltaDesign.preset(preset).validate() == []


@pytest.mark.parametrize("preset", PRESETS)
def test_ik_fk_roundtrip(preset):
    d = DeltaDesign.preset(preset)
    rnd = random.Random(1)
    n = 0
    for _ in range(300):
        th = [rnd.uniform(-0.4, 1.3) for _ in range(3)]
        try:
            p = kin.fk(d, th)
        except kin.Unreachable:
            continue
        back = kin.ik(d, p)
        if max(abs(a - b) for a, b in zip(th, back)) < 1e-9:
            n += 1
    assert n > 200


def test_jacobian_matches_finite_difference():
    d = DeltaDesign.preset("edu_dynamixel")
    th = [0.3, 0.5, 0.1]
    J = kin.jacobian(d, th)
    p0 = kin.fk(d, th)
    for c in range(3):
        t2 = list(th)
        t2[c] += 1e-7
        p1 = kin.fk(d, t2)
        for r in range(3):
            assert abs((p1[r] - p0[r]) / 1e-7 - J[r][c]) < 1e-5


def test_home_is_centered():
    d = DeltaDesign.preset("edu_servo")
    p = kin.fk(d, (0.35, 0.35, 0.35))
    assert abs(p[0]) < 1e-12 and abs(p[1]) < 1e-12 and p[2] < 0


def test_workspace_cylinder_reasonable():
    d = DeltaDesign.preset("edu_dynamixel")
    cyl = kin.work_cylinder(d, 0.1)
    assert 0.15 < cyl["diameter"] < 0.6
    assert cyl["z_top"] < 0


@pytest.mark.parametrize("kind", tr.PROFILES)
def test_profiles_limits(kind):
    p = tr.Profile(kind, 0.3, 1.0, 8.0)
    s, v, a = p.at(p.T)
    assert abs(s - 0.3) < 1e-9 and abs(v) < 1e-9
    vmax = max(p.at(p.T * k / 400)[1] for k in range(401))
    amax = max(abs(p.at(p.T * k / 400)[2]) for k in range(401))
    assert vmax <= 1.0 + 1e-9 and amax <= 8.0 + 1e-9


def test_static_torque_sign_and_symmetry():
    d = DeltaDesign.preset("edu_dynamixel")
    tau = dyn.static_torques(d, (0.0, 0.0, -0.3))
    # the arms must hold the weight up -> negative (upward) torque, equal by symmetry
    assert all(t < 0 for t in tau)
    assert max(tau) - min(tau) < 1e-9


def _rot(ax, a):
    import numpy as np
    x, y, z = ax
    c, s = math.cos(a), math.sin(a)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def test_urdf_closes_chain():
    np = pytest.importorskip("numpy")
    d = DeltaDesign.preset("printer_stepper")
    th = (0.2, 0.7, 0.4)
    root = ET.fromstring(urdf.generate(d).split("\n", 1)[1])
    js = urdf.joint_state(d, th)
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}

    def val(j):
        m = j.find("mimic")
        return js[m.get("joint")] if m is not None else js.get(j.get("name"), 0.0)

    def T(link):
        M = np.eye(4)
        while link in by_child:
            j = by_child[link]
            o = j.find("origin")
            r, p_, y = [float(v) for v in o.get("rpy").split()]
            A = np.eye(4)
            A[:3, :3] = _rot((0, 0, 1), y) @ _rot((0, 1, 0), p_) @ _rot((1, 0, 0), r)
            A[:3, 3] = [float(v) for v in o.get("xyz").split()]
            B = np.eye(4)
            if j.get("type") in ("revolute", "prismatic"):
                ax = [float(v) for v in j.find("axis").get("xyz").split()]
                if j.get("type") == "revolute":
                    B[:3, :3] = _rot(ax, val(j))
                else:
                    B[:3, 3] = np.array(ax) * val(j)
            M = A @ B @ M
            link = j.find("parent").get("link")
        return M

    p = np.array(kin.fk(d, th))
    for i in range(3):
        u = np.array([math.cos(kin.PHI[i]), math.sin(kin.PHI[i]), 0])
        t = np.array([-u[1], u[0], 0])
        for k, s in (("a", 1), ("b", -1)):
            end = T("forearm%d%s" % (i + 1, k)) @ np.array([d.forearm, 0, 0, 1])
            want = p + d.effector_radius * u + s * d.forearm_spacing / 2 * t
            assert np.abs(end[:3] - want).max() < 1e-5


def test_robot_record_pick_place():
    r = DeltaRobot("edu_dynamixel", scene="pick_place")
    parts = r.parts()
    assert parts
    r.home()
    target = parts[0]
    bin_ = r.scene.bin("A")
    r.arch_to(target["x"], target["y"], target["top"], height=0.03)
    assert r.tool_on() == target["id"]
    r.arch_to(bin_["x"], bin_["y"], target["top"] + 0.02, height=0.03)
    r.tool_off()
    assert r.scene.score()["per_bin"]["A"] == 1
    res = r.analyze()
    assert "peak_ratio" in res


def test_robot_rejects_outside_workspace():
    r = DeltaRobot("edu_servo")
    with pytest.raises(WorkspaceError):
        r.move_to(0.5, 0, -0.2)


def test_virtual_time_limit():
    r = DeltaRobot("edu_servo", backend="record", time_limit=5)
    with pytest.raises(RuntimeError):
        while True:
            r.wait(1.0)


def test_websim_origin_filter():
    from deltarobot.backends.websim import origin_allowed
    assert origin_allowed("https://samcho93.github.io")
    assert origin_allowed("http://localhost:8000")
    assert origin_allowed(None)
    assert not origin_allowed("https://evil.example")
    assert not origin_allowed("http://localhost.evil.example")


def test_urdf_gripper_fingers_close_symmetrically():
    np = pytest.importorskip("numpy")
    d = DeltaDesign.preset("edu_dynamixel")          # servo parallel gripper
    assert d.tool_spec["kind"] == "gripper"
    root = ET.fromstring(urdf.generate(d).split("\n", 1)[1])
    joints = {j.get("name"): j for j in root.findall("joint")}
    a, b = joints["gripper_finger_a"], joints["gripper_finger_b"]
    assert a.get("type") == b.get("type") == "prismatic"
    assert b.find("mimic").get("joint") == "gripper_finger_a"
    th = (0.35, 0.35, 0.35)
    for tool, gap in ((0, 2 * urdf.GRIPPER_FINGER_X), (1, 2 * (urdf.GRIPPER_FINGER_X - urdf.GRIPPER_STROKE))):
        v = urdf.joint_state(d, th, tool)["gripper_finger_a"]
        xa = float(a.find("origin").get("xyz").split()[0]) + float(a.find("axis").get("xyz").split()[0]) * v
        xb = float(b.find("origin").get("xyz").split()[0]) + float(b.find("axis").get("xyz").split()[0]) * v
        assert abs(xa + xb) < 1e-12 and abs((xa - xb) - gap) < 1e-12
    assert "gripper_finger_a" not in urdf.joint_state(DeltaDesign.preset("edu_servo"), th)   # suction cup
