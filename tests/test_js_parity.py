"""Python <-> JS parity: the web simulator must compute exactly what the Python library computes."""

import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

from deltarobot import DeltaDesign  # noqa: E402
from deltarobot import dynamics as dyn  # noqa: E402
from deltarobot import kinematics as kin  # noqa: E402
from deltarobot import scene, urdf  # noqa: E402
from deltarobot import trajectory as tr  # noqa: E402
from deltarobot.design import summary  # noqa: E402

CASES = {
    "edu_servo": {"preset": "edu_servo", "thetas": [[0.3, 0.5, 0.2], [0.0, 0.0, 0.0], [1.2, -0.5, 0.9]],
                  "points": [[0.02, -0.03, -0.19], [-0.04, 0.03, -0.21], [0.3, 0.0, -0.1], [0.0, 0.0, -0.3]]},
    "industrial": {"preset": "industrial_picker", "thetas": [[0.4, 0.2, 0.6], [0.1, 0.9, 0.3], [1.4, 1.4, 1.4]],
                   "points": [[0.1, 0.05, -0.8], [-0.2, -0.1, -0.75], [0.6, 0.6, -0.5], [0.0, 0.0, -0.95]]},
    "gripper": {"preset": "edu_dynamixel", "thetas": [[0.3, 0.4, 0.5], [0.35, 0.35, 0.35], [0.1, 0.9, 0.2]],
                "points": [[0.0, 0.0, -0.3], [0.03, -0.02, -0.31], [0.2, 0.2, -0.2], [0.0, 0.0, -0.4]]},
    "stepper": {"preset": "printer_stepper", "thetas": [[0.35, 0.35, 0.35], [0.8, 0.1, 0.4], [0.0, 1.0, 0.5]],
                "points": [[0.0, 0.0, -0.3], [0.05, 0.05, -0.32], [-0.1, 0.02, -0.28], [0.0, 0.2, -0.4]]},
}


def close(a, b, tol=1e-9, path=""):
    if isinstance(a, dict):
        assert set(a) == set(b), path
        for k in a:
            close(a[k], b[k], tol, path + "." + str(k))
    elif isinstance(a, (list, tuple)):
        assert b is not None and len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b)):
            close(x, y, tol, "%s[%d]" % (path, i))
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        if math.isinf(a) or math.isinf(b):
            assert a == b, path
        else:
            assert abs(a - b) <= tol * max(1.0, abs(a)), "%s: py=%r js=%r" % (path, a, b)
    else:
        assert a == b, "%s: py=%r js=%r" % (path, a, b)


def try_(f, *a):
    try:
        return list(f(*a))
    except kin.Unreachable:
        return None


def py_results(c):
    d = DeltaDesign.preset(c["preset"])
    r = {"summary": summary(d), "validate": d.validate()}
    r["fk"] = [try_(kin.fk, d, t) for t in c["thetas"]]
    r["ik"] = [try_(kin.ik, d, p) for p in c["points"]]
    lims = []
    for p in c["points"]:
        rep = kin.limit_report(d, p)
        rep["theta"] = list(rep["theta"]) if rep["theta"] else None
        lims.append(rep)
    r["limits"] = lims
    r["jac"] = kin.jacobian(d, c["thetas"][0])
    r["cond"] = kin.condition_number(d, c["thetas"][0])
    r["passive"] = [list(x) for x in kin.passive_angles(d, c["thetas"][0])]
    r["torque"] = dyn.joint_torques(d, c["thetas"][0], [1, -2, 0.5], None, [0.5, -1, 2])
    r["avail"] = [dyn.available_torque(d, w) for w in (0, 1, 5, 50)]
    r["inertia_ratio"] = dyn.inertia_ratio(d)
    profs = []
    for k in tr.PROFILES:
        p = tr.Profile(k, 0.2, 0.8, 6)
        profs.append([p.T, list(p.at(p.T * 0.3)), list(p.at(p.T * 0.8))])
    r["profiles"] = profs
    ap = tr.arch_path(c["points"][0], c["points"][1], 0.03)
    r["arch"] = [ap.length, list(ap.point(ap.length * 0.37)), list(ap.point(ap.length * 0.81))]
    cyl = kin.work_cylinder(d, 0.25 * (d.upper_arm + d.forearm))
    cyl["profile"] = [list(x) for x in cyl["profile"][:12]]
    r["cyl"] = cyl
    r["scene"] = scene.default_scene(d, "pick_place")
    r["scene_conv"] = scene.default_scene(d, "conveyor")
    r["urdf"] = urdf.generate(d)
    r["jstate"] = urdf.joint_state(d, c["thetas"][0])
    r["jstate_tool"] = urdf.joint_state(d, c["thetas"][0], 1)
    frames = [{"t": k * 0.01, "q": [0.3 + 0.2 * math.sin(3 * k * 0.01), 0.35, 0.4 - 0.1 * k * 0.01], "tool": 0}
              for k in range(41)]
    r["evaluate"] = dyn.evaluate(d, dyn.analyze(d, frames))
    if d.tool_spec["kind"] != "pen" and not d.tool_spec["ferrous_only"]:
        from deltarobot import DeltaRobot
        rb = DeltaRobot(d, scene=r["scene_conv"])
        rb.home()
        rb.wait(3.0)
        part = sorted([q for q in rb.parts() if q["on"] == "conveyor"], key=lambda q: -q["x"])[0]
        got = rb.track_pick(part)
        fr = rb.backend.frames
        r["track"] = {"got": got, "t": rb.time, "n": len(fr), "q": list(rb.q), "mid": fr[int(len(fr) * 0.8)]}
    return r


@pytest.fixture(scope="module")
def js():
    node = shutil.which("node")
    if not node:
        pytest.skip("node not installed")
    res = subprocess.run([node, str(ROOT / "tests" / "js" / "parity.mjs")], input=json.dumps(CASES),
                         capture_output=True, text=True, encoding="utf-8", check=True)
    return json.loads(res.stdout)


@pytest.mark.parametrize("name", list(CASES))
def test_parity(js, name):
    py = json.loads(json.dumps(py_results(CASES[name])))
    j = js[name]
    assert py["urdf"] == j["urdf"], "URDF text differs"
    for key in py:
        if key == "urdf":
            continue
        tol = 1e-6 if key.startswith("scene") else 1e-5 if key == "track" else 1e-7
        close(py[key], j[key], tol, key)
