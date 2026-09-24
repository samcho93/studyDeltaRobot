"""Tests for the ROS 2 package that do NOT need ROS installed.

* delta_robot/core.py — pure logic used by the driver / commander / web bridge nodes
* the pick-and-place example, run on the record backend
* syntax check of every .py in the package, launch files parsed with ast
* package.xml / setup.py / rviz / firmware sanity checks
"""

import ast
import json
import math
import py_compile
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "ros2_ws" / "src" / "delta_robot"
sys.path.insert(0, str(ROOT / "python"))
sys.path.insert(0, str(PKG))

from deltarobot import DeltaDesign, DeltaRobot  # noqa: E402
from deltarobot import kinematics as kin  # noqa: E402
from deltarobot import urdf as durdf  # noqa: E402
from deltarobot.backends.base import max_joint_speed  # noqa: E402
from deltarobot.backends.websim import origin_allowed  # noqa: E402

from delta_robot import core  # noqa: E402


@pytest.fixture(scope="module")
def servo():
    return DeltaDesign.preset("edu_servo")


@pytest.fixture(scope="module")
def dxl():
    return DeltaDesign.preset("edu_dynamixel")


def test_core_does_not_import_rclpy():
    src = (PKG / "delta_robot" / "core.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    mods = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
           {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert not any(m and m.split(".")[0] in ("rclpy", "sensor_msgs", "std_msgs", "geometry_msgs") for m in mods)


# ---------------------------------------------------------------- design helpers
def test_load_design_preset_and_file(dxl):
    assert core.load_design("edu_dynamixel").to_dict() == dxl.to_dict()
    from_file = core.load_design("edu_servo", str(PKG / "config" / "design_example.json"))
    assert from_file.to_dict() == dxl.to_dict()     # file wins over preset
    with pytest.raises(KeyError):
        core.load_design("no_such_preset")


def test_joint_speed_limit(servo):
    assert core.joint_speed_limit(servo) == pytest.approx(float(servo.motor_spec["max_speed"]) / servo.ratio)
    stepper = DeltaDesign.preset("printer_stepper")
    assert core.joint_speed_limit(stepper) == pytest.approx(float(stepper.motor_spec["max_speed"]) / 4)


def test_joint_state_lists_match_urdf(dxl):
    q = (0.3, 0.5, 0.2)
    names, pos = core.joint_state_lists(dxl, q)
    # 3 motors + 6 passive (a rods) + 3 virtual effector + 1 gripper finger (edu_dynamixel has a gripper)
    assert len(names) == len(pos) == 13
    assert names[:3] == list(core.JOINTS) and pos[:3] == list(q)
    assert names[9:12] == ["effector_x", "effector_y", "effector_z"]
    assert pos[9:12] == pytest.approx(list(kin.fk(dxl, q)))
    assert names[12] == "gripper_finger_a" and pos[12] == 0.0
    _, closed = core.joint_state_lists(dxl, q, 1)
    assert closed[12] < 0
    # every name is a movable, non-mimic joint of the generated URDF
    root = ET.fromstring(durdf.generate(dxl))
    free = {j.get("name") for j in root.iter("joint")
            if j.get("type") != "fixed" and j.find("mimic") is None}
    assert set(names) == free


def test_tcp_from_joints(dxl):
    q = (0.4, 0.4, 0.4)
    p = kin.fk(dxl, q)
    assert core.tcp_from_joints(dxl, q) == pytest.approx((p[0], p[1], p[2] - dxl.tool_length))


def test_extract_motor_positions():
    names = ["effector_x", "motor3_joint", "motor1_joint", "motor2_joint"]
    assert core.extract_motor_positions(names, [9.0, 0.3, 0.1, 0.2]) == (0.1, 0.2, 0.3)
    assert core.extract_motor_positions(["motor1_joint", "motor2_joint"], [0.1, 0.2]) is None
    assert core.extract_motor_positions(list(core.JOINTS), [0.1, float("nan"), 0.2]) is None
    assert core.extract_motor_positions(list(core.JOINTS), [0.1, 0.2]) is None   # short position array


def test_check_joints(servo):
    assert core.check_joints(servo, (0.3, 0.3, 0.3)) is None
    assert "motor2" in core.check_joints(servo, (0.3, 2.0, 0.3))
    assert core.check_joints(servo, (0.3, 0.3)) is not None


# ---------------------------------------------------------------- mock motors
def test_rate_limited_tracking(servo):
    jt = core.RateLimitedJoints(servo)
    assert jt.q == core.home_joints(servo)
    vmax = core.joint_speed_limit(servo)
    assert jt.set_target((1.0, 0.35, 0.0)) is None
    dt = 0.01
    n = int(0.5 / (vmax * dt))                  # stays short of the 0.65 rad move
    prev = jt.q
    for _ in range(n):
        q = jt.step(dt)
        for a, b in zip(q, prev):
            assert abs(a - b) <= vmax * dt + 1e-12
        prev = q
    assert q[0] == pytest.approx(0.35 + n * vmax * dt)
    assert q[2] == pytest.approx(max(0.0, 0.35 - n * vmax * dt))
    assert q[1] == pytest.approx(0.35)
    for _ in range(200):
        jt.step(dt)
    assert jt.settled and jt.q == pytest.approx((1.0, 0.35, 0.0))
    assert jt.step(0.0) == jt.q


def test_rate_limited_clamps(servo):
    jt = core.RateLimitedJoints(servo)
    assert jt.set_target((5.0, -5.0, 0.3)) is None
    assert jt.target == (servo.theta_max, servo.theta_min, 0.3)
    assert jt.set_target((0.1, float("inf"), 0.1)) is not None
    assert jt.set_target((0.1, 0.2)) is not None


def test_rate_limited_rejects_fk_failure(servo):
    short = servo.copy(forearm=0.14)            # (0,0,0) cannot be assembled with this rod
    jt = core.RateLimitedJoints(short, q0=(0.7, 0.7, 0.7))
    err = jt.set_target((0.0, 0.0, 0.0))
    assert err and "forward kinematics" in err
    assert jt.target == (0.7, 0.7, 0.7)


def test_freeze_and_release(servo):
    jt = core.RateLimitedJoints(servo)
    jt.set_target((1.2, 1.2, 1.2))
    jt.step(0.05)
    jt.freeze()
    held = jt.q
    assert jt.set_target((0.0, 0.0, 0.0)) == "emergency stop active"
    for _ in range(10):
        jt.step(0.01)
    assert jt.q == held
    jt.release()
    assert jt.target == held                    # no jump back to the old target
    assert jt.set_target((0.5, 0.5, 0.5)) is None


def test_throttle():
    th = core.Throttle(50.0)
    assert th.ready(0.0)
    assert not th.ready(0.01)
    assert th.ready(0.02)
    # 100 Hz calls -> 50 Hz passes
    th = core.Throttle(50.0)
    passes = sum(th.ready(k * 0.01) for k in range(100))
    assert passes == 50
    th.reset()
    assert th.ready(0.991)


def test_watchdog():
    wd = core.Watchdog(1.0)
    assert not wd.expired(100.0)                # never before the first command
    wd.feed(0.0)
    assert not wd.expired(0.5)
    assert wd.expired(1.2)
    assert not wd.expired(5.0)                  # log once
    wd.feed(6.0)
    assert not wd.expired(6.5)
    assert wd.expired(7.1)


# ---------------------------------------------------------------- serial protocol
def test_serial_lines_match_library_backend():
    q = (0.1, math.radians(45), -0.2)
    assert core.serial_joint_line(q) == "J %.2f %.2f %.2f" % tuple(math.degrees(v) for v in q)
    assert core.serial_joint_line((0.0, math.pi / 4, math.pi / 2)) == "J 0.00 45.00 90.00"
    assert core.serial_tool_line(True) == "T 1" and core.serial_tool_line(False) == "T 0"
    assert (core.SERIAL_ESTOP, core.SERIAL_RESET, core.SERIAL_STATUS) == ("E", "R", "?")


def test_parse_status_line():
    q, tool = core.parse_status_line("OK 10.00 20.50 -5.00 1\r\n")
    assert q == pytest.approx((math.radians(10), math.radians(20.5), math.radians(-5)))
    assert tool == 1
    assert core.parse_status_line("ERR estop") is None
    assert core.parse_status_line("OK 1 2 x 0") is None


def test_serial_backend_has_reset():
    from deltarobot.backends.serial_backend import SerialBackend

    class FakeSerial:
        def __init__(self):
            self.lines = []

        def write(self, data):
            self.lines.append(data.decode("ascii"))

    b = SerialBackend()
    b.ser = FakeSerial()
    b.estop()
    b.reset()
    assert b.ser.lines == ["E\n", "R\n"]


# ---------------------------------------------------------------- commander planning
def test_plan_line_reaches_goal(dxl):
    q0 = core.home_joints(dxl)
    start = core.tcp_from_joints(dxl, q0)
    goal = (0.03, 0.02, start[2] - 0.03)
    qs = core.plan_line(dxl, q0, goal)
    assert len(qs) > 5
    assert core.tcp_from_joints(dxl, qs[-1]) == pytest.approx(goal, abs=1e-9)
    # straight line: every sample lies on the segment start -> goal
    for q in qs:
        p = core.tcp_from_joints(dxl, q)
        u = [goal[i] - start[i] for i in range(3)]
        w = [p[i] - start[i] for i in range(3)]
        cross = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
        assert math.sqrt(sum(c * c for c in cross)) < 1e-9
    # joint speed respects the motor
    stream = [(core.DT * (k + 1), q) for k, q in enumerate(qs)]
    assert max_joint_speed(stream, q0, 0.0) <= core.joint_speed_limit(dxl)


def test_plan_line_profiles_and_speed(dxl):
    q0 = core.home_joints(dxl)
    start = core.tcp_from_joints(dxl, q0)
    goal = (start[0] + 0.05, start[1], start[2])
    slow = core.plan_line(dxl, q0, goal, "trapezoid", speed=0.05, accel=0.5)
    fast = core.plan_line(dxl, q0, goal, "trapezoid", speed=0.2, accel=2.0)
    assert len(slow) > len(fast)
    with pytest.raises(core.PlanError):
        core.plan_line(dxl, q0, goal, "bogus")


def test_plan_line_rejects_outside(dxl):
    q0 = core.home_joints(dxl)
    with pytest.raises(core.PlanError, match="outside workspace"):
        core.plan_line(dxl, q0, (0.0, 0.0, 0.5))          # above the base
    with pytest.raises(core.PlanError, match="outside workspace"):
        core.plan_line(dxl, q0, (1.0, 0.0, -0.3))         # far out
    with pytest.raises(core.PlanError):
        core.plan_line(dxl, q0, (float("nan"), 0.0, -0.3))


def test_plan_line_rejects_path_through_bad_region(servo):
    # both ends are reachable (effector centre) but the straight line crosses a motor-limit region
    a, b = (-0.14, 0.0, -0.15), (0.04, -0.12, -0.13)
    assert kin.reachable(servo, a) and kin.reachable(servo, b)
    mid = tuple((a[i] + b[i]) / 2 for i in range(3))
    assert not kin.reachable(servo, mid)
    q0 = kin.ik(servo, a)
    goal_tcp = (b[0], b[1], b[2] - servo.tool_length)
    with pytest.raises(core.PlanError, match="path point"):
        core.plan_line(servo, q0, goal_tcp)
    # a normal move: every returned sample passes limit_report
    q0 = core.home_joints(servo)
    for th in core.plan_line(servo, q0, (0.02, -0.02, core.tcp_from_joints(servo, q0)[2] - 0.02)):
        assert kin.limit_report(servo, kin.fk(servo, th))["ok"]


def test_plan_line_rejects_too_fast(servo):
    q0 = core.home_joints(servo)
    start = core.tcp_from_joints(servo, q0)
    goal = (start[0] + 0.04, start[1], start[2] - 0.03)
    with pytest.raises(core.PlanError, match="joint speed"):
        core.plan_line(servo, q0, goal, "trapezoid", speed=50.0, accel=5000.0)
    assert core.plan_line(servo, q0, goal, "trapezoid", speed=50.0, accel=5000.0, check_speed=False)


def test_plan_line_bad_start(servo):
    with pytest.raises(core.PlanError, match="current joints"):
        core.plan_line(servo.copy(forearm=0.14), (0.0, 0.0, 0.0), (0.0, 0.0, -0.15))


# ---------------------------------------------------------------- web bridge protocol
def test_design_and_state_messages(dxl):
    from deltarobot.scene import default_scene
    scene = default_scene(dxl, "pick_place")
    msg = json.loads(json.dumps(core.design_message(dxl, scene)))
    assert msg["type"] == "design" and msg["design"] == dxl.to_dict() and msg["scene"]["kind"] == "pick_place"
    st = json.loads(json.dumps(core.state_message(1.234567, (0.1, 0.2, 0.3), True)))
    assert st == {"type": "state", "t": 1.2346, "q": [0.1, 0.2, 0.3], "tool": 1}
    assert core.state_message(0, (0, 0, 0), False)["tool"] == 0


def test_parse_client_message():
    assert core.parse_client_message('{"type":"hello","design":{}}')["type"] == "hello"
    assert core.parse_client_message(b'{"type":"estop"}') == {"type": "estop"}
    for bad in ("not json", "[1,2]", '{"kind":"x"}', '{"type":3}', b"\xff\xfe", None):
        assert core.parse_client_message(bad) is None


def test_goal_from_message():
    assert core.goal_from_message({"type": "goal", "xyz": [0.03, 0, -0.25]}) == (0.03, 0.0, -0.25)
    for bad in ([0, 0], [0, "a", 0], [0, float("inf"), 0], "xyz", None, [True, 0, 0]):
        assert core.goal_from_message({"type": "goal", "xyz": bad}) is None


def test_origin_check_is_shared():
    assert origin_allowed(None)
    assert origin_allowed("https://samcho93.github.io")
    assert origin_allowed("http://localhost:8000")
    assert not origin_allowed("null")
    assert not origin_allowed("https://evil.example")
    src = (PKG / "delta_robot" / "web_bridge_node.py").read_text(encoding="utf-8")
    assert "from deltarobot.backends.websim import origin_allowed" in src


# ---------------------------------------------------------------- example on the record backend
@pytest.mark.parametrize("preset", ["edu_servo", "edu_dynamixel"])
def test_pick_place_demo_record(preset, capsys):
    from delta_robot.examples.pick_place_demo import pick_place
    robot = DeltaRobot(preset, backend="record", scene="pick_place")
    assert pick_place(robot, 3) == 3
    assert robot.scene.score()["total"] == 3
    frames = robot.backend.frames
    stream = [(f["t"], tuple(f["q"])) for f in frames[1:]]
    w = max_joint_speed(stream, frames[0]["q"], frames[0]["t"])
    # the ros2 backend refuses moves faster than the motor -> demo must stay below
    assert w <= core.joint_speed_limit(robot.design)


# ---------------------------------------------------------------- package files
PY_FILES = sorted(p for p in PKG.rglob("*.py"))


@pytest.mark.parametrize("path", PY_FILES, ids=lambda p: str(p.relative_to(PKG)))
def test_package_py_compiles(path, tmp_path):
    py_compile.compile(str(path), cfile=str(tmp_path / "x.pyc"), doraise=True)


@pytest.mark.parametrize("name", ["display.launch.py", "bringup.launch.py"])
def test_launch_files_parse(name):
    tree = ast.parse((PKG / "launch" / name).read_text(encoding="utf-8"))
    funcs = {n.name for n in tree.body if isinstance(n, ast.FunctionDef)}
    assert "generate_launch_description" in funcs


def test_launch_util_declares_arguments():
    src = (PKG / "delta_robot" / "launch_util.py").read_text(encoding="utf-8")
    ast.parse(src)
    for arg in ("preset", "design_file", "use_rviz", "web_bridge", "hardware", "port"):
        assert 'DeclareLaunchArgument("%s"' % arg in src
    assert "OpaqueFunction" in src and "robot_description" in src and "delta.rviz" in src


def test_package_xml():
    root = ET.parse(PKG / "package.xml").getroot()
    assert root.findtext("name") == "delta_robot"
    deps = {e.text for e in root if e.tag.endswith("depend")}
    for d in ("rclpy", "sensor_msgs", "std_msgs", "std_srvs", "geometry_msgs", "robot_state_publisher",
              "rviz2", "launch", "launch_ros"):
        assert d in deps
    assert root.find("export/build_type").text == "ament_python"
    assert (PKG / "resource" / "delta_robot").exists()


def test_setup_entry_points():
    src = (PKG / "setup.py").read_text(encoding="utf-8")
    ast.parse(src)
    for ep in ("delta_driver = delta_robot.driver_node:main",
               "delta_commander = delta_robot.commander_node:main",
               "delta_web_bridge = delta_robot.web_bridge_node:main",
               "delta_pick_place_demo = delta_robot.examples.pick_place_demo:main"):
        assert ep in src
    for mod in ("driver_node", "commander_node", "web_bridge_node"):
        tree = ast.parse((PKG / "delta_robot" / (mod + ".py")).read_text(encoding="utf-8"))
        assert "main" in {n.name for n in tree.body if isinstance(n, ast.FunctionDef)}


def test_rviz_config():
    yaml = pytest.importorskip("yaml")
    cfg = yaml.safe_load((PKG / "rviz" / "delta.rviz").read_text(encoding="utf-8"))
    vm = cfg["Visualization Manager"]
    assert vm["Global Options"]["Fixed Frame"] == "base_link"
    classes = {d["Class"] for d in vm["Displays"]}
    assert {"rviz_default_plugins/RobotModel", "rviz_default_plugins/TF", "rviz_default_plugins/Grid"} <= classes
    model = next(d for d in vm["Displays"] if d["Class"].endswith("RobotModel"))
    assert model["Description Topic"]["Value"] == "/robot_description"


def test_firmware_protocol():
    ino = (ROOT / "firmware" / "delta_servo" / "delta_servo.ino").read_text(encoding="utf-8")
    for token in ("115200", "'J'", "'T'", "'E'", "'R'", "'?'", "OK ", "detach"):
        assert token in ino
    assert (ROOT / "firmware" / "README.md").exists()
