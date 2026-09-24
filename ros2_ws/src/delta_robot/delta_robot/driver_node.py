"""delta_driver — motor driver node (mock motors or the Arduino serial firmware).

Subscribes
    /delta/joint_command  sensor_msgs/JointState  motor1_joint..motor3_joint [rad]
    /delta/tool_command   std_msgs/Bool
    /delta/estop          std_msgs/Bool           True = freeze, False = release
Publishes
    /joint_states         sensor_msgs/JointState  active + passive + effector_x/y/z
    /delta/tool_state     std_msgs/Bool
    /delta/tcp            geometry_msgs/PointStamped (frame base_link)
Parameters
    preset (edu_dynamixel), design_file (''), hardware ('mock'|'serial'),
    port ('/dev/ttyACM0'), baud (115200), rate (100.0 Hz)

Safety limits (theta range, FK validity, joint speed) are enforced HERE, not in clients.
With hardware:=serial the published joint_states are the *commanded* (rate-limited)
angles; the hobby servos give no position feedback.
"""

from __future__ import annotations

import time
from typing import Any, Optional

from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from deltarobot import kinematics as kin

from . import core
from .ros_util import run_node


class DeltaDriver(Node):
    def __init__(self) -> None:
        super().__init__("delta_driver")
        self.declare_parameter("preset", "edu_dynamixel")
        self.declare_parameter("design_file", "")
        self.declare_parameter("hardware", "mock")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("rate", 100.0)

        gp = self.get_parameter
        self.design = core.load_design(gp("preset").value, gp("design_file").value)
        self.hardware = str(gp("hardware").value)
        if self.hardware not in ("mock", "serial"):
            raise ValueError("hardware must be 'mock' or 'serial', got %r" % self.hardware)
        rate = float(gp("rate").value)
        if rate <= 0:
            raise ValueError("rate must be > 0")

        self.joints = core.RateLimitedJoints(self.design)
        self.tool = False
        self.watchdog = core.Watchdog(1.0)
        self.serial_throttle = core.Throttle(core.SERIAL_MAX_RATE)
        self.ser: Any = None
        self._serial_dirty = True
        if self.hardware == "serial":
            self._open_serial(str(gp("port").value), int(gp("baud").value))

        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_tool = self.create_publisher(Bool, "/delta/tool_state", 10)
        self.pub_tcp = self.create_publisher(PointStamped, "/delta/tcp", 10)
        self.create_subscription(JointState, "/delta/joint_command", self._on_command, 10)
        self.create_subscription(Bool, "/delta/tool_command", self._on_tool, 10)
        self.create_subscription(Bool, "/delta/estop", self._on_estop, 10)

        self._last_tick = time.monotonic()
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "delta_driver: motor=%s tool=%s hardware=%s vmax=%.2f rad/s theta=[%.2f, %.2f] rad"
            % (self.design.motor, self.design.tool, self.hardware, self.joints.vmax,
               self.design.theta_min, self.design.theta_max))

    # ------------------------------------------------------------ serial
    def _open_serial(self, port: str, baud: int) -> None:
        try:
            import serial  # type: ignore
        except ImportError as e:
            raise RuntimeError("hardware:=serial needs pyserial (pip install pyserial)") from e
        self.ser = serial.Serial(port, baud, timeout=0)
        time.sleep(2.0)                 # most Arduinos reset when the port opens
        self.ser.reset_input_buffer()
        self._write(core.SERIAL_RESET)  # re-attach servos if the firmware was e-stopped
        self._write(core.serial_tool_line(False))
        self.get_logger().info("serial port %s @ %d opened" % (port, baud))

    def _write(self, line: str) -> None:
        if self.ser is None:
            return
        try:
            self.ser.write((line + "\n").encode("ascii"))
        except Exception as e:  # noqa: BLE001 - keep the node alive, report once per burst
            self.get_logger().error("serial write failed: %s" % e, throttle_duration_sec=2.0)

    def _drain_serial(self) -> None:
        if self.ser is None:
            return
        try:
            data = self.ser.read(self.ser.in_waiting or 0)
        except Exception:  # noqa: BLE001
            return
        if data:
            for line in data.decode("ascii", "replace").splitlines():
                if line.strip():
                    self.get_logger().debug("firmware: " + line.strip())

    # ------------------------------------------------------------ callbacks
    def _on_command(self, msg: JointState) -> None:
        q = core.extract_motor_positions(msg.name, msg.position)
        if q is None:
            self.get_logger().warning("joint_command needs motor1_joint..motor3_joint positions",
                                      throttle_duration_sec=1.0)
            return
        if self.joints.frozen:
            self.get_logger().warning("e-stop active: command ignored (publish False on /delta/estop)",
                                      throttle_duration_sec=1.0)
            return
        err = self.joints.set_target(q)
        if err:
            self.get_logger().warning("command rejected: %s" % err, throttle_duration_sec=0.5)
            return
        self.watchdog.feed(time.monotonic())

    def _on_tool(self, msg: Bool) -> None:
        if self.joints.frozen and msg.data:
            self.get_logger().warning("e-stop active: tool command ignored")
            return
        self.tool = bool(msg.data)
        self._write(core.serial_tool_line(self.tool))

    def _on_estop(self, msg: Bool) -> None:
        if msg.data and not self.joints.frozen:
            self.joints.freeze()
            self.tool = False                 # the firmware also switches the tool off on 'E'
            self._write(core.SERIAL_ESTOP)
            self.get_logger().error("EMERGENCY STOP — holding position, commands ignored")
        elif not msg.data and self.joints.frozen:
            self.joints.release()
            self._write(core.SERIAL_RESET)
            self.serial_throttle.reset()
            self._serial_dirty = True
            self.get_logger().info("e-stop released")

    # ------------------------------------------------------------ loop
    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(0.1, now - self._last_tick)   # clamp after hiccups
        self._last_tick = now
        before = self.joints.q
        q = self.joints.step(dt)
        if q != before:
            self._serial_dirty = True
        if self.watchdog.expired(now):
            self.get_logger().info("no joint_command for 1 s — holding last target")

        if self.ser is not None and not self.joints.frozen and self._serial_dirty \
                and self.serial_throttle.ready(now):
            self._write(core.serial_joint_line(q))
            self._serial_dirty = False
        self._drain_serial()
        self._publish(q)

    def _publish(self, q: core.Vec) -> None:
        stamp = self.get_clock().now().to_msg()
        try:
            names, pos = core.joint_state_lists(self.design, q, 1 if self.tool else 0)
            tcp: Optional[core.Vec] = core.tcp_from_joints(self.design, q)
        except kin.Unreachable:
            names, pos, tcp = list(core.JOINTS), list(q), None
        js = JointState()
        js.header.stamp = stamp
        js.name = names
        js.position = pos
        self.pub_js.publish(js)
        self.pub_tool.publish(Bool(data=self.tool))
        if tcp is not None:
            pt = PointStamped()
            pt.header.stamp = stamp
            pt.header.frame_id = "base_link"
            pt.point.x, pt.point.y, pt.point.z = tcp
            self.pub_tcp.publish(pt)

    def shutdown(self) -> None:
        if self.ser is not None:
            try:
                self._write(core.serial_tool_line(False))
                self.ser.close()
            finally:
                self.ser = None


def main(args: Optional[list] = None) -> None:
    run_node(DeltaDriver, args)


if __name__ == "__main__":
    main()
