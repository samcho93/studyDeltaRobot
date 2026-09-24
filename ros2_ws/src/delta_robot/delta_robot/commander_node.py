"""delta_commander — Cartesian goals -> straight-line joint streams for delta_driver.

Subscribes
    /delta/goal           geometry_msgs/Point   TCP target [m] (tool tip, base frame)
    /joint_states         sensor_msgs/JointState (current motor angles from the driver)
    /delta/estop          std_msgs/Bool         True clears the queue
Publishes
    /delta/joint_command  sensor_msgs/JointState (100 Hz while busy)
    /delta/tool_command   std_msgs/Bool
    /delta/busy           std_msgs/Bool
Services
    /delta/tool           std_srvs/SetBool      tool on/off (queued after pending motion)
Parameters
    preset, design_file, profile (scurve), speed (0 = design default) [m/s], accel (0 = 10*speed) [m/s^2]

Goals received while a move is running are planned from the end of the queued motion.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Tuple, Union

from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from . import core
from .ros_util import run_node

QueueItem = Tuple[str, Union[core.Vec, bool]]   # ("q", (t1,t2,t3)) | ("tool", on)


class DeltaCommander(Node):
    def __init__(self) -> None:
        super().__init__("delta_commander")
        self.declare_parameter("preset", "edu_dynamixel")
        self.declare_parameter("design_file", "")
        self.declare_parameter("profile", "scurve")
        self.declare_parameter("speed", 0.0)
        self.declare_parameter("accel", 0.0)
        gp = self.get_parameter
        self.design = core.load_design(gp("preset").value, gp("design_file").value)

        self.q_now: Optional[core.Vec] = None      # from /joint_states
        self.queue: Deque[QueueItem] = deque()
        self.q_tail: Optional[core.Vec] = None     # last queued joint target
        self._busy_sent: Optional[bool] = None
        self._busy_heartbeat = core.Throttle(10.0)

        self.pub_cmd = self.create_publisher(JointState, "/delta/joint_command", 10)
        self.pub_tool = self.create_publisher(Bool, "/delta/tool_command", 10)
        self.pub_busy = self.create_publisher(Bool, "/delta/busy", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Point, "/delta/goal", self._on_goal, 10)
        self.create_subscription(Bool, "/delta/estop", self._on_estop, 10)
        self.create_service(SetBool, "/delta/tool", self._on_tool_srv)
        self.create_timer(core.DT, self._tick)
        self.get_logger().info("delta_commander ready: publish geometry_msgs/Point on /delta/goal")

    # ------------------------------------------------------------ inputs
    def _on_joint_states(self, msg: JointState) -> None:
        q = core.extract_motor_positions(msg.name, msg.position)
        if q is not None:
            self.q_now = q

    def _on_estop(self, msg: Bool) -> None:
        if msg.data and self.queue:
            self.queue.clear()
            self.q_tail = None
            self.get_logger().error("e-stop: motion queue cleared")

    def _on_goal(self, msg: Point) -> None:
        goal = (float(msg.x), float(msg.y), float(msg.z))
        start = self.q_tail if self.queue and self.q_tail is not None else self.q_now
        if start is None:
            self.get_logger().error("no /joint_states yet — is delta_driver running?")
            return
        gp = self.get_parameter
        try:
            samples = core.plan_line(self.design, start, goal, str(gp("profile").value),
                                     float(gp("speed").value) or None, float(gp("accel").value) or None)
        except core.PlanError as e:
            self.get_logger().error("goal (%.3f, %.3f, %.3f) rejected: %s" % (goal + (e,)))
            return
        for q in samples:
            self.queue.append(("q", q))
        if samples:
            self.q_tail = samples[-1]
        self.get_logger().info("goal (%.3f, %.3f, %.3f): %d samples (%.2f s)"
                               % (goal + (len(samples), len(samples) * core.DT)))

    def _on_tool_srv(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        on = bool(req.data)
        if self.queue:
            self.queue.append(("tool", on))
            res.message = "tool %s queued after current motion" % ("on" if on else "off")
        else:
            self.pub_tool.publish(Bool(data=on))
            res.message = "tool %s" % ("on" if on else "off")
        res.success = True
        return res

    # ------------------------------------------------------------ output
    def _tick(self) -> None:
        # publish at most one joint sample per tick; tool events are emitted in order
        while self.queue:
            kind, val = self.queue.popleft()
            if kind == "tool":
                self.pub_tool.publish(Bool(data=bool(val)))
                continue
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(core.JOINTS)
            js.position = [float(v) for v in val]  # type: ignore[union-attr]
            self.pub_cmd.publish(js)
            break
        busy = bool(self.queue)
        if not busy:
            self.q_tail = None
        now = self.get_clock().now().nanoseconds * 1e-9
        # publish on every change plus a 10 Hz heartbeat
        if busy != self._busy_sent or self._busy_heartbeat.ready(now):
            self.pub_busy.publish(Bool(data=busy))
            self._busy_sent = busy


def main(args: Optional[list] = None) -> None:
    run_node(DeltaCommander, args)


if __name__ == "__main__":
    main()
