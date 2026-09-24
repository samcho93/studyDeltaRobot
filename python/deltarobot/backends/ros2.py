"""ros2 backend: publish motor commands to the delta_robot driver node.

Topics (see ros2_ws/src/delta_robot):
    /delta/joint_command  sensor_msgs/JointState  (motor1_joint..motor3_joint, rad)
    /delta/tool_command   std_msgs/Bool
    /delta/estop          std_msgs/Bool           (subscribed; True -> EmergencyStop)
    /delta/session        std_msgs/String         JSON {"design", "scene"} published once at start —
                                                  delta_web_bridge restarts its clock at that moment so
                                                  the web simulator's conveyor runs in step with this client
Requires a sourced ROS 2 environment (rclpy).
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict

from .base import RealtimeBackend, Vec

JOINTS = ["motor1_joint", "motor2_joint", "motor3_joint"]


class Ros2Backend(RealtimeBackend):
    enforce_speed = True

    def __init__(self, node_name: str = "deltarobot_client", namespace: str = ""):
        super().__init__()
        self.node_name, self.namespace = node_name, namespace
        self.node: Any = None

    def open(self, robot: Any) -> Dict[str, Any]:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from sensor_msgs.msg import JointState
            from rclpy.qos import DurabilityPolicy, QoSProfile
            from std_msgs.msg import Bool, String
        except ImportError as e:  # pragma: no cover
            raise RuntimeError("ros2 백엔드는 ROS 2 환경(source /opt/ros/<distro>/setup.bash)에서 실행하세요") from e
        if not rclpy.ok():
            rclpy.init()
        self._JointState, self._Bool, self._String = JointState, Bool, String
        self.node = rclpy.create_node(self.node_name, namespace=self.namespace)
        self.pub_q = self.node.create_publisher(JointState, "/delta/joint_command", 10)
        self.pub_tool = self.node.create_publisher(Bool, "/delta/tool_command", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_session = self.node.create_publisher(String, "/delta/session", latched)

        def on_estop(msg: Any) -> None:
            if msg.data:
                self._estop = True

        self.node.create_subscription(Bool, "/delta/estop", on_estop, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        threading.Thread(target=self._executor.spin, daemon=True).start()
        return {}

    def start(self, robot: Any) -> None:
        import json
        super().start(robot)
        msg = self._String()
        msg.data = json.dumps({"design": robot.design.to_dict(), "scene": robot.scene.data})
        self.pub_session.publish(msg)
        self._t0 = time.monotonic()      # t = 0 for this program == bridge clock reset

    def send(self, q: Vec, tool: int) -> None:
        msg = self._JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(v) for v in q]
        self.pub_q.publish(msg)

    def set_tool(self, on: int) -> None:
        m = self._Bool()
        m.data = bool(on)
        self.pub_tool.publish(m)

    def close(self) -> None:
        if self.node is not None:
            self._executor.shutdown()
            self.node.destroy_node()
            self.node = None
