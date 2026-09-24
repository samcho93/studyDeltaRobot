"""delta_web_bridge — lets the web simulator ("PC Python 연결" button) show the ROS robot.

Runs a WebSocket SERVER on ws://127.0.0.1:8765 speaking the same protocol as the
``websim`` backend (docs/PROTOCOL.md):

    browser -> {"type":"hello", ...}            bridge -> {"type":"design","design":..,"scene":..}
    bridge  -> {"type":"state","t","q","tool"}  (<= 50 Hz, from /joint_states + /delta/tool_state)
    browser -> {"type":"estop"}                 -> /delta/estop  std_msgs/Bool(True)
    browser -> {"type":"goal","xyz":[x,y,z]}    -> /delta/goal   geometry_msgs/Point (TCP, m)

Parameters: preset, design_file, scene ('pick_place'), host ('127.0.0.1'), port (8765).
Requires: pip install "websockets>=12"
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional, Set

from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from deltarobot.backends.websim import origin_allowed
from deltarobot.scene import default_scene

from . import core
from .ros_util import run_node


class DeltaWebBridge(Node):
    def __init__(self) -> None:
        super().__init__("delta_web_bridge")
        self.declare_parameter("preset", "edu_dynamixel")
        self.declare_parameter("design_file", "")
        self.declare_parameter("scene", "pick_place")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        gp = self.get_parameter
        self.design = core.load_design(gp("preset").value, gp("design_file").value)
        self.scene: Dict[str, Any] = default_scene(self.design, str(gp("scene").value))

        self._lock = threading.Lock()
        self._clients: Set[Any] = set()          # clients that said hello
        self._q: Optional[core.Vec] = None
        self._tool = False
        self._t0 = time.monotonic()
        self._throttle = core.Throttle(core.STATE_MAX_RATE)

        self.pub_estop = self.create_publisher(Bool, "/delta/estop", 10)
        self.pub_goal = self.create_publisher(Point, "/delta/goal", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Bool, "/delta/tool_state", self._on_tool_state, 10)
        self.create_timer(1.0 / core.STATE_MAX_RATE, self._tick)

        self._server: Any = None
        self._start_server(str(gp("host").value), int(gp("port").value))

    # ------------------------------------------------------------ websocket server
    def _start_server(self, host: str, port: int) -> None:
        try:
            from websockets.sync.server import serve
        except ImportError as e:
            raise RuntimeError('delta_web_bridge needs websockets>=12: pip install "websockets>=12"') from e

        def process_request(connection: Any, request: Any) -> Any:
            origin = request.headers.get("Origin")
            if not origin_allowed(origin):
                self.get_logger().warning("rejected connection from origin %r" % origin)
                return connection.respond(403, "origin not allowed\n")
            return None

        self._server = serve(self._handler, host, port, process_request=process_request)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.get_logger().info("web bridge on ws://%s:%d — press [PC Python 연결] in the simulator"
                               % (host, port))

    def _handler(self, ws: Any) -> None:
        log = self.get_logger()
        try:
            for raw in ws:
                msg = core.parse_client_message(raw)
                if msg is None:
                    continue
                kind = msg["type"]
                if kind == "hello":
                    ws.send(json.dumps(core.design_message(self.design, self.scene)))
                    with self._lock:
                        self._clients.add(ws)
                    log.info("simulator connected")
                elif kind == "estop":
                    self.pub_estop.publish(Bool(data=True))
                    log.error("e-stop from web simulator")
                elif kind == "goal":
                    xyz = core.goal_from_message(msg)
                    if xyz is None:
                        log.warning("malformed goal message: %s" % (msg,))
                        continue
                    self.pub_goal.publish(Point(x=xyz[0], y=xyz[1], z=xyz[2]))
        except Exception as e:  # noqa: BLE001 - connection errors end this client only
            log.debug("client error: %s" % e)
        finally:
            with self._lock:
                was = ws in self._clients
                self._clients.discard(ws)
            if was:
                log.info("simulator disconnected")

    # ------------------------------------------------------------ ROS side
    def _on_joint_states(self, msg: JointState) -> None:
        q = core.extract_motor_positions(msg.name, msg.position)
        if q is not None:
            self._q = q

    def _on_tool_state(self, msg: Bool) -> None:
        self._tool = bool(msg.data)

    def _tick(self) -> None:
        if self._q is None or not self._throttle.ready(time.monotonic()):
            return
        with self._lock:
            clients = list(self._clients)
        if not clients:
            return
        data = json.dumps(core.state_message(time.monotonic() - self._t0, self._q, self._tool))
        for ws in clients:
            try:
                ws.send(data)
            except Exception:  # noqa: BLE001 - client went away; handler cleans up
                pass

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server = None


def main(args: Optional[list] = None) -> None:
    run_node(DeltaWebBridge, args)


if __name__ == "__main__":
    main()
