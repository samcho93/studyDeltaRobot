"""websim backend: PC Python drives the web simulator in real time.

    robot = DeltaRobot(backend="websim")        # design/scene taken from the open simulator
    robot = DeltaRobot("edu_servo", backend="websim", port=8765)

Python runs a tiny WebSocket server on 127.0.0.1; in the simulator open the
"연결" tab and press "PC Python 연결".  Protocol: docs/PROTOCOL.md.
Requires:  pip install "websockets>=12"
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional, Set

from .base import RealtimeBackend, Vec

ALLOWED_ORIGINS = ("https://samcho93.github.io", "http://localhost", "http://127.0.0.1",
                   "https://localhost", "https://127.0.0.1")


def origin_allowed(origin: Optional[str]) -> bool:
    """Browser pages may connect only from the course site or a local preview."""
    if not origin:
        return True               # non-browser clients (tests, ROS tools)
    if origin == "null":
        return False
    return any(origin == o or origin.startswith(o + ":") or origin.startswith(o + "/")
               for o in ALLOWED_ORIGINS)


class WebSimBackend(RealtimeBackend):
    def __init__(self, host: str = "127.0.0.1", port: int = 8765, wait_connect: float = 60.0,
                 quiet: bool = False):
        super().__init__()
        self.host, self.port = host, port
        self.wait_connect = wait_connect
        self.quiet = quiet
        self._clients: Set[Any] = set()
        self._lock = threading.Lock()
        self._hello: Optional[Dict[str, Any]] = None
        self._got_hello = threading.Event()
        self._server: Any = None

    def _log(self, msg: str) -> None:
        if not self.quiet:
            print("[websim] " + msg)

    # ------------------------------------------------------------ server
    def _handler(self, ws: Any) -> None:
        with self._lock:
            self._clients.add(ws)
        self._log("시뮬레이터 연결됨")
        try:
            for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                kind = msg.get("type")
                if kind == "hello":
                    self._hello = msg
                    self._got_hello.set()
                elif kind == "estop":
                    self._estop = True
                    self._log("비상정지 수신!")
        finally:
            with self._lock:
                self._clients.discard(ws)
            self._log("시뮬레이터 연결 끊김")

    def open(self, robot: Any) -> Dict[str, Any]:
        try:
            from websockets.sync.server import serve
        except ImportError as e:  # pragma: no cover
            raise RuntimeError('websim 백엔드에는 websockets>=12 가 필요합니다: pip install "websockets>=12"') from e

        def process_request(connection: Any, request: Any) -> Any:
            if not origin_allowed(request.headers.get("Origin")):
                return connection.respond(403, "origin not allowed\n")
            return None

        self._server = serve(self._handler, self.host, self.port, process_request=process_request)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self._log("ws://%s:%d 에서 대기 중 — 시뮬레이터 '연결' 탭에서 [PC Python 연결]을 누르세요"
                  % (self.host, self.port))
        if not self._got_hello.wait(self.wait_connect):
            self.close()
            raise TimeoutError("%.0f 초 안에 시뮬레이터가 연결되지 않았습니다" % self.wait_connect)
        hello = self._hello or {}
        return {"design": hello.get("design"), "scene": hello.get("scene")}

    def start(self, robot: Any) -> None:
        super().start(robot)
        self._broadcast({"type": "design", "design": robot.design.to_dict(), "scene": robot.scene.data})
        time.sleep(0.2)
        self._t0 = time.monotonic()
        self.send(robot.q, 0)

    def _broadcast(self, msg: Dict[str, Any]) -> None:
        data = json.dumps(msg)
        with self._lock:
            clients = list(self._clients)
        for ws in clients:
            try:
                ws.send(data)
            except Exception:  # noqa: BLE001 - client went away
                pass

    def send(self, q: Vec, tool: int) -> None:
        self._broadcast({"type": "state", "t": round(self.now(), 4), "q": [round(v, 6) for v in q],
                         "tool": int(tool)})

    def close(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server = None
