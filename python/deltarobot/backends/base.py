"""Backend base classes."""

from __future__ import annotations

import sys
import time
from typing import Any, Dict, List, Sequence, Tuple

Vec = Tuple[float, float, float]


def safe_print(msg: str) -> None:
    """print() that never crashes on consoles without UTF-8 (e.g. Korean Windows cp949)."""
    try:
        print(msg)
    except UnicodeEncodeError:
        enc = getattr(sys.stdout, "encoding", None) or "ascii"
        print(msg.encode(enc, errors="replace").decode(enc, errors="replace"))


def max_joint_speed(stream: Sequence[Tuple[float, Vec]], q0: Sequence[float], t0: float) -> float:
    """Largest |dq/dt| along a sampled stream (rad/s)."""
    best = 0.0
    pt, pq = t0, q0
    for t, q in stream:
        dt = t - pt
        if dt > 1e-9:
            for i in range(3):
                best = max(best, abs(q[i] - pq[i]) / dt)
        pt, pq = t, q
    return best


class Backend:
    """Interface used by :class:`deltarobot.robot.DeltaRobot`."""

    realtime = False
    enforce_speed = False     # hardware backends refuse moves faster than the motor

    def __init__(self) -> None:
        self.robot: Any = None
        self._estop = False
        self._warned: set = set()

    # lifecycle
    def open(self, robot: Any) -> Dict[str, Any]:
        """Connect; may return {"design": ..., "scene": ...} offered by the other side."""
        return {}

    def start(self, robot: Any) -> None:
        self.robot = robot

    def close(self) -> None:
        pass

    # clock
    def now(self) -> float:
        raise NotImplementedError

    # safety
    def check(self) -> None:
        if self._estop:
            from ..robot import EmergencyStop
            raise EmergencyStop("비상정지 요청을 받았습니다 — 프로그램을 멈춥니다")

    def check_stream(self, stream: Sequence[Tuple[float, Vec]]) -> None:
        d = self.robot.design
        limit = float(d.motor_spec["max_speed"]) / d.ratio
        w = max_joint_speed(stream, self.robot.q, self.now())
        if w > limit:
            msg = ("관절 속도 %.2f rad/s 가 모터 한계 %.2f rad/s(감속 후)를 넘습니다 — "
                   "set_speed()로 속도를 낮추세요" % (w, limit))
            if self.enforce_speed:
                from ..robot import WorkspaceError
                raise WorkspaceError(msg)
            if "speed" not in self._warned:
                self._warned.add("speed")
                safe_print("경고: " + msg)

    # motion
    def stream(self, stream: List[Tuple[float, Vec]], tool: int) -> None:
        raise NotImplementedError

    def wait(self, seconds: float) -> None:
        raise NotImplementedError

    def set_tool(self, on: int) -> None:
        raise NotImplementedError


class RealtimeBackend(Backend):
    """Streams samples paced by the wall clock; subclasses implement send()."""

    realtime = True

    def __init__(self) -> None:
        super().__init__()
        self._t0 = time.monotonic()

    def now(self) -> float:
        return time.monotonic() - self._t0

    def send(self, q: Vec, tool: int) -> None:
        raise NotImplementedError

    def stream(self, stream: List[Tuple[float, Vec]], tool: int) -> None:
        for t, q in stream:
            self.check()
            delay = t - self.now()
            if delay > 0:
                time.sleep(delay)
            self.send(q, tool)

    def wait(self, seconds: float) -> None:
        end = self.now() + seconds
        while True:
            self.check()
            left = end - self.now()
            if left <= 0:
                break
            time.sleep(min(0.05, left))

    def set_tool(self, on: int) -> None:
        self.send(self.robot.q, on)
