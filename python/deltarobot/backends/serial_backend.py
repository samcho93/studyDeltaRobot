"""serial backend: stream motor angles to the Arduino firmware (firmware/delta_servo).

Line protocol (115200 baud, ASCII, one command per line):
    J <deg1> <deg2> <deg3>   target motor angles [deg], 0 = arm horizontal, + = down
    T <0|1>                  tool off / on
    E                        emergency stop (servos detach until reset)
    ?                        status -> "OK <deg1> <deg2> <deg3> <tool>"
Requires: pip install pyserial
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from .base import RealtimeBackend, Vec


class SerialBackend(RealtimeBackend):
    enforce_speed = True

    def __init__(self, port: str = "COM3", baud: int = 115200, rate: float = 50.0):
        super().__init__()
        self.port, self.baud = port, baud
        self.min_period = 1.0 / rate
        self._last = -1.0
        self.ser: Any = None

    def open(self, robot: Any) -> Dict[str, Any]:
        try:
            import serial  # type: ignore
        except ImportError as e:  # pragma: no cover
            raise RuntimeError("serial 백엔드에는 pyserial이 필요합니다: pip install pyserial") from e
        import time as _t
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        _t.sleep(2.0)   # most Arduinos reset when the port opens
        self.ser.reset_input_buffer()
        return {}

    def _write(self, line: str) -> None:
        self.ser.write((line + "\n").encode("ascii"))

    def send(self, q: Vec, tool: int) -> None:
        now = self.now()
        if now - self._last < self.min_period:
            return
        self._last = now
        self._write("J %.2f %.2f %.2f" % tuple(math.degrees(v) for v in q))

    def set_tool(self, on: int) -> None:
        self._write("T %d" % int(on))

    def stream(self, stream: List[Tuple[float, Vec]], tool: int) -> None:
        super().stream(stream, tool)
        self._last = -1.0
        self.send(stream[-1][1], tool)     # always deliver the final target

    def estop(self) -> None:
        if self.ser is not None:
            self._write("E")

    def close(self) -> None:
        if self.ser is not None:
            try:
                self._write("T 0")
            finally:
                self.ser.close()
                self.ser = None
