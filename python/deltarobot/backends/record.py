"""Record backend: virtual clock, frames kept in memory.

Used by the Playground (Pyodide) — after the script ends the frames are handed to the
web simulator for playback — and on a PC for dry runs / motor analysis.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List, Tuple

from .base import Backend, Vec

RECORDERS: List["RecordBackend"] = []
HOLD_DT = 0.05
# set by the Playground: {"design": {...}, "scene": {...}} of the simulator on the page
DEFAULT_HELLO: Dict[str, Any] = {}


class RecordBackend(Backend):
    def __init__(self, time_limit: float = 600.0, verbose: bool = False):
        super().__init__()
        self.t = 0.0
        self.time_limit = time_limit
        self.verbose = verbose
        self.frames: List[Dict[str, Any]] = []
        RECORDERS.append(self)

    def open(self, robot: Any) -> Dict[str, Any]:
        return dict(DEFAULT_HELLO)

    def start(self, robot: Any) -> None:
        super().start(robot)
        self.frames.append({"t": 0.0, "q": list(robot.q), "tool": 0})

    def now(self) -> float:
        return self.t

    def _advance(self, t: float) -> None:
        if t > self.time_limit:
            raise RuntimeError("가상 시간 %.0f s 제한을 넘었습니다 (무한 루프인가요?)" % self.time_limit)
        self.t = t

    def stream(self, stream: List[Tuple[float, Vec]], tool: int) -> None:
        for t, q in stream:
            self.frames.append({"t": round(t, 5), "q": [round(v, 6) for v in q], "tool": tool})
        self._advance(stream[-1][0])
        if self.verbose:
            p = self.robot.fk(*stream[-1][1])
            print("[%.2fs] TCP -> (%.3f, %.3f, %.3f)" % (self.t, p[0], p[1], p[2]))

    def wait(self, seconds: float) -> None:
        end = self.t + seconds
        q = list(self.robot.q)
        t = self.t
        while t < end - 1e-9:
            t = min(end, t + HOLD_DT)
            self.frames.append({"t": round(t, 5), "q": q, "tool": self.robot.tool_state})
        self._advance(end)

    def set_tool(self, on: int) -> None:
        self.frames.append({"t": round(self.t, 5), "q": list(self.robot.q), "tool": on})

    def export(self) -> Dict[str, Any]:
        r = self.robot
        return {"type": "timeline", "design": r.design.to_dict(), "scene": r.scene.data,
                "frames": self.frames, "events": r.scene.events, "duration": self.t}


def export_last_json() -> str:
    """JSON of the most recently created recorder (used by the Playground worker)."""
    live = [r for r in RECORDERS if r.robot is not None]
    return json.dumps(live[-1].export() if live else None)


def active() -> Any:
    live = [r for r in RECORDERS if r.robot is not None]
    return live[-1] if live else None


def reset() -> None:
    RECORDERS.clear()
