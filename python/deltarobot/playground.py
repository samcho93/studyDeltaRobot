"""Playground runtime helpers (Pyodide only).

install() makes time.sleep / time.time / time.monotonic follow the virtual clock of the
active record backend, so a student's `time.sleep(0.5)` becomes 0.5 s of simulated time
instead of blocking the browser.
"""

from __future__ import annotations

import time as _time

from .backends import record

_real = {}


def _vnow() -> float:
    r = record.active()
    return r.now() if r is not None else 0.0


def _vsleep(seconds: float) -> None:
    r = record.active()
    if r is not None:
        r.robot.wait(seconds)


def install() -> None:
    if _real:
        return
    for name in ("sleep", "time", "monotonic", "perf_counter"):
        _real[name] = getattr(_time, name)
    _time.sleep = _vsleep
    _time.time = lambda: 1.7e9 + _vnow()
    _time.monotonic = _vnow
    _time.perf_counter = _vnow


def reset() -> None:
    record.reset()
