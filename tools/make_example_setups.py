"""Starting setup (design + scene) of every Playground example -> tools/playground-setups.json.

When the student picks an example, the Playground puts the embedded simulator into a
stand-by state showing the robot and work cell the example will use, before it is run.
Each example is executed once with the record backend; the robot it creates last (the one
that is played back) gives the setup. Examples that use the simulator's own design
(`DeltaRobot()` without a design) only store the scene kind.

    python tools/make_example_setups.py        # also run by build.py
"""

from __future__ import annotations

import contextlib
import io
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))
SRC = ROOT / "tools" / "playground-examples.js"
OUT = ROOT / "tools" / "playground-setups.json"


def examples() -> list:
    text = SRC.read_text(encoding="utf-8")
    return [(i, c.replace("\\`", "`")) for i, c in re.findall(r"id: '(\w+)'.*?code: `(.*?)`,\n", text, flags=re.S)]


def run_example(name: str, code: str) -> Tuple[str, Optional[Dict[str, Any]]]:
    """Execute one example; returns (stdout, setup)."""
    from deltarobot import DeltaDesign
    from deltarobot.backends import record
    # a marker design stands in for "the simulator's design" so we can tell whether the code chose one
    marker = DeltaDesign.preset("edu_servo").copy(effector_mass=0.0417).to_dict()
    record.reset()
    record.DEFAULT_HELLO.clear()
    record.DEFAULT_HELLO.update({"design": marker})
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            exec(compile(code, name + ".py", "exec"), {"__name__": "__main__"})
        rec = record.active()
    finally:
        record.DEFAULT_HELLO.clear()
    if rec is None:
        return buf.getvalue(), None
    robot = rec.robot
    scene = json.loads(json.dumps(robot.scene.data))
    if robot.design.to_dict() == marker:
        return buf.getvalue(), {"sim_design": True, "kind": scene.get("kind", "pick_place")}
    return buf.getvalue(), {"design": robot.design.to_dict(), "scene": scene}


def build() -> Dict[str, Any]:
    return {name: run_example(name, code)[1] for name, code in examples()}


def main() -> None:
    data = build()
    OUT.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")) + "\n", encoding="utf-8", newline="\n")
    print("setups: %d examples -> %s (%.0f kB)" % (len(data), OUT.relative_to(ROOT), OUT.stat().st_size / 1024))


if __name__ == "__main__":
    main()
