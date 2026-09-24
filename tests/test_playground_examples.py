"""Run every Playground example (tools/playground-examples.js) with the record backend."""

import contextlib
import io
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

SRC = (ROOT / "tools" / "playground-examples.js").read_text(encoding="utf-8")
EXAMPLES = re.findall(r"id: '(\w+)'.*?code: `(.*?)`,\n", SRC, flags=re.S)


def test_examples_found():
    assert len(EXAMPLES) >= 18
    assert len({e[0] for e in EXAMPLES}) == len(EXAMPLES)


@pytest.mark.parametrize("name,code", EXAMPLES, ids=[e[0] for e in EXAMPLES])
def test_example_runs(name, code):
    from deltarobot.backends import record
    record.reset()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        exec(compile(code.replace("\\`", "`"), name + ".py", "exec"), {"__name__": "__main__"})
    out = buf.getvalue()
    assert "Traceback" not in out
    assert "실패" not in out, out
    if name == "sort":
        assert "'total': 0" not in out, out
    if name == "conveyor":
        m = re.search(r"'total': (\d+)", out)
        assert m and int(m.group(1)) >= 8, out
    expect = {
        "tray": "트레이 12/12",
        "reject": "놓친 불량 0개",
        "pcb": "배치 8/8",
        "vision": "'correct': 6",
        "kitting": "C-red",
        "safe": "'total': 6",
        "motor": "선정 (",
    }
    if name in expect:
        assert expect[name] in out, out
    if name == "safe":
        assert "번째 시도에 집음" in out, out
