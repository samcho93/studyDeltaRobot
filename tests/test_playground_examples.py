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
    assert len(EXAMPLES) >= 6


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
