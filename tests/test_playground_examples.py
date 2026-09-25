"""Run every Playground example (tools/playground-examples.js) with the record backend.

Also checks tools/playground-setups.json (stand-by view of each example) is up to date.
"""

import json
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))
sys.path.insert(0, str(ROOT / "tools"))

import make_example_setups as setups_mod  # noqa: E402

EXAMPLES = setups_mod.examples()
SETUPS = json.loads((ROOT / "tools" / "playground-setups.json").read_text(encoding="utf-8"))


def test_examples_found():
    assert len(EXAMPLES) >= 18
    assert len({e[0] for e in EXAMPLES}) == len(EXAMPLES)


@pytest.mark.parametrize("name,code", EXAMPLES, ids=[e[0] for e in EXAMPLES])
def test_example_runs(name, code):
    out, setup = setups_mod.run_example(name, code)
    assert "Traceback" not in out
    assert "실패" not in out, out
    # stand-by setup shown when the example is picked must match what the code really uses
    assert name in SETUPS, "python tools/make_example_setups.py 를 실행하세요"
    assert json.loads(json.dumps(setup)) == SETUPS[name], "playground-setups.json 이 낡았습니다 — python build.py"
    if name == "sort":
        assert "'total': 0" not in out, out
    if name == "conveyor":
        m = re.search(r"'total': (\d+)", out)
        assert m and int(m.group(1)) >= 1, out
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
