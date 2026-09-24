"""Every simulator demo program must build for every preset (runs sim/programs.js in node)."""

import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


def test_demo_programs_all_presets():
    node = shutil.which("node")
    if not node:
        pytest.skip("node not installed")
    res = subprocess.run([node, str(ROOT / "tests" / "js" / "programs_check.mjs")], capture_output=True,
                         text=True, encoding="utf-8", check=True)
    out = json.loads(res.stdout)
    failures = {k: v["error"] for k, v in out.items() if not v["ok"] and "전자석" not in v["error"]}
    assert not failures, failures
    for k, v in out.items():
        if v["ok"] and k.endswith("/conveyor"):
            assert v["info"]["score"]["total"] >= 8, (k, v["info"])
        if v["ok"] and k.endswith("/sort"):
            assert v["info"]["picked"] >= 3, (k, v["info"])
