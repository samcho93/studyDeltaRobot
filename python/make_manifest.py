"""Write python/manifest.json — the files the Playground mounts into Pyodide.

Run after changing anything under python/deltarobot/:   python python/make_manifest.py
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SKIP = {"serial_backend.py", "ros2.py", "websim.py"}   # PC-only backends


def main() -> None:
    files = []
    for p in sorted((ROOT / "deltarobot").rglob("*")):
        if p.is_dir() or "__pycache__" in p.parts or p.name in SKIP:
            continue
        if p.suffix in (".py", ".json"):
            files.append(p.relative_to(ROOT).as_posix())
    h = hashlib.sha256()
    for rel in files:
        h.update(rel.encode())
        h.update((ROOT / rel).read_bytes().replace(b"\r\n", b"\n"))
    out = {"version": h.hexdigest()[:12], "files": files}
    (ROOT / "manifest.json").write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8", newline="\n")
    print("manifest: %d files, version %s" % (len(files), out["version"]))


if __name__ == "__main__":
    main()
