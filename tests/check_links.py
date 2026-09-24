#!/usr/bin/env python3
"""Internal link checker for the built static site (stdlib only).

Scans index.html and every .html under lessons/, tools/, sim/ for href/src (and srcset)
links, and reports:
  * internal files that do not exist
  * #anchors that do not exist in the target page (id="..." or <a name="...">)
External links (http:, https:, //, mailto:, data:, javascript:, ...) are skipped.
Absolute paths starting with /studyGo2/ (the GitHub Pages base) or / are resolved
against the repository root.

Usage:  python tests/check_links.py [--quiet]
Exit code 1 if any broken link was found.
"""
from __future__ import annotations

import os
import sys
from html.parser import HTMLParser
from typing import Dict, List, Optional, Set, Tuple
from urllib.parse import unquote, urlsplit

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCAN_DIRS = ("lessons", "tools", "sim")
SKIP_DIR_NAMES = {"node_modules", ".git", "vendor", "__pycache__"}
SITE_BASE = "/studyGo2/"
EXTERNAL_PREFIXES = ("http:", "https:", "//", "mailto:", "tel:", "data:", "javascript:",
                     "blob:", "ws:", "wss:", "about:")
LINK_ATTRS = {"href", "src", "poster", "data-src"}


class PageParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.links: List[Tuple[int, str]] = []
        self.anchors: Set[str] = set()
        self._in_template_script = 0

    def handle_starttag(self, tag: str, attrs: List[Tuple[str, Optional[str]]]) -> None:
        a = dict(attrs)
        if a.get("id"):
            self.anchors.add(a["id"])
        if tag == "a" and a.get("name"):
            self.anchors.add(a["name"])
        line = self.getpos()[0]
        for key, val in attrs:
            if val is None:
                continue
            if key in LINK_ATTRS:
                self.links.append((line, val.strip()))
            elif key == "srcset":
                for part in val.split(","):
                    url = part.strip().split(" ")[0]
                    if url:
                        self.links.append((line, url))

    handle_startendtag = handle_starttag


_cache: Dict[str, PageParser] = {}


def parse(path: str) -> PageParser:
    if path not in _cache:
        p = PageParser()
        with open(path, encoding="utf-8", errors="replace") as fh:
            p.feed(fh.read())
        _cache[path] = p
    return _cache[path]


def html_files() -> List[str]:
    files = []
    idx = os.path.join(ROOT, "index.html")
    if os.path.isfile(idx):
        files.append(idx)
    for d in SCAN_DIRS:
        base = os.path.join(ROOT, d)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [n for n in dirnames if n not in SKIP_DIR_NAMES]
            for fn in filenames:
                if fn.lower().endswith(".html"):
                    files.append(os.path.join(dirpath, fn))
    return sorted(files)


def is_external(url: str) -> bool:
    low = url.lower()
    return low.startswith(EXTERNAL_PREFIXES) or "${" in url or "{{" in url


def resolve(page: str, url: str) -> Tuple[Optional[str], str]:
    parts = urlsplit(url)
    path = unquote(parts.path)
    frag = unquote(parts.fragment)
    if not path:
        return page, frag
    if path.startswith(SITE_BASE):
        target = os.path.join(ROOT, path[len(SITE_BASE):])
    elif path.startswith("/"):
        target = os.path.join(ROOT, path.lstrip("/"))
    else:
        target = os.path.join(os.path.dirname(page), path)
    target = os.path.normpath(target)
    if os.path.isdir(target) or path.endswith("/"):
        target = os.path.join(target, "index.html")
    return target, frag


def main(argv: List[str]) -> int:
    quiet = "--quiet" in argv
    pages = html_files()
    broken: List[str] = []
    checked = 0
    for page in pages:
        rel_page = os.path.relpath(page, ROOT).replace(os.sep, "/")
        for line, url in parse(page).links:
            if not url or is_external(url):
                continue
            checked += 1
            target, frag = resolve(page, url)
            if target is None:
                continue
            if not target.startswith(ROOT):
                broken.append(f"{rel_page}:{line}: {url}  -> outside the repository")
                continue
            if not os.path.exists(target):
                broken.append(f"{rel_page}:{line}: {url}  -> missing file "
                              f"{os.path.relpath(target, ROOT).replace(os.sep, '/')}")
                continue
            if frag and target.lower().endswith(".html"):
                if frag not in parse(target).anchors:
                    broken.append(f"{rel_page}:{line}: {url}  -> missing anchor #{frag}")
    if not quiet or broken:
        print(f"check_links: {len(pages)} pages, {checked} internal links checked, "
              f"{len(broken)} broken")
    for b in broken:
        print("  BROKEN " + b)
    if broken:
        targets: Dict[str, int] = {}
        for b in broken:
            key = b.split("->", 1)[1].strip()
            targets[key] = targets.get(key, 0) + 1
        print("summary by target:")
        for key, n in sorted(targets.items(), key=lambda kv: (-kv[1], kv[0])):
            print(f"  {n:4d} x {key}")
    return 1 if broken else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
