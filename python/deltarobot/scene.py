"""Work cell scene: table, parts, bins and an optional conveyor.

The scene is plain JSON so the web simulator, the Playground and PC Python share it:

    {"kind": "pick_place", "surface_z": -0.33,
     "parts": [{"id": "p1", "x": .., "y": .., "size": 0.03, "h": 0.018,
                "color": "red", "material": "plastic"}],
     "bins": [{"id": "A", "x": .., "y": .., "w": .., "d": .., "h": .., "color": "red"}],
     "conveyor": {"y": .., "width": .., "x_start": .., "x_end": .., "speed": ..,
                  "spawn": [{"t": 0.0, "y_off": 0.0, "color": "red", "material": "plastic"}, ...]}
                 | null}

Grab / release rules are mirrored in assets/js/delta/scene.js.
"""

from __future__ import annotations

import copy
import math
from typing import Any, Dict, List, Optional, Sequence

from . import kinematics as kin

GRAB_Z_TOL = 0.012
PART_SIZE = 0.03


def _round(v: float) -> float:
    """Same as JS Math.round(v * 1e4) / 1e4 (half up, not banker's rounding)."""
    return math.floor(v * 1e4 + 0.5) / 1e4


def default_scene(design, kind: str = "pick_place") -> Dict[str, Any]:
    """Scene sized to the design's work cylinder (same numbers as the web simulator)."""
    height = 0.25 * (design.upper_arm + design.forearm)
    cyl = kin.work_cylinder(design, height)
    rc = max(0.02, cyl["diameter"] / 2.0)
    surface = cyl["z_bottom"] - design.tool_length
    size = min(0.06, max(0.015, 0.12 * rc))
    h = 0.6 * size
    scene: Dict[str, Any] = {"kind": kind, "surface_z": _round(surface), "work_radius": _round(rc),
                             "work_top": _round(cyl["z_top"] - design.tool_length),
                             "parts": [], "bins": [], "conveyor": None}
    colors = ["red", "blue", "green"]
    if kind == "pick_place":
        k = 0
        for row in range(2):
            for col in range(3):
                scene["parts"].append({
                    "id": "p%d" % (k + 1),
                    "x": _round(-0.55 * rc + row * 0.22 * rc),
                    "y": _round((col - 1) * 0.3 * rc),
                    "size": _round(size), "h": _round(h),
                    "color": colors[col], "material": "steel" if row == 1 else "plastic"})
                k += 1
        for j, c in enumerate(colors):
            scene["bins"].append({"id": "ABC"[j], "x": _round(0.5 * rc), "y": _round((j - 1) * 0.36 * rc),
                                  "w": _round(0.3 * rc), "d": _round(0.3 * rc), "h": _round(0.8 * h),
                                  "color": c})
    elif kind == "conveyor":
        spawn = []
        offs = [0.0, 0.12, -0.12, 0.06, -0.06]
        for n in range(30):
            spawn.append({"t": _round(1.0 + 2.0 * n), "y_off": _round(offs[n % 5] * rc),
                          "color": colors[n % 2], "material": "plastic"})
        scene["conveyor"] = {"y": _round(-0.45 * rc), "width": _round(0.45 * rc),
                             "x_start": _round(-1.3 * rc), "x_end": _round(1.3 * rc),
                             "speed": _round(0.2 * rc), "size": _round(size), "h": _round(h),
                             "spawn": spawn}
        for j, c in enumerate(colors[:2]):
            scene["bins"].append({"id": "AB"[j], "x": _round((j - 0.5) * 0.7 * rc), "y": _round(0.5 * rc),
                                  "w": _round(0.35 * rc), "d": _round(0.3 * rc), "h": _round(0.8 * h),
                                  "color": c})
    elif kind == "drawing":
        scene["paper"] = {"x": 0.0, "y": 0.0, "w": _round(1.4 * rc), "d": _round(1.4 * rc)}
    return scene


class Scene:
    """Mutable scene state driven by tool on/off events."""

    def __init__(self, data: Optional[Dict[str, Any]] = None):
        self.data = copy.deepcopy(data) if data else {"kind": "empty", "surface_z": -1.0,
                                                       "parts": [], "bins": [], "conveyor": None}
        self.surface_z = float(self.data.get("surface_z", -1.0))
        self._parts: List[Dict[str, Any]] = []
        for p in self.data.get("parts", []):
            q = dict(p)
            q.update(mode="table", z=self.surface_z)
            self._parts.append(q)
        conv = self.data.get("conveyor")
        if conv:
            for n, s in enumerate(conv.get("spawn", [])):
                self._parts.append({"id": "c%d" % (n + 1), "size": conv["size"], "h": conv["h"],
                                    "color": s.get("color", "red"), "material": s.get("material", "plastic"),
                                    "mode": "conveyor", "x0": conv["x_start"], "t0": s["t"],
                                    "y": conv["y"] + s.get("y_off", 0.0), "z": self.surface_z})
        self.held: Optional[Dict[str, Any]] = None
        self.events: List[Dict[str, Any]] = []

    # ------------------------------------------------------------ queries
    def _pos(self, p: Dict[str, Any], t: float) -> Optional[List[float]]:
        mode = p["mode"]
        if mode == "gone" or mode == "held":
            return None
        if mode == "conveyor":
            if t < p["t0"]:
                return None
            conv = self.data["conveyor"]
            x = p["x0"] + conv["speed"] * (t - p["t0"])
            if x > conv["x_end"]:
                return None
            return [x, p["y"], p["z"]]
        return [p["x"], p["y"], p["z"]]

    def parts(self, t: float = 0.0) -> List[Dict[str, Any]]:
        """Visible (not held) parts at time t: id, x, y, z (bottom), top, size, color, material, on."""
        out = []
        for p in self._parts:
            pos = self._pos(p, t)
            if pos is None:
                continue
            out.append({"id": p["id"], "x": pos[0], "y": pos[1], "z": pos[2], "top": pos[2] + p["h"],
                        "size": p["size"], "h": p["h"], "color": p["color"], "material": p["material"],
                        "on": p["mode"]})
        return out

    def bins(self) -> List[Dict[str, Any]]:
        return [dict(b) for b in self.data.get("bins", [])]

    def bin(self, bin_id: str) -> Dict[str, Any]:
        for b in self.data.get("bins", []):
            if b["id"] == bin_id:
                return dict(b)
        raise KeyError("no bin %r" % bin_id)

    def conveyor(self) -> Optional[Dict[str, Any]]:
        c = self.data.get("conveyor")
        return dict(c) if c else None

    def score(self) -> Dict[str, Any]:
        """Parts per bin and how many ended in the bin of the same colour."""
        per: Dict[str, int] = {b["id"]: 0 for b in self.data.get("bins", [])}
        correct = 0
        for p in self._parts:
            if p["mode"] == "bin":
                per[p["bin"]] = per.get(p["bin"], 0) + 1
                if p.get("bin_color") == p["color"]:
                    correct += 1
        return {"per_bin": per, "correct": correct, "total": sum(per.values())}

    # ------------------------------------------------------------ tool events
    def grab(self, tcp: Sequence[float], t: float, tool: Dict[str, Any]) -> Optional[str]:
        """Tool switched on at TCP position; returns the id of the attached part."""
        if self.held is not None or tool.get("kind") == "pen":
            return None
        best, best_d = None, 1e9
        for p in self._parts:
            pos = self._pos(p, t)
            if pos is None:
                continue
            if tool.get("ferrous_only") and p["material"] != "steel":
                continue
            dxy = math.hypot(tcp[0] - pos[0], tcp[1] - pos[1])
            top = pos[2] + p["h"]
            if tool.get("kind") == "gripper":
                zok = pos[2] - 0.002 <= tcp[2] <= top + GRAB_Z_TOL
            else:
                zok = abs(tcp[2] - top) <= GRAB_Z_TOL
            if zok and dxy <= max(0.012, 0.45 * p["size"]) and dxy < best_d:
                best, best_d = p, dxy
        if best is None:
            self.events.append({"t": t, "type": "grab_miss"})
            return None
        best["mode"] = "held"
        self.held = best
        self.events.append({"t": t, "type": "grab", "id": best["id"]})
        return best["id"]

    def release(self, tcp: Sequence[float], t: float) -> Optional[str]:
        p = self.held
        if p is None:
            return None
        self.held = None
        x, y = tcp[0], tcp[1]
        p["x"], p["y"] = x, y
        for b in self.data.get("bins", []):
            if abs(x - b["x"]) <= b["w"] / 2 and abs(y - b["y"]) <= b["d"] / 2:
                p.update(mode="bin", z=self.surface_z + 0.003, bin=b["id"], bin_color=b.get("color"))
                self.events.append({"t": t, "type": "place", "id": p["id"], "bin": b["id"]})
                return p["id"]
        conv = self.data.get("conveyor")
        if conv and abs(y - conv["y"]) <= conv["width"] / 2 and conv["x_start"] <= x <= conv["x_end"]:
            p.update(mode="conveyor", x0=x, t0=t, y=y, z=self.surface_z)
        else:
            p.update(mode="table", z=self.surface_z)
        self.events.append({"t": t, "type": "drop", "id": p["id"]})
        return p["id"]
