"""Generate the SVG figures used by the design-track chapters (d01-d11).

Run from the repo root:  python content/figures/gen_design.py
Every curve is computed with the deltarobot library (python/deltarobot) — no hand-drawn data.
Only CSS variables / classes from assets/css/main.css are used so the figures follow the
light/dark theme.  Writes only files named d0*.svg / d1*.svg.
"""
from __future__ import annotations

import math
import re
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent.parent
sys.path.insert(0, str(ROOT / "python"))

from deltarobot import DeltaDesign, DeltaRobot, default_scene  # noqa: E402
from deltarobot import dynamics as dyn  # noqa: E402
from deltarobot import kinematics as kin  # noqa: E402
from deltarobot import trajectory as traj  # noqa: E402
from deltarobot import urdf  # noqa: E402

COLORS = ["var(--accent)", "var(--danger)", "var(--warn)", "var(--task)", "var(--info)", "var(--text-dim)"]


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Svg:
    def __init__(self, name: str, w: int, h: int, label: str):
        assert name.startswith(("d0", "d1")), name
        self.name, self.w, self.h, self.label = name, w, h, label
        self.parts: list = []
        self.mid = "arr-" + name

    def add(self, s: str) -> None:
        self.parts.append("  " + s)

    def box(self, x, y, w, h, hi=False, rx=10):
        self.add('<rect class="%s" x="%g" y="%g" width="%g" height="%g" rx="%g"/>'
                 % ("box-hi" if hi else "box", x, y, w, h, rx))

    def rect(self, x, y, w, h, color, fill_opacity=0.12, dash=False, width=1.5):
        self.add('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" style="fill:%s;fill-opacity:%g;stroke:%s"'
                 ' stroke-width="%g"%s/>' % (x, y, w, h, color, fill_opacity, color, width,
                                             ' stroke-dasharray="6 4"' if dash else ""))

    def text(self, x, y, s, cls="lbl", anchor="middle", extra=""):
        self.add('<text class="%s" x="%.1f" y="%.1f" text-anchor="%s"%s>%s</text>'
                 % (cls, x, y, anchor, extra, esc(s)))

    def ctext(self, x, y, s, color, anchor="start", size=12):
        self.add('<text x="%.1f" y="%.1f" text-anchor="%s" font-size="%g" style="fill:%s">%s</text>'
                 % (x, y, anchor, size, color, esc(s)))

    def line(self, x1, y1, x2, y2, hi=False, arrow=False, width=1.6, dash=False):
        self.add('<path class="%s" d="M%.1f %.1f L%.1f %.1f" stroke-width="%g"%s%s/>'
                 % ("ln-hi" if hi else "ln", x1, y1, x2, y2, width,
                    ' stroke-dasharray="5 4"' if dash else "",
                    ' marker-end="url(#%s%s)"' % (self.mid, "-hi" if hi else "") if arrow else ""))

    def path(self, d, hi=False, width=1.6, arrow=False, dash=False):
        self.add('<path class="%s" d="%s" stroke-width="%g"%s%s/>'
                 % ("ln-hi" if hi else "ln", d, width,
                    ' stroke-dasharray="5 4"' if dash else "",
                    ' marker-end="url(#%s%s)"' % (self.mid, "-hi" if hi else "") if arrow else ""))

    def cpath(self, d, color, width=2.0, dash="", fill="none", opacity=1.0):
        self.add('<path d="%s" style="stroke:%s;fill:%s;fill-opacity:%g" stroke-width="%g"'
                 ' stroke-linejoin="round" stroke-linecap="round"%s/>'
                 % (d, color, fill, opacity, width, ' stroke-dasharray="%s"' % dash if dash else ""))

    def dot(self, x, y, r, color):
        self.add('<circle cx="%.1f" cy="%.1f" r="%g" style="fill:%s"/>' % (x, y, r, color))

    def circle(self, cx, cy, r, cls="box"):
        self.add('<circle class="%s" cx="%.1f" cy="%.1f" r="%g"/>' % (cls, cx, cy, r))

    def write(self) -> None:
        defs = (
            '  <defs>\n'
            '    <marker id="{m}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            '<path d="M0 0 L10 5 L0 10 z" style="fill:var(--text-faint)"/></marker>\n'
            '    <marker id="{m}-hi" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            '<path d="M0 0 L10 5 L0 10 z" style="fill:var(--accent)"/></marker>\n'
            '  </defs>'.format(m=self.mid))
        body = "\n".join(self.parts)
        svg = ('<svg viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="%s">\n%s\n%s\n</svg>\n'
               % (self.w, self.h, esc(self.label), defs, body))
        with open(OUT / (self.name + ".svg"), "w", encoding="utf-8", newline="\n") as fh:
            fh.write(svg)
        print("wrote", self.name + ".svg")


class Axes:
    """Minimal x-y plot area inside an Svg."""

    def __init__(self, svg: Svg, x, y, w, h, xr, yr, logy=False):
        self.s, self.x, self.y, self.w, self.h = svg, x, y, w, h
        self.xr, self.yr, self.logy = xr, yr, logy

    def X(self, v):
        return self.x + (v - self.xr[0]) / (self.xr[1] - self.xr[0]) * self.w

    def Y(self, v):
        if self.logy:
            v = max(v, self.yr[0])
            t = (math.log10(v) - math.log10(self.yr[0])) / (math.log10(self.yr[1]) - math.log10(self.yr[0]))
        else:
            t = (v - self.yr[0]) / (self.yr[1] - self.yr[0])
        return self.y + self.h - t * self.h

    def frame(self, xticks, yticks, xlabel="", ylabel="", xfmt="%g", yfmt="%g", grid=True):
        s = self.s
        eps = 1e-9
        xticks = [v for v in xticks if self.xr[0] - eps <= v <= self.xr[1] + eps]
        yticks = [v for v in yticks if self.yr[0] - eps <= v <= self.yr[1] + eps]
        for v in yticks:
            yy = self.Y(v)
            if grid:
                s.line(self.x, yy, self.x + self.w, yy, width=0.6, dash=True)
            s.text(self.x - 6, yy + 4, yfmt % v, "tag", anchor="end")
        for v in xticks:
            xx = self.X(v)
            if grid:
                s.line(xx, self.y, xx, self.y + self.h, width=0.6, dash=True)
            s.text(xx, self.y + self.h + 16, xfmt % v, "tag")
        s.line(self.x, self.y + self.h, self.x + self.w, self.y + self.h, width=1.2)
        s.line(self.x, self.y, self.x, self.y + self.h, width=1.2)
        if xlabel:
            s.text(self.x + self.w / 2, self.y + self.h + 34, xlabel, "sub")
        if ylabel:
            s.text(self.x, self.y - 10, ylabel, "sub", anchor="start")

    def plot(self, xs, ys, color, width=2.0, dash=""):
        pts = []
        for a, b in zip(xs, ys):
            a = min(max(a, self.xr[0]), self.xr[1])
            if not self.logy:
                b = min(max(b, self.yr[0]), self.yr[1])
            else:
                b = min(max(b, self.yr[0]), self.yr[1])
            pts.append("%.1f %.1f" % (self.X(a), self.Y(b)))
        if pts:
            self.s.cpath("M" + " L".join(pts), color, width, dash)

    def hline(self, v, color, label="", dash="6 4", anchor="end"):
        self.s.cpath("M%.1f %.1f L%.1f %.1f" % (self.x, self.Y(v), self.x + self.w, self.Y(v)), color, 1.4, dash)
        if label:
            xx = self.x + self.w - 4 if anchor == "end" else self.x + 6
            self.s.ctext(xx, self.Y(v) - 5, label, color, anchor=anchor, size=11)


def legend(svg: Svg, x, y, items, dy=18):
    for k, (label, color, dash) in enumerate(items):
        yy = y + k * dy
        svg.cpath("M%.1f %.1f L%.1f %.1f" % (x, yy, x + 24, yy), color, 2.4, dash)
        svg.text(x + 30, yy + 4, label, "tag", anchor="start")


# ====================================================================== helpers
def adept_frames(design, across, speed=None, accel=None, up=0.025, n=1, radius=0.0):
    r = DeltaRobot(design, scene="empty")
    if speed:
        r.set_speed(speed, accel)
    z = r.scene.data["surface_z"] + 0.02
    r.move_to(-across / 2, 0, z)
    n0 = len(r.backend.frames)
    for _ in range(n):
        r.arch_to(across / 2, 0, z, height=up, radius=radius)
        r.arch_to(-across / 2, 0, z, height=up, radius=radius)
    return r, n0


# ====================================================================== d01
def d01_sim_tabs():
    s = Svg("d01-sim-tabs", 760, 300, "시뮬레이터 다섯 탭과 설계 반복 흐름")
    tabs = [("설계", ["프리셋·R r L l w", "모터·감속기·툴", "[설계 검증]"]),
            ("조그", ["TCP x/y/z", "θ1~θ3 슬라이더", "조건수·한계 표시"]),
            ("작업", ["장면·데모", "프로파일·속도", "재생·배속"]),
            ("분석", ["θ·ω·τ 그래프", "피크·RMS·속도", "사이클·작업영역"]),
            ("연결", ["PC Python", "JSON·URDF", "공유 링크"])]
    w, gap = 136, 14
    x0 = (760 - (5 * w + 4 * gap)) / 2
    for i, (title, lines) in enumerate(tabs):
        x = x0 + i * (w + gap)
        s.box(x, 60, w, 120, hi=(i in (0, 3)))
        s.text(x + w / 2, 88, title)
        for k, t in enumerate(lines):
            s.text(x + w / 2, 115 + k * 20, t, "sub")
        if i < 4:
            s.line(x + w, 120, x + w + gap - 1, 120, arrow=True)
    s.text(380, 36, "상단: 비상정지 · 테마 · 도구 링크      URL: ?preset= · ?scene= · #design=", "tag")
    xa = x0 + 3 * (w + gap) + w / 2
    xb = x0 + w / 2
    s.path("M%.1f 180 L%.1f 230 L%.1f 230 L%.1f 184" % (xa, xa, xb, xb), hi=True, arrow=True, width=1.8)
    s.text((xa + xb) / 2, 248, "✖ 가 있으면 설계 탭으로 돌아가 수정 → 다시 검증", "tag")
    s.text(380, 282,"설계값은 브라우저에 저장되어 URDF 뷰어 · Playground가 같은 설계를 씁니다", "sub")
    s.write()


def d01_graph_reading():
    d = DeltaDesign.preset("edu_servo")
    r, n0 = adept_frames(d, 0.18, speed=0.6, accel=6.0)
    data = dyn.analyze(d, r.backend.frames)[n0:]
    t0 = data[0]["t"]
    ts = [f["t"] - t0 for f in data]
    m = d.motor_spec
    n, eta = d.ratio, d.efficiency
    s = Svg("d01-graph-reading", 760, 480,"분석 탭 그래프 읽기: 모터 속도와 토크, 한계선")
    T = ts[-1]
    a1 = Axes(s, 70, 40, 640, 130, (0, T), (0, 8))
    a1.frame([0, 0.5, 1.0, 1.5], [0, 2, 4, 6, 8], "", "모터 속도 |ω| [rad/s]", yfmt="%g")
    for i in range(3):
        a1.plot(ts, [abs(f["theta_d"][i]) * n for f in data], COLORS[i], 1.8)
    a1.hline(m["max_speed"], "var(--danger)", "최고 속도 %.1f rad/s" % m["max_speed"])
    a2 = Axes(s, 70, 230, 640, 130, (0, T), (0, 1.2))
    a2.frame([0, 0.5, 1.0, 1.5], [0, 0.3, 0.6, 0.9, 1.2], "시간 [s]", "모터 토크 |τ| [N·m]", yfmt="%.1f")
    for i in range(3):
        a2.plot(ts, [abs(f["tau"][i]) / (n * eta) for f in data], COLORS[i], 1.8)
        a2.plot(ts, [dyn.available_torque(d, f["theta_d"][i] * n) for f in data], COLORS[i], 1.1, "3 3")
    a2.hline(m["peak_torque"], "var(--danger)", "피크(정지) 토크 %.2f N·m" % m["peak_torque"])
    a2.hline(m["rated_torque"], "var(--warn)", "정격 토크 %.2f N·m (RMS 비교)" % m["rated_torque"])
    legend(s, 520, 420, [("모터 1", COLORS[0], ""), ("모터 2", COLORS[1], ""), ("모터 3", COLORS[2], "")])
    legend(s, 90, 420, [("실선: 필요 토크 |τ|", "var(--text-dim)", ""),
                                 ("점선: 그 순간 속도에서 낼 수 있는 토크", "var(--text-dim)", "3 3")])
    s.write()


# ====================================================================== d02
def d02_adept_cycle():
    s = Svg("d02-adept-cycle", 760, 450,"Adept 사이클: 25 mm 상승, 305 mm 이동, 25 mm 하강 후 복귀")
    path = traj.arch_path((-0.1525, 0, 0), (0.1525, 0, 0), 0.025)
    pts = [path.point(path.length * k / 200) for k in range(201)]
    # top: path shape, z exaggerated x4
    ax = Axes(s, 80, 40, 600, 110, (-0.17, 0.17), (-0.005, 0.035))
    sx = lambda v: ax.X(v)
    sy = lambda v: ax.Y(v)
    s.cpath("M" + " L".join("%.1f %.1f" % (sx(p[0]), sy(p[2])) for p in pts), "var(--accent)", 2.6)
    s.line(sx(-0.17), sy(0), sx(0.17), sy(0), width=0.8, dash=True)
    s.dot(sx(-0.1525), sy(0), 4, "var(--info)")
    s.dot(sx(0.1525), sy(0), 4, "var(--warn)")
    s.text(sx(-0.1525), sy(0) + 18, "집기 (pick)", "tag")
    s.text(sx(0.1525), sy(0) + 18, "놓기 (place)", "tag")
    s.line(sx(-0.1525), sy(0.031), sx(0.1525), sy(0.031), arrow=True, width=1.2)
    s.text(0 + sx(0), sy(0.031) - 6, "305 mm", "lbl")
    s.line(sx(-0.165), sy(0), sx(-0.165), sy(0.025), arrow=True, width=1.2)
    s.text(sx(-0.165) - 8, sy(0.0125) + 4, "25 mm", "tag", anchor="end")
    zoom = (ax.h / (ax.yr[1] - ax.yr[0])) / (ax.w / (ax.xr[1] - ax.xr[0]))
    s.text(380, 186, "모서리 반지름 = 높이/2 (기본값) · z 방향 %.1f배 확대" % zoom, "tag")
    # bottom: speed profile of one round trip (industrial_picker defaults)
    d = DeltaDesign.preset("industrial_picker")
    r = DeltaRobot(d, scene="empty")
    prof = traj.Profile(r.profile, path.length, r.speed, r.accel)
    T = prof.T
    ts = [T * k / 200 for k in range(201)]
    vs = [prof.at(t)[1] for t in ts]
    a2 = Axes(s, 80, 230, 600, 140, (0, 2 * T), (0, 0.8))
    a2.frame([0, 0.25, 0.5, 0.75, 1.0, 1.25], [0, 0.2, 0.4, 0.6, 0.8], "시간 [s]", "TCP 속도 [m/s]", yfmt="%.1f")
    a2.plot(ts + [T + t for t in ts], vs + vs, "var(--accent)", 2.2)
    s.line(a2.X(T), a2.y, a2.X(T), a2.y + a2.h, width=1, dash=True)
    s.text(a2.X(T / 2), a2.y + 16, "가는 길", "tag")
    s.text(a2.X(1.5 * T), a2.y + 16, "돌아오는 길", "tag")
    s.text(380, 436,
           "industrial_picker 기본값 %.2f m/s, %.1f m/s², scurve → 1 사이클 %.2f s" % (r.speed, r.accel, 2 * T),
           "tag")
    s.write()


# ====================================================================== d03
def d03_ratio_effect():
    base = DeltaDesign.preset("edu_dynamixel")
    s = Svg("d03-ratio-effect", 760, 440, "l/L 비에 따른 작업영역 단면과 작업 실린더")
    ax = Axes(s, 80, 40, 480, 340, (0, 260), (-560, -80))
    ax.frame([0, 50, 100, 150, 200, 250], [-550, -450, -350, -250, -150], "반지름 [mm] (중심축 기준)",
             "이펙터 중심 z [mm]")
    items = []
    for k, ratio in enumerate((1.5, 2.0, 2.3, 2.5, 3.0)):
        d = base.copy(forearm=round(ratio * base.upper_arm, 4))
        c = kin.work_cylinder(d, 0.10)
        prof = [(z * 1000, rr * 1000) for z, rr in c["profile"] if rr > 0]
        col = COLORS[k]
        ax.plot([p[1] for p in prof], [p[0] for p in prof], col, 2.0)
        x0, x1 = ax.X(0), ax.X(c["diameter"] * 500)
        y0, y1 = ax.Y(c["z_top"] * 1000), ax.Y(c["z_bottom"] * 1000)
        s.rect(x0, y0, x1 - x0, y1 - y0, col, 0.08, dash=True, width=1.2)
        items.append(("%.1f · l %d → D %d" % (ratio, round(d.forearm * 1000), round(c["diameter"] * 1000)),
                      col, ""))
    s.text(585, 56, "l/L · l [mm] → D [mm]", "sub", anchor="start")
    legend(s, 585, 78, items, dy=24)
    s.text(585, 210, "실선: 높이별 최대 반지름", "tag", anchor="start")
    s.text(585, 228, "(한 원 전체가 도달 가능)", "tag", anchor="start")
    s.text(585, 252, "점선 사각형: H = 100 mm", "tag", anchor="start")
    s.text(585, 270, "작업 실린더 (반단면)", "tag", anchor="start")
    s.text(585, 300, "L = 130, R = 100, r = 35 mm", "tag", anchor="start")
    s.write()


# ====================================================================== d04
def d04_torque_speed():
    s = Svg("d04-torque-speed", 760, 490, "모터 종류별 속도-토크 모델과 출력축 환산 곡선")
    # left: normalised shapes
    ax = Axes(s, 60, 40, 280, 270, (0, 1.05), (0, 1.1))
    ax.frame([0, 0.25, 0.5, 0.75, 1.0], [0, 0.25, 0.5, 0.75, 1.0], "ω / ω_max", "가용 토크 / 피크 토크",
             xfmt="%.2f", yfmt="%.2f")
    kinds = [("rc_servo_mg996r", "RC · 스마트 서보 (직선)", COLORS[0], ""),
             ("stepper_nema17", "스테퍼 NEMA17 (풀아웃)", COLORS[2], ""),
             ("stepper_nema23", "스테퍼 NEMA23", COLORS[3], "4 3"),
             ("ac_servo_400w", "BLDC · AC 서보 (평탄)", COLORS[1], "")]
    for mid, label, col, dash in kinds:
        d = DeltaDesign().copy(motor=mid, gearbox="none", gear_ratio=1)
        wm, pk = d.motor_spec["max_speed"], d.motor_spec["peak_torque"]
        xs = [k / 200 * 1.05 for k in range(201)]
        ax.plot(xs, [dyn.available_torque(d, x * wm) / pk for x in xs], col, 2.2, dash)
    legend(s, 70, 385, [(k[1], k[2], k[3]) for k in kinds], dy=19)
    # right: output-side curves for typical drive trains (log y)
    ax2 = Axes(s, 460, 40, 270, 270, (0, 30), (0.1, 100), logy=True)
    ax2.frame([0, 10, 20, 30], [0.1, 1, 10, 100], "출력축 속도 [rad/s]", "출력축 가용 토크 [N·m] (로그)")
    combos = [("rc_servo_mg996r", "none", 1, "MG996R 직결"),
              ("dxl_xm430_w350", "none", 1, "XM430 직결"),
              ("stepper_nema17", "belt", 4, "NEMA17 + 벨트 4:1 (÷1.5)"),
              ("bldc_qdd", "planetary", 9, "BLDC + 유성 9:1"),
              ("ac_servo_200w", "planetary", 20, "AC 200 W + 유성 20:1"),
              ("ac_servo_400w", "planetary", 20, "AC 400 W + 유성 20:1")]
    for k, (mid, gb, n, label) in enumerate(combos):
        d = DeltaDesign().copy(motor=mid, gearbox=gb, gear_ratio=n)
        N, eta = d.ratio, d.efficiency
        xs = [30 * j / 300 for j in range(301)]
        ys = []
        for w in xs:
            a = dyn.available_torque(d, w * N) * N * eta
            if d.motor_spec["kind"] == "stepper":
                a /= dyn.STEPPER_MARGIN
            ys.append(a if a > 0 else 0.1)
        ax2.plot(xs, ys, COLORS[k], 2.0)
    legend(s, 470, 375, [(c[3], COLORS[k], "") for k, c in enumerate(combos)], dy=18)
    s.write()


def d04_ratio_sweep():
    base = DeltaDesign.preset("industrial_picker").copy(payload=0.5, motor="ac_servo_400w")
    ratios = [3, 5, 6, 9, 10, 15, 20, 30, 50]
    rows = []
    for n in ratios:
        d = base.copy(gear_ratio=n)
        r, _ = adept_frames(d, 0.305, speed=3.0, accel=30.0, radius=0.025)
        res = r.analyze()
        rows.append((n, res["peak_ratio"], res["rms_ratio"], res["speed_ratio"], res["inertia_ratio"]))
    s = Svg("d04-ratio-sweep", 760, 452,"감속비에 따른 피크·RMS·속도 사용률과 관성비")
    ax = Axes(s, 70, 40, 420, 290, (0, 55), (0, 2.5))
    ax.frame([0, 10, 20, 30, 40, 50], [0, 0.5, 1.0, 1.5, 2.0, 2.5], "감속비 N", "사용률 (1 이하 통과)", yfmt="%.1f")
    ax.hline(1.0, "var(--danger)", "한계 1.0")
    for k, (idx, label) in enumerate(((1, "피크 토크 비"), (2, "RMS 토크 비"), (3, "속도 비"))):
        ax.plot([r[0] for r in rows], [r[idx] for r in rows], COLORS[k], 2.2)
        for rr in rows:
            s.dot(ax.X(rr[0]), ax.Y(min(rr[idx], 2.5)), 3, COLORS[k])
    legend(s, 80, 392, [("피크 토크 비", COLORS[0], ""), ("RMS 토크 비", COLORS[1], "")], dy=18)
    legend(s, 250, 392, [("속도 비", COLORS[2], "")], dy=18)
    ax2 = Axes(s, 560, 40, 170, 290, (0, 55), (0.1, 1000), logy=True)
    ax2.frame([0, 25, 50], [0.1, 1, 10, 100, 1000], "감속비 N", "관성비 (로그)")
    ax2.plot([r[0] for r in rows], [r[4] for r in rows], COLORS[4], 2.2)
    for rr in rows:
        s.dot(ax2.X(rr[0]), ax2.Y(rr[4]), 3, COLORS[4])
    ax2.hline(10, "var(--warn)", "10", anchor="end")
    d = base.copy(gear_ratio=20)
    jl = (d.arm_inertia - d.drive_inertia + d.moving_plate_mass * d.upper_arm ** 2 / 3.0)
    nstar = math.sqrt(jl / d.motor_spec["rotor_inertia"])
    s.cpath("M%.1f %.1f L%.1f %.1f" % (ax2.X(nstar), ax2.y, ax2.X(nstar), ax2.y + ax2.h), "var(--text-dim)", 1.2, "3 3")
    s.text(ax2.X(nstar) + 4, ax2.y + 14, "N* ≈ %.0f" % nstar, "tag", anchor="start")
    s.text(380, 440, "AC 400 W + 유성, 0.5 kg, Adept 305 mm 사이클, 3 m/s · 30 m/s², 모서리 반지름 25 mm", "tag")
    s.write()
    return rows


# ====================================================================== d05
def d05_holding():
    s = Svg("d05-holding-force", 760, 414,"가속도에 따른 필요 유지력과 툴별 유지력")
    ax = Axes(s, 70, 40, 470, 300, (0, 80), (0, 110))
    ax.frame([0, 20, 40, 60, 80], [0, 20, 40, 60, 80, 100], "이동판 가속도 a [m/s²] (중력과 같은 방향으로 더해지는 최악의 경우)",
             "힘 [N]")
    SF = 2.0
    for k, m in enumerate((0.05, 0.2, 0.5)):
        xs = [0, 80]
        ax.plot(xs, [m * (9.81 + a) * SF for a in xs], COLORS[k], 2.4)
        s.ctext(ax.X(66), ax.Y(m * (9.81 + 66) * SF) + (14 if k == 0 else -7),
                "m = %g kg" % m, COLORS[k], anchor="middle", size=11)
    tools = DeltaDesign().copy()
    from deltarobot import catalog
    cat = catalog()["tools"]
    marks = [("suction_20", cat["suction_20"]["force"]), ("suction_40", cat["suction_40"]["force"]),
             ("gripper_servo 2μF", 2 * cat["gripper_servo"]["friction"] * cat["gripper_servo"]["force"]),
             ("gripper_pneumatic 2μF", 2 * cat["gripper_pneumatic"]["friction"] * cat["gripper_pneumatic"]["force"]),
             ("electromagnet (철)", cat["electromagnet"]["force"])]
    for k, (label, F) in enumerate(marks):
        yy = ax.Y(F)
        s.cpath("M%.1f %.1f L%.1f %.1f" % (ax.x, yy, ax.x + ax.w, yy), "var(--text-dim)", 1.1, "5 4")
        s.text(ax.x + ax.w + 8, yy + 4, "%s %.1f N" % (label, F), "tag", anchor="start")
    s.text(305, 402, "필요 유지력 F = m (g + a) · SF,  SF = %.1f" % SF, "tag")
    del tools
    s.write()


# ====================================================================== d06
def d06_profiles():
    dist, vmax, amax = 0.30, 2.0, 20.0
    s = Svg("d06-profiles", 760, 560, "네 가지 속도 프로파일의 위치·속도·가속도")
    profs = [traj.Profile(k, dist, vmax, amax) for k in traj.PROFILES]
    T = max(p.T for p in profs)
    panels = [(0, "위치 s [m]", (0, 0.32), [0, 0.1, 0.2, 0.3], "%.1f"),
              (1, "속도 ṡ [m/s]", (0, 2.2), [0, 1, 2], "%g"),
              (2, "가속도 s̈ [m/s²]", (-40, 40), [-40, -20, 0, 20, 40], "%g")]
    for k, (idx, lab, yr, yt, fmt) in enumerate(panels):
        ax = Axes(s, 80, 30 + k * 172, 500, 110, (0, T * 1.02), yr)
        ax.frame([0, 0.1, 0.2, 0.3], yt, "시간 [s]" if k == 2 else "", lab, xfmt="%.1f", yfmt=fmt)
        for j, p in enumerate(profs):
            ts = [p.T * i / 300 for i in range(301)]
            ax.plot(ts, [p.at(t)[idx] for t in ts], COLORS[j], 2.0)
    legend(s, 600, 60, [("%s  T = %.3f s" % (p.kind, p.T), COLORS[j], "") for j, p in enumerate(profs)], dy=22)
    s.text(600, 170, "d = 0.30 m", "tag", anchor="start")
    s.text(600, 188, "vmax = 2 m/s", "tag", anchor="start")
    s.text(600, 203, "amax = 20 m/s²", "tag", anchor="start")
    s.text(600, 230, "같은 한계값이라도", "tag", anchor="start")
    s.text(600, 246, "프로파일마다 T 가 다름", "tag", anchor="start")
    s.write()
    return profs


def d06_arch_paths():
    s = Svg("d06-arch-paths", 760, 260, "아치 궤적의 모서리 반지름")
    ax = Axes(s, 60, 30, 640, 170, (-0.17, 0.17), (-0.004, 0.045))
    for k, (h, rad, label) in enumerate(((0.025, 0.0, "높이 25 mm, 반지름 기본(12.5 mm)"),
                                         (0.025, 0.025, "높이 25 mm, 반지름 25 mm"),
                                         (0.04, 0.04, "높이 40 mm, 반지름 40 mm"),
                                         (0.025, 0.001, "높이 25 mm, 반지름 1 mm (거의 직각)"))):
        p = traj.arch_path((-0.1525, 0, 0), (0.1525, 0, 0), h, rad)
        pts = [p.point(p.length * i / 300) for i in range(301)]
        ax.plot([q[0] for q in pts], [q[2] for q in pts], COLORS[k], 2.2, "" if k < 3 else "4 3")
    s.line(ax.X(-0.17), ax.Y(0), ax.X(0.17), ax.Y(0), width=0.8, dash=True)
    legend(s, 80, 222, [("높이 25, 반지름 기본 12.5 mm", COLORS[0], ""), ("높이 25, 반지름 25 mm", COLORS[1], "")])
    legend(s, 420, 222, [("높이 40, 반지름 40 mm", COLORS[2], ""), ("높이 25, 반지름 1 mm", COLORS[3], "4 3")])
    s.text(700, 22, "z 방향 확대 (가로 305 mm)", "tag", anchor="end")
    s.write()


# ====================================================================== d07
def d07_urdf_tree():
    d = DeltaDesign.preset("edu_dynamixel")
    text = urdf.generate(d)
    joints = re.findall(r'<joint name="([^"]+)" type="([^"]+)"', text)
    links = re.findall(r'<link name="([^"]+)"', text)
    mimic = len(re.findall(r"<mimic ", text))
    kinds = {}
    for _, t in joints:
        kinds[t] = kinds.get(t, 0) + 1
    s = Svg("d07-urdf-tree", 760, 515, "deltarobot.urdf 가 만드는 트리 구조")
    s.box(300, 20, 160, 40, hi=True)
    s.text(380, 45, "base_link")
    # arm chain (arm 1 shown)
    chain = [("motor1_joint", "revolute · 능동"), ("upper_arm1", None), ("elbow1a_pitch", "revolute · 수동"),
             ("elbow1a_link", None), ("elbow1a_yaw", "revolute · 수동"), ("forearm1a", None)]
    x, y = 60, 90
    for k, (name, kind) in enumerate(chain):
        yy = y + k * 55
        if kind:
            s.text(x + 110, yy + 22, name, "lbl")
            s.text(x + 110, yy + 38, kind, "tag")
        else:
            s.box(x + 30, yy + 5, 160, 36)
            s.text(x + 110, yy + 28, name)
        if k < len(chain) - 1:
            s.line(x + 110, yy + 42, x + 110, yy + 60, arrow=True, width=1.2)
    s.path("M340 60 L340 75 L170 75 L170 94", arrow=True, width=1.2)
    s.text(380, 505, "왼쪽: 팔 1 의 a 로드.  b 로드는 elbow1b_pitch/yaw 가 a 를 mimic (평행사변형)", "tag")
    s.box(262, 250, 150, 70)
    s.text(337, 275, "× 3 팔", "lbl")
    s.text(337, 295, "motor2 · motor3 도", "tag")
    s.text(337, 310, "같은 모양", "tag")
    # virtual effector chain
    ev = [("effector_x", "prismatic · 가상"), ("effector_x_link", None), ("effector_y", "prismatic · 가상"),
          ("effector_y_link", None), ("effector_z", "prismatic · 가상"), ("effector", None),
          ("tool_joint → tool_link", "fixed"), ("tcp_joint → tcp", "fixed")]
    x2 = 460
    for k, (name, kind) in enumerate(ev):
        yy = 80 + k * 46
        if kind:
            s.text(x2 + 110, yy + 22, name, "lbl")
            s.text(x2 + 110, yy + 37, kind, "tag")
        else:
            s.box(x2 + 30, yy + 6, 160, 32, hi=(name == "effector"))
            s.text(x2 + 110, yy + 27, name)
        if k < len(ev) - 1:
            s.line(x2 + 110, yy + 40, x2 + 110, yy + 50, arrow=True, width=1.2)
    s.path("M420 60 L420 75 L570 75 L570 84", arrow=True, width=1.2)
    s.text(380, 482, "edu_dynamixel: 링크 %d · 관절 %d (revolute %d, prismatic %d, fixed %d) · mimic %d"
           % (len(links), len(joints), kinds.get("revolute", 0), kinds.get("prismatic", 0),
              kinds.get("fixed", 0), mimic), "sub")
    s.write()
    return len(links), len(joints), kinds, mimic


# ====================================================================== d08
def d08_workspace_fit():
    base = DeltaDesign.preset("edu_servo")
    v0 = base.copy(base_radius=0.07, effector_radius=0.03, upper_arm=0.08, forearm=0.16,
                   upper_arm_material="pla_print", forearm_material="pla_print", payload=0.01)
    v1 = base.copy(payload=0.01)
    s = Svg("d08-workspace-fit", 760, 400, "탁상형 델타: 1차 설계와 최종 설계의 작업영역, 요구 실린더")
    ax = Axes(s, 80, 40, 440, 300, (0, 220), (-360, -60))
    ax.frame([0, 50, 100, 150, 200], [-350, -300, -250, -200, -150, -100], "반지름 [mm]", "이펙터 중심 z [mm]")
    for k, (d, label) in enumerate(((v0, "v0: R70 r30 L80 l160"), (v1, "v1: R80 r30 L100 l220"))):
        c = kin.work_cylinder(d, 0.04)
        prof = [(z * 1000, rr * 1000) for z, rr in c["profile"] if rr > 0]
        col = COLORS[k * 3]
        ax.plot([p[1] for p in prof], [p[0] for p in prof], col, 2.2)
        x1 = ax.X(c["diameter"] * 500)
        s.rect(ax.X(0), ax.Y(c["z_top"] * 1000), x1 - ax.X(0),
               ax.Y(c["z_bottom"] * 1000) - ax.Y(c["z_top"] * 1000), col, 0.15, width=1.2)
        s.text(560, 80 + k * 70, label, "tag", anchor="start")
        s.ctext(560, 98 + k * 70, "D × H = %d × %d mm" % (round(c["diameter"] * 1000), round(c["height"] * 1000)),
                col, size=12)
    # requirement: D200 x H40 (drawn at the v1 cylinder)
    c1 = kin.work_cylinder(v1, 0.04)
    s.rect(ax.X(0), ax.Y(c1["z_top"] * 1000), ax.X(100) - ax.X(0), ax.Y(c1["z_top"] * 1000 - 40) - ax.Y(c1["z_top"] * 1000),
           "var(--danger)", 0.0, dash=True, width=1.6)
    s.ctext(560, 240, "요구: D 200 × H 40 mm (점선)", "var(--danger)", size=12)
    s.text(560, 262, "칠한 사각형 = 각 설계의", "tag", anchor="start")
    s.text(560, 278, "H = 40 mm 작업 실린더 (반단면)", "tag", anchor="start")
    s.write()


# ====================================================================== d09
def d09_cell():
    design = DeltaDesign.preset("industrial_picker").copy(motor="ac_servo_400w", gearbox="planetary",
                                                          gear_ratio=20, tool="suction_40", payload=0.5)
    scene = default_scene(design, "conveyor")
    for n, sp in enumerate(scene["conveyor"]["spawn"]):
        sp["t"] = 1.0 + 1.0 * n
    robot = DeltaRobot(design, scene=scene)
    robot.set_speed(3.0, 30.0)
    conv = robot.scene.conveyor()
    v, H, RAD, DWELL = conv["speed"], 0.025, 0.025, 0.05
    bins = {b["color"]: b for b in robot.bins()}
    place_z = robot.scene.surface_z + bins["red"]["h"] + 0.02
    picks = []

    def move_time(p0, p1):
        path = traj.arch_path(p0, p1, H, RAD)
        return traj.Profile(robot.profile, path.length, robot.speed, robot.accel).T

    robot.move_to(-0.25, conv["y"], place_z)
    while robot.time < 36.0:
        target = None
        for part in sorted(robot.parts(), key=lambda p: -p["x"]):
            if part["on"] != "conveyor":
                continue
            x = part["x"]
            for _ in range(3):
                x = part["x"] + v * move_time(robot.position, (x, part["y"], part["top"]))
            if robot.reachable(x, part["y"], part["top"]):
                target = (part, x)
                break
        if target is None:
            robot.wait(0.02)
            continue
        part, x = target
        robot.arch_to(x, part["y"], part["top"], height=H, radius=RAD)
        if robot.tool_on() is None:
            robot.wait(DWELL)
            robot.tool_off()
            continue
        picks.append((x, part["y"], part["color"]))
        robot.wait(DWELL)
        b = bins[part["color"]]
        robot.arch_to(b["x"], b["y"], place_z, height=H, radius=RAD)
        robot.tool_off()
        robot.wait(DWELL)
    # boundary of reachable region at the pick height
    ztop = scene["surface_z"] + conv["h"]
    s = Svg("d09-cell-layout", 760, 470, "컨베이어 피킹 셀 평면도와 실제 집기 위치")
    sc = 380.0 / 1.2   # px per m
    cx, cy = 330, 235
    X = lambda x: cx + x * sc
    Y = lambda y: cy - y * sc
    bpts = []
    for k in range(73):
        a = 2 * math.pi * k / 72
        lo, hi = 0.0, kin.workspace_bounds(design)[0]
        for _ in range(22):
            mid = (lo + hi) / 2
            if robot.reachable(mid * math.cos(a), mid * math.sin(a), ztop):
                lo = mid
            else:
                hi = mid
        bpts.append((lo * math.cos(a), lo * math.sin(a)))
    s.cpath("M" + " L".join("%.1f %.1f" % (X(p[0]), Y(p[1])) for p in bpts) + " Z", "var(--accent)", 1.8,
            fill="var(--accent)", opacity=0.07)
    rc = scene["work_radius"]
    s.cpath("M%.1f %.1f a%.1f %.1f 0 1 0 %.1f 0 a%.1f %.1f 0 1 0 %.1f 0" % (
        X(-rc), Y(0), rc * sc, rc * sc, 2 * rc * sc, rc * sc, rc * sc, -2 * rc * sc), "var(--text-dim)", 1.2, "5 4")
    s.rect(X(conv["x_start"]), Y(conv["y"] + conv["width"] / 2), (conv["x_end"] - conv["x_start"]) * sc,
           conv["width"] * sc, "var(--text-dim)", 0.10, width=1)
    s.line(X(conv["x_start"]) + 10, Y(conv["y"]) + conv["width"] * sc / 2 + 14,
           X(conv["x_start"]) + 90, Y(conv["y"]) + conv["width"] * sc / 2 + 14, arrow=True, width=1.4)
    s.text(X(conv["x_start"]) + 100, Y(conv["y"]) + conv["width"] * sc / 2 + 18,
           "컨베이어 %.0f mm/s" % (conv["speed"] * 1000), "tag", anchor="start")
    for b in robot.bins():
        col = "var(--danger)" if b["color"] == "red" else "var(--info)"
        s.rect(X(b["x"] - b["w"] / 2), Y(b["y"] + b["d"] / 2), b["w"] * sc, b["d"] * sc, col, 0.15, width=1.4)
        s.text(X(b["x"]), Y(b["y"]) + 4, "상자 %s (%s)" % (b["id"], b["color"]), "tag")
    for x, y, c in picks:
        s.dot(X(x), Y(y), 3.2, "var(--danger)" if c == "red" else "var(--info)")
    tx = 560
    s.text(tx, 60, "초록 실선: 부품 윗면 높이에서", "tag", anchor="start")
    s.text(tx, 76, "도달 가능한 경계", "tag", anchor="start")
    s.text(tx, 100, "회색 점선: 장면의", "tag", anchor="start")
    s.text(tx, 116, "작업 반지름 %.0f mm" % (rc * 1000), "tag", anchor="start")
    s.text(tx, 140, "점: 실제 집은 위치", "tag", anchor="start")
    s.text(tx, 156, "(%d개, 예측한 x)" % len(picks), "tag", anchor="start")
    s.text(tx, 190, "눈금 100 mm", "tag", anchor="start")
    s.line(tx, 200, tx + 0.1 * sc, 200, width=2)
    s.write()
    return len(picks)


# ====================================================================== d10
def d10_a5_fit():
    base = DeltaDesign.preset("printer_stepper").copy(tool="pen", payload=0.0)
    small = base.copy(base_radius=0.08, effector_radius=0.03, upper_arm=0.1, forearm=0.22)
    s = Svg("d10-a5-fit", 760, 440, "종이 높이에서 도달 가능한 영역과 A5 용지")
    sc = 0.6   # px per mm
    cx, cy = 235, 215
    X = lambda x: cx + x * 1000 * sc
    Y = lambda y: cy - y * 1000 * sc
    cases = ((small, "v0: R80 r30 L100 l220", None), (base, "v1: printer_stepper R110 r35 L140 l320", None),
             (base, "최종: v1 + 종이 면을 z = −320 mm 로", -0.320))
    for k, (d, label, zset) in enumerate(cases):
        r = DeltaRobot(d, scene="drawing")
        z = r.scene.surface_z if zset is None else zset
        pts = []
        for j in range(145):
            a = 2 * math.pi * j / 144
            lo, hi = 0.0, kin.workspace_bounds(d)[0]
            for _ in range(22):
                mid = (lo + hi) / 2
                if r.reachable(mid * math.cos(a), mid * math.sin(a), z):
                    lo = mid
                else:
                    hi = mid
            pts.append((lo * math.cos(a), lo * math.sin(a)))
        col = (COLORS[3], COLORS[1], COLORS[0])[k]
        s.cpath("M" + " L".join("%.1f %.1f" % (X(p[0]), Y(p[1])) for p in pts) + " Z", col, 2.0,
                fill=col, opacity=0.05, dash="" if k != 1 else "6 4")
        s.text(520, 50 + k * 54, label, "tag", anchor="start")
        s.ctext(520, 68 + k * 54, "종이 면 z = %.0f mm 에서의 경계" % (z * 1000), col, size=12)
    s.rect(X(-0.105), Y(0.074), 0.210 * 1000 * sc, 0.148 * 1000 * sc, "var(--text-dim)", 0.0, dash=True, width=1.8)
    s.text(X(0), Y(0.074) - 8, "A5 210 × 148 mm", "lbl")
    s.text(520, 230, "경계: 144 방향으로 이분 탐색한", "tag", anchor="start")
    s.text(520, 246, "reachable() 결과 (모터·볼조인트·팔꿈치", "tag", anchor="start")
    s.text(520, 262, "한계 포함)", "tag", anchor="start")
    s.write()


# ====================================================================== d11
def d11_flow():
    s = Svg("d11-design-loop", 760, 300, "설계 캡스톤 흐름: 요구사항에서 검증 보고서까지")
    steps = [("① 요구사항", "페이로드·D×H", "사이클·정밀도"), ("② 치수", "R r L l (D03)", "작업 실린더"),
             ("③ 구동", "모터·감속기", "D04 · 관성비"), ("④ 툴·궤적", "유지력 (D05)", "프로파일 (D06)"),
             ("⑤ 검증", "analyze()", "응용 프로그램"), ("⑥ 보고서", "표·그래프", "JSON·URDF")]
    w, gap = 116, 10
    x0 = (760 - (6 * w + 5 * gap)) / 2
    for i, (a, b, c) in enumerate(steps):
        x = x0 + i * (w + gap)
        s.box(x, 70, w, 96, hi=(i == 4))
        s.text(x + w / 2, 98, a)
        s.text(x + w / 2, 122, b, "sub")
        s.text(x + w / 2, 142, c, "sub")
        if i < 5:
            s.line(x + w, 118, x + w + gap - 1, 118, arrow=True)
    xa = x0 + 4 * (w + gap) + w / 2
    xb = x0 + 1 * (w + gap) + w / 2
    s.path("M%.1f 166 L%.1f 216 L%.1f 216 L%.1f 170" % (xa, xa, xb, xb), hi=True, arrow=True, width=1.8)
    s.text((xa + xb) / 2, 234, "✖ 항목이 있으면 원인 단계로 돌아가 반복 (반복 기록도 보고서에 남김)", "tag")
    s.text(380, 40, "모든 수치는 시뮬레이터·deltarobot 실행 결과로 뒷받침", "sub")
    s.text(380, 272, "제출물: 설계 JSON + 검증 코드 + 보고서(요구 대비 결과표) + 발표", "tag")
    s.write()


if __name__ == "__main__":
    d01_sim_tabs()
    d01_graph_reading()
    d02_adept_cycle()
    d03_ratio_effect()
    d04_torque_speed()
    rows = d04_ratio_sweep()
    for r in rows:
        print("  ratio %2d  peak %.2f rms %.2f speed %.2f inertia %.1f" % r)
    d05_holding()
    for p in d06_profiles():
        print("  %s T=%.3f" % (p.kind, p.T))
    d06_arch_paths()
    print("  urdf", d07_urdf_tree())
    d08_workspace_fit()
    print("  d09 picks", d09_cell())
    d10_a5_fit()
    d11_flow()
