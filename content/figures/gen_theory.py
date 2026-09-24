"""Generate the SVG figures used by the theory-track chapters (t01-t08).

Run from the repo root:  python content/figures/gen_theory.py
Geometry is computed with the real library (python/deltarobot) so the drawings match the
formulas in the lessons.  Only the CSS classes defined in assets/css/main.css (.lbl .sub .tag
.hi .wr .dg .ln .ln-hi .box .box-hi .mono) and CSS variables are used, so every figure
follows the light/dark theme.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(OUT.parent.parent / "python"))

from deltarobot import DeltaDesign  # noqa: E402
from deltarobot import dynamics as dyn  # noqa: E402
from deltarobot import kinematics as kin  # noqa: E402


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def f(v: float) -> str:
    return ("%.1f" % v).rstrip("0").rstrip(".")


class Svg:
    def __init__(self, name: str, w: int, h: int, label: str):
        self.name, self.w, self.h, self.label = name, w, h, label
        self.parts: list = []
        self.mid = "arr-" + name  # marker id unique per figure (figures are inlined)

    def add(self, s: str) -> None:
        self.parts.append("  " + s)

    def box(self, x, y, w, h, hi=False, rx=8):
        self.add('<rect class="%s" x="%s" y="%s" width="%s" height="%s" rx="%s"/>'
                 % ("box-hi" if hi else "box", f(x), f(y), f(w), f(h), f(rx)))

    def rect(self, x, y, w, h, style):
        self.add('<rect x="%s" y="%s" width="%s" height="%s" style="%s"/>' % (f(x), f(y), f(w), f(h), style))

    def text(self, x, y, s, cls="lbl", anchor="middle", extra=""):
        self.add('<text class="%s" x="%s" y="%s" text-anchor="%s"%s>%s</text>'
                 % (cls, f(x), f(y), anchor, extra, esc(s)))

    def _stroke(self, hi, style):
        if style:
            return 'style="fill:none;%s"' % style
        return 'class="%s"' % ("ln-hi" if hi else "ln")

    def line(self, x1, y1, x2, y2, hi=False, arrow=False, width=1.6, dash=False, style=None, cap=False):
        self.path("M%s %s L%s %s" % (f(x1), f(y1), f(x2), f(y2)), hi, width, arrow, dash, style, cap)

    def path(self, d, hi=False, width=1.6, arrow=False, dash=False, style=None, cap=False):
        mk = ""
        if arrow:
            mk = ' marker-end="url(#%s%s)"' % (self.mid, "-hi" if hi else "")
        self.add('<path %s d="%s" stroke-width="%s"%s%s%s/>'
                 % (self._stroke(hi, style), d, f(width),
                    ' stroke-dasharray="5 4"' if dash else "",
                    ' stroke-linecap="round"' if cap else "", mk))

    def poly(self, pts, cls="box", closed=True, style=None, width=1.4):
        d = "M" + " L".join("%s %s" % (f(x), f(y)) for x, y in pts) + (" Z" if closed else "")
        attr = 'style="%s"' % style if style else 'class="%s"' % cls
        self.add('<path %s d="%s" stroke-width="%s"/>' % (attr, d, f(width)))

    def circle(self, cx, cy, r, cls="box", style=None):
        attr = 'style="%s"' % style if style else 'class="%s"' % cls
        self.add('<circle %s cx="%s" cy="%s" r="%s"/>' % (attr, f(cx), f(cy), f(r)))

    def arc(self, cx, cy, r, a0, a1, hi=False, arrow=False, width=1.4, style=None):
        """Arc in screen angles (degrees, 0 = +x screen, positive = clockwise on screen)."""
        x0, y0 = cx + r * math.cos(math.radians(a0)), cy + r * math.sin(math.radians(a0))
        x1, y1 = cx + r * math.cos(math.radians(a1)), cy + r * math.sin(math.radians(a1))
        large = 1 if abs(a1 - a0) > 180 else 0
        sweep = 1 if a1 > a0 else 0
        self.path("M%s %s A%s %s 0 %d %d %s %s" % (f(x0), f(y0), f(r), f(r), large, sweep, f(x1), f(y1)),
                  hi, width, arrow, style=style)

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
        print("wrote %s.svg (%d bytes)" % (self.name, len(svg.encode("utf-8"))))


INK = "stroke:var(--text-dim)"
WARN = "stroke:var(--warn)"
DANGER = "stroke:var(--danger)"


# --------------------------------------------------------------------- 3D projection
class View:
    """Oblique orthographic view: camera azimuth az, elevation el (deg), scale S px/m."""

    def __init__(self, cx, cy, S, az=-75.0, el=18.0):
        self.cx, self.cy, self.S = cx, cy, S
        a, e = math.radians(az), math.radians(el)
        self.r = (-math.sin(a), math.cos(a), 0.0)
        self.u = (-math.sin(e) * math.cos(a), -math.sin(e) * math.sin(a), math.cos(e))
        self.v = (math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e))  # towards camera

    def __call__(self, p):
        return (self.cx + self.S * sum(p[k] * self.r[k] for k in range(3)),
                self.cy - self.S * sum(p[k] * self.u[k] for k in range(3)))

    def depth(self, p):
        return sum(p[k] * self.v[k] for k in range(3))


def draw_delta(s: Svg, d: DeltaDesign, view: View, p, labels: bool = False, tool: bool = True):
    """Draw the whole robot for effector centre p. Returns dict of screen points for labels."""
    th = kin.ik(d, p)
    pts = {}
    # base plate
    ring = [view((1.25 * d.R * math.cos(a), 1.25 * d.R * math.sin(a), 0.0))
            for a in [2 * math.pi * k / 48 for k in range(48)]]
    s.poly(ring, "box")
    pts["base"] = view((0, 0, 0))
    # arms, back to front
    order = sorted(range(3), key=lambda i: view.depth(kin.elbow(d, i, th[i])))
    for i in order:
        c, sn = math.cos(kin.PHI[i]), math.sin(kin.PHI[i])
        t = (-sn, c, 0.0)
        m = kin.motor_point(d, i)
        e = kin.elbow(d, i, th[i])
        b = kin.ball_joint(d, i, p)
        hw = d.forearm_spacing / 2
        e1 = tuple(e[k] + hw * t[k] for k in range(3))
        e2 = tuple(e[k] - hw * t[k] for k in range(3))
        b1 = tuple(b[k] + hw * t[k] for k in range(3))
        b2 = tuple(b[k] - hw * t[k] for k in range(3))
        # motor body along the tangential axis
        m1 = tuple(m[k] + 0.03 * t[k] for k in range(3))
        m2 = tuple(m[k] - 0.03 * t[k] for k in range(3))
        s.line(*view(m1), *view(m2), width=9, style=INK, cap=True)
        s.line(*view(m), *view(e), width=5, hi=True, cap=True)
        s.line(*view(e1), *view(e2), width=2.5, style=INK, cap=True)
        s.line(*view(e1), *view(b1), width=1.8, style=INK)
        s.line(*view(e2), *view(b2), width=1.8, style=INK)
        for q in (e1, e2, b1, b2):
            s.circle(*view(q), 2.6, "hi")
        pts["motor%d" % i] = view(m)
        pts["elbow%d" % i] = view(e)
        pts["rod%d" % i] = view(tuple((e1[k] + b1[k]) / 2 for k in range(3)))
        pts["ball%d" % i] = view(b1)
        pts["arm%d" % i] = view(tuple((m[k] + e[k]) / 2 for k in range(3)))
    # effector plate
    plate = [view((p[0] + 1.35 * d.r * math.cos(a), p[1] + 1.35 * d.r * math.sin(a), p[2]))
             for a in [2 * math.pi * k / 36 for k in range(36)]]
    s.poly(plate, "box-hi")
    pts["eff"] = view(p)
    if tool:
        tip = (p[0], p[1], p[2] - d.tool_length)
        s.line(*view(p), *view(tip), width=3, style=INK)
        s.circle(*view(tip), 3.5, "hi")
        pts["tcp"] = view(tip)
    return pts


# --------------------------------------------------------------------- t01
def t01_serial_vs_parallel():
    s = Svg("t01-serial-vs-parallel", 760, 360, "직렬로봇과 델타(병렬)로봇의 구조 비교")
    # --- serial arm (left)
    s.text(180, 28, "직렬로봇 (열린 사슬)")
    s.line(40, 300, 320, 300, width=1.4)
    s.box(80, 270, 60, 30)
    joints = [(110, 270), (140, 170), (250, 120), (300, 180)]
    for (x1, y1), (x2, y2) in zip(joints, joints[1:]):
        s.line(x1, y1, x2, y2, hi=True, width=7, cap=True)
    for k, (x, y) in enumerate(joints[:-1]):
        s.box(x - 13, y - 13, 26, 26, rx=4)
        s.text(x, y + 4, "M%d" % (k + 1), "tag")
    s.circle(300, 180, 5, "hi")
    s.text(40, 330, "모터 M2·M3가 앞 링크에 실려 함께 움직임", "sub", "start")
    s.text(40, 348, "→ 움직이는 질량이 크고, 오차가 링크마다 누적", "sub", "start")
    # --- delta (right) with real geometry
    d = DeltaDesign.preset("edu_dynamixel")
    view = View(560, 70, 780, az=-80, el=16)
    s.text(560, 28, "델타로봇 (닫힌 사슬 3개)")
    pts = draw_delta(s, d, view, (0.0, 0.0, -0.28))
    s.text(700, 64, "모터 3개는 베이스에 고정", "sub", "start")
    s.line(698, 60, pts["motor0"][0] + 10, pts["motor0"][1], arrow=True, width=1.1)
    s.text(420, 330, "움직이는 것은 가벼운 위팔·로드·이펙터뿐", "sub", "start")
    s.text(420, 348, "→ 큰 가속도, 세 팔이 하중을 나눠 받음", "sub", "start")
    s.write()


# --------------------------------------------------------------------- t02
def t02_structure():
    s = Svg("t02-structure", 760, 460, "델타로봇의 구성 요소")
    d = DeltaDesign.preset("edu_dynamixel")
    view = View(330, 90, 900, az=-80, el=18)
    pts = draw_delta(s, d, view, (0.03, 0.0, -0.27))

    def callout(key_pt, tx, ty, label, sub=None, anchor="start"):
        s.line(tx + (-6 if anchor == "start" else 6), ty - 4, key_pt[0], key_pt[1], width=1.0, arrow=True)
        s.text(tx, ty, label, "lbl", anchor)
        if sub:
            s.text(tx, ty + 17, sub, "sub", anchor)

    callout(pts["base"], 560, 40, "베이스 (고정)", "모터 3개가 120° 간격")
    callout(pts["motor0"], 600, 100, "모터 = 능동 회전관절", "축은 베이스 원의 접선 방향")
    callout(pts["arm0"], 620, 170, "위팔 L", "모터가 직접 돌리는 링크")
    callout(pts["elbow0"], 620, 235, "팔꿈치 (볼조인트 2개)", "간격 w")
    callout(pts["rod0"], 620, 300, "아래팔 = 평행사변형", "길이 l 로드 2개")
    callout(pts["eff"], 610, 380, "이펙터 (움직이는 판)", "볼조인트 6개, 병진만")
    callout(pts["tcp"], 150, 430, "TCP (툴 끝)", "이펙터 중심 − 툴 길이", anchor="end")
    s.write()


def t02_parallelogram():
    s = Svg("t02-parallelogram", 760, 300, "평행사변형 아래팔이 이펙터의 방향을 유지하는 원리")
    w, l = 90, 190

    def para(x0, y0, ang, label):
        # top bar at (x0, y0), rods of length l at angle ang (deg from vertical)
        dx, dy = l * math.sin(math.radians(ang)), l * math.cos(math.radians(ang))
        a, b = (x0, y0), (x0 + w, y0)
        c, e = (x0 + w + dx, y0 + dy), (x0 + dx, y0 + dy)
        s.line(*a, *b, width=6, style=INK, cap=True)
        s.line(*a, *e, width=2.2, style=INK)
        s.line(*b, *c, width=2.2, style=INK)
        s.line(*e, *c, width=6, hi=True, cap=True)
        for q in (a, b, c, e):
            s.circle(*q, 5, "hi")
        s.text(x0 + w / 2 + dx / 2, y0 + dy + 34, label, "sub")
        return a, b, c, e

    s.text(40, 30, "위: 팔꿈치 막대(위팔 끝)   아래: 이펙터 쪽 막대", "sub", "start")
    para(70, 60, 0, "θ만 바뀐 자세")
    para(300, 60, 25, "로드가 옆으로 기운 자세")
    para(560, 60, -30, "반대로 기운 자세")
    s.text(380, 290, "마주 보는 두 변은 항상 평행 → 이펙터 쪽 막대는 팔꿈치 막대와 같은 방향을 유지", "lbl")
    s.write()


# --------------------------------------------------------------------- t03
def t03_top_view():
    d = DeltaDesign.preset("edu_dynamixel")
    s = Svg("t03-top-view", 760, 440, "위에서 본 좌표계와 설계 파라미터 R, r, φ")
    S, cx, cy = 800.0, 300.0, 225.0

    def P(x, y):
        return cx + S * x, cy - S * y

    th = d.home_theta
    # axes
    s.line(*P(-0.2, 0), *P(0.27, 0), arrow=True, width=1.1)
    s.line(*P(0, -0.24), *P(0, 0.26), arrow=True, width=1.1)
    s.text(*P(0.272, -0.018), "x", "lbl", "start")
    s.text(*P(0.01, 0.255), "y", "lbl", "start")
    # base circle R and effector triangle
    s.circle(*P(0, 0), S * d.R, style="fill:none;stroke:var(--border);stroke-dasharray:5 4")
    tri = [P(d.r * math.cos(a) * 2, d.r * math.sin(a) * 2) for a in (math.pi / 3, math.pi, 5 * math.pi / 3)]
    s.poly(tri, "box-hi")
    for i in range(3):
        c, sn = math.cos(kin.PHI[i]), math.sin(kin.PHI[i])
        m = (d.R * c, d.R * sn)
        e = kin.elbow(d, i, th)
        b = (d.r * c, d.r * sn)
        # motor axis (tangential)
        s.line(*P(m[0] + 0.035 * sn, m[1] - 0.035 * c), *P(m[0] - 0.035 * sn, m[1] + 0.035 * c),
               width=10, style=INK, cap=True)
        s.line(*P(*m), *P(e[0], e[1]), hi=True, width=5, cap=True)
        s.line(*P(e[0], e[1]), *P(*b), width=1.6, style=INK, dash=True)
        s.circle(*P(*b), 4.5, "hi")
        s.circle(*P(e[0], e[1]), 4.5, "hi")
        lx, ly = P((d.R + d.L + 0.03) * c, (d.R + d.L + 0.03) * sn)
        s.text(lx, ly + 4, "팔 %d" % (i + 1), "lbl")
    # R and r dimension
    s.line(*P(0, 0), *P(d.R * math.cos(0.5), d.R * math.sin(-0.5)), arrow=True, width=1.1)
    s.text(*P(0.052, -0.058), "R", "lbl")
    s.line(*P(0, 0), *P(d.r, 0), hi=True, arrow=True, width=1.4)
    s.text(*P(0.017, 0.012), "r", "lbl")
    # phi2
    s.arc(cx, cy, 55, 0, -120, arrow=True, width=1.1)
    s.text(*P(0.05, 0.08), "φ₂ = 120°", "tag", "start")
    # legend
    s.box(540, 30, 205, 180)
    s.text(556, 56, "R  베이스 중심 → 모터축", "sub", "start")
    s.text(556, 78, "r  이펙터 중심 → 볼조인트 쌍", "sub", "start")
    s.text(556, 100, "φᵢ = 0°, 120°, 240°", "sub", "start")
    s.text(556, 122, "모터축은 반지름 방향에 수직", "sub", "start")
    s.text(556, 144, "굵은 선: 위팔 (θ = 0.35 rad)", "sub", "start")
    s.text(556, 166, "점선: 아래팔의 위쪽 투영", "sub", "start")
    s.text(556, 188, "edu_dynamixel 치수, 축척 일정", "sub", "start")
    s.text(560, 420, "z축은 화면 밖(위)을 향함 · 작업은 z < 0", "tag")
    s.write()


def t03_arm_plane():
    d = DeltaDesign.preset("edu_dynamixel")
    p = (0.0, 0.0, -0.28)
    th = kin.ik(d, p)[0]
    s = Svg("t03-arm-plane", 760, 460, "팔 평면에서 본 모터각 θ와 TCP")
    S, cx, cy = 1000.0, 200.0, 70.0

    def P(x, z):
        return cx + S * x, cy - S * z

    s.line(*P(-0.1, 0), *P(0.3, 0), width=1.0, dash=True)
    s.text(*P(0.3, 0.008), "z = 0 (모터축 평면)", "tag", "end")
    s.line(*P(0, 0.03), *P(0, -0.37), width=1.0, dash=True)
    s.text(*P(0.004, 0.035), "베이스 중심축", "tag", "start")
    m = (d.R, 0.0)
    e = (d.R + d.L * math.cos(th), -d.L * math.sin(th))
    b = (p[0] + d.r, p[2])
    tip = (p[0], p[2] - d.tool_length)
    s.line(*P(*m), *P(e[0] + 0.06, m[1]), width=1.0, dash=True)
    s.line(*P(*m), *P(*e), hi=True, width=6, cap=True)
    s.line(*P(*e), *P(*b), width=2.2, style=INK)
    s.line(*P(p[0] - 0.05, p[2]), *P(b[0] + 0.012, p[2]), width=6, style="stroke:var(--accent-dim)", cap=True)
    s.line(*P(p[0], p[2]), *P(*tip), width=3, style=INK)
    s.circle(*P(*m), 8, "box")
    s.circle(*P(*e), 5, "hi")
    s.circle(*P(*b), 5, "hi")
    s.circle(*P(*tip), 5, "hi")
    s.circle(*P(p[0], p[2]), 4, "box")
    # theta arc (screen angle positive = clockwise = downward)
    s.arc(*P(*m), 55, 0, math.degrees(th), arrow=True, width=1.3)
    s.text(P(*m)[0] + 64, P(*m)[1] + 26, "θ (아래로 +)", "lbl", "start")
    s.text(*P(d.R, 0.02), "모터", "tag")
    s.text(P(*e)[0] + 10, P(*e)[1] - 6, "E 팔꿈치", "tag", "start")
    s.text((P(*m)[0] + P(*e)[0]) / 2 + 8, (P(*m)[1] + P(*e)[1]) / 2 - 8, "L", "lbl", "start")
    s.text((P(*e)[0] + P(*b)[0]) / 2 + 10, (P(*e)[1] + P(*b)[1]) / 2, "l (투영)", "lbl", "start")
    s.text(P(*b)[0] + 10, P(*b)[1] + 4, "B 볼조인트 쌍", "tag", "start")
    s.text(P(p[0], p[2])[0] - 10, P(p[0], p[2])[1] - 10, "이펙터 중심 P", "tag", "end")
    s.text(P(*tip)[0] + 10, P(*tip)[1] + 4, "TCP = P − (0, 0, 툴 길이)", "tag", "start")
    # R and r dims
    y0 = P(0, 0)[1] - 22
    s.line(P(0, 0)[0], y0, P(d.R, 0)[0], y0, width=1.0, arrow=True)
    s.text((P(0, 0)[0] + P(d.R, 0)[0]) / 2, y0 - 6, "R", "lbl")
    yb = P(0, p[2])[1] + 26
    s.line(P(0, 0)[0], yb, P(d.r, 0)[0], yb, width=1.0, arrow=True)
    s.text(P(d.r, 0)[0] + 8, yb + 4, "r", "lbl", "start")
    s.box(470, 120, 275, 170)
    s.text(486, 146, "팔 좌표계 (팔 1 기준)", "lbl", "start")
    s.text(486, 170, "모터 M = (R, 0, 0)", "sub", "start")
    s.text(486, 192, "팔꿈치 E = (R + L cos θ, 0, −L sin θ)", "sub", "start")
    s.text(486, 214, "볼조인트 B = P + r·(1, 0, 0)", "sub", "start")
    s.text(486, 236, "제약: |B − E| = l", "sub", "start")
    s.text(486, 258, "θ = %.3f rad (P = (0, 0, −0.28))" % th, "sub", "start")
    s.text(486, 278, "edu_dynamixel, 축척 일정", "sub", "start")
    s.write()


# --------------------------------------------------------------------- t04
def t04_ik_arm_plane():
    d = DeltaDesign.preset("edu_dynamixel")
    p = (0.05, -0.04, -0.26)
    i = 1
    c, sn = math.cos(kin.PHI[i]), math.sin(kin.PHI[i])
    xa, ya = p[0] * c + p[1] * sn, -p[0] * sn + p[1] * c
    a = xa + d.r - d.R
    K = (d.l ** 2 - d.L ** 2 - a * a - ya * ya - p[2] ** 2) / (2 * d.L)
    rho = math.hypot(a, p[2])
    alpha = math.atan2(a, p[2])
    t1 = alpha + math.asin(K / rho)
    t2 = alpha + math.pi - math.asin(K / rho)
    tsel = kin.ik_arm(d, i, p)
    other = t2 if abs(math.remainder(t1 - tsel, 2 * math.pi)) < 1e-9 else t1
    s = Svg("t04-ik-arm-plane", 760, 440, "팔 평면에서 두 원의 교점으로 푸는 역기구학")
    S, cx, cy = 900.0, 250.0, 90.0

    def P(x, z):
        return cx + S * x, cy - S * z

    s.line(*P(-0.2, 0), *P(0.3, 0), width=1.0, dash=True)
    s.line(*P(0, 0.06), *P(0, -0.33), width=1.0, dash=True)
    s.text(*P(-0.195, 0.008), "z = 0", "tag", "start")
    s.text(*P(0.004, 0.055), "x′=0", "tag", "start")
    m = (d.R, 0.0)
    b = (xa + d.r, p[2])
    rp = math.sqrt(d.l ** 2 - ya * ya)
    # circle 1: elbow circle around motor
    s.circle(*P(*m), S * d.L, style="fill:none;stroke:var(--accent);stroke-dasharray:6 4;stroke-width:1.3")
    # circle 2 (only the upper arc) around B'
    s.arc(*P(*b), S * rp, -115, -35, style="stroke:var(--warn);stroke-dasharray:6 4", width=1.3)
    eo = (d.R + d.L * math.cos(tsel), -d.L * math.sin(tsel))
    ei = (d.R + d.L * math.cos(other), -d.L * math.sin(other))
    s.line(*P(*m), *P(*ei), width=3, style=INK, dash=True)
    s.line(*P(*ei), *P(*b), width=1.4, style=INK, dash=True)
    s.line(*P(*m), *P(*eo), hi=True, width=6, cap=True)
    s.line(*P(*eo), *P(*b), width=2.2, style=INK)
    s.circle(*P(*m), 8, "box")
    s.circle(*P(*eo), 5.5, "hi")
    s.circle(*P(*ei), 5, "box")
    s.circle(*P(*b), 5.5, "hi")
    s.text(P(*m)[0], P(*m)[1] - 14, "모터 (R, 0)", "tag")
    s.text(P(*eo)[0] + 12, P(*eo)[1] - 4, "해 A: 팔꿈치 바깥 (선택)", "tag", "start")
    s.text(P(*eo)[0] + 12, P(*eo)[1] + 12, "θ = %.3f rad" % tsel, "tag", "start")
    s.text(P(*ei)[0] - 12, P(*ei)[1] + 4, "해 B: 팔꿈치 안쪽", "tag", "end")
    s.text(P(*ei)[0] - 12, P(*ei)[1] + 20, "θ = %.3f rad" % other, "tag", "end")
    s.text(P(*b)[0] + 12, P(*b)[1] + 4, "B′ = (x′ + r, z)", "tag", "start")
    s.text(*P(d.R + d.L + 0.005, -0.05), "반지름 L", "sub", "start")
    s.text(*P(-0.17, -0.02), "반지름 √(l² − y′²)", "sub", "start")
    s.box(540, 250, 210, 175)
    s.text(552, 274, "팔 2 (φ = 120°) 의 팔 평면", "lbl", "start")
    s.text(552, 296, "P = (0.05, −0.04, −0.26)", "sub", "start")
    s.text(552, 316, "x′ = %.4f, y′ = %.4f" % (xa, ya), "sub", "start")
    s.text(552, 336, "a = x′ + r − R = %.4f" % a, "sub", "start")
    s.text(552, 356, "K = %.4f, ρ = %.4f" % (K, rho), "sub", "start")
    s.text(552, 376, "α = atan2(a, z) = %.4f" % alpha, "sub", "start")
    s.text(552, 396, "두 해 중 cos θ 가 큰 쪽", "sub", "start")
    s.text(552, 414, "edu_dynamixel, 축척 일정", "sub", "start")
    s.write()


# --------------------------------------------------------------------- t05
def t05_trilateration():
    d = DeltaDesign.preset("edu_dynamixel")
    th = d.home_theta
    s = Svg("t05-trilateration", 800, 440, "세 구의 교점으로 푸는 순기구학")
    # left: top view, E_i -> C_i
    S, cx, cy = 650.0, 190.0, 230.0

    def T(x, y):
        return cx + S * x, cy - S * y

    s.text(190, 30, "위에서 본 구 중심 이동: Cᵢ = Eᵢ − r·uᵢ")
    s.circle(*T(0, 0), 4, "box")
    for i in range(3):
        e = kin.elbow(d, i, th)
        u = kin.arm_axis(i)
        ci = (e[0] - d.r * u[0], e[1] - d.r * u[1])
        s.line(*T(0, 0), *T(e[0], e[1]), width=1.0, dash=True)
        s.line(*T(e[0], e[1]), *T(*ci), hi=True, width=1.8, arrow=True)
        s.circle(*T(e[0], e[1]), 5.5, "box")
        s.circle(*T(*ci), 5, "hi")
        lx, ly = T(e[0] * 1.2, e[1] * 1.2)
        s.text(lx, ly + 4, "E%d" % (i + 1), "lbl")
        mx, my = T(ci[0] * 0.72, ci[1] * 0.72)
        s.text(mx, my + 4, "C%d" % (i + 1), "tag")
    s.text(190, 400, "각 팔의 제약 |P + r·uᵢ − Eᵢ| = l", "sub")
    s.text(190, 418, "⇔ |P − Cᵢ| = l : 반지름 l 인 구 3개", "sub")
    # right: side view (x-z), real geometry at home
    S2, cx2, cy2 = 520.0, 575.0, 205.0

    def Q(x, z):
        return cx2 + S2 * x, cy2 - S2 * z

    s.text(575, 30, "옆에서 본 두 교점 (θ₁ = θ₂ = θ₃ = %.2f rad)" % th)
    c = [(kin.elbow(d, i, th)[0] - d.r * kin.arm_axis(i)[0], kin.elbow(d, i, th)[2]) for i in range(3)]
    for (x, z), sty in ((c[0], "stroke:var(--accent)"), (c[1], "stroke:var(--warn)")):
        s.circle(*Q(x, z), S2 * d.l, style="fill:none;%s;stroke-dasharray:6 4;stroke-width:1.2" % sty)
        s.circle(*Q(x, z), 5, "hi")
    s.text(Q(*c[0])[0] + 8, Q(*c[0])[1] - 8, "C1", "tag", "start")
    s.text(Q(*c[1])[0] - 8, Q(*c[1])[1] - 8, "C2, C3 (겹쳐 보임)", "tag", "end")
    s.line(*Q(-0.2, c[0][1]), *Q(0.26, c[0][1]), width=1.2, style=INK)
    s.text(*Q(0.26, c[0][1] + 0.01), "세 중심의 평면", "tag", "end")
    lo = kin.fk(d, (th, th, th))
    hi_z = 2 * c[0][1] - lo[2]
    s.line(*Q(0, lo[2]), *Q(0, hi_z), width=1.2, dash=True)
    s.circle(*Q(0, lo[2]), 6, "hi")
    s.circle(*Q(0, hi_z), 6, "box")
    s.text(Q(0, lo[2])[0] + 10, Q(0, lo[2])[1] + 18, "아래 해 z = %.4f (선택)" % lo[2], "lbl", "start")
    s.text(Q(0, hi_z)[0] + 10, Q(0, hi_z)[1] - 6, "위 해 z = %.4f" % hi_z, "sub", "start")
    s.text(Q(0, c[0][1])[0] - 8, (Q(0, c[0][1])[1] + Q(0, lo[2])[1]) / 2, "h", "lbl", "end")
    s.text(Q(0, c[0][1])[0] - 8, (Q(0, c[0][1])[1] + Q(0, hi_z)[1]) / 2, "h", "lbl", "end")
    s.write()


# --------------------------------------------------------------------- t06
def t06_singularity():
    s = Svg("t06-singularity", 760, 360, "델타로봇의 두 종류 특이점")
    # panel 1: type 1, arm and rod aligned
    s.text(190, 30, "1형 (역기구학·경계) 특이점")
    m = (60, 120)
    ang = math.radians(28)
    L, l = 110, 230
    e = (m[0] + L * math.cos(ang), m[1] + L * math.sin(ang))
    b = (e[0] + l * math.cos(ang), e[1] + l * math.sin(ang))
    s.circle(*m, 8, "box")
    s.line(*m, *e, hi=True, width=6, cap=True)
    s.line(*e, *b, width=2.2, style=INK)
    s.circle(*e, 5, "hi")
    s.circle(*b, 5, "hi")
    # dE/dtheta perpendicular to the arm
    pe = (e[0] - 45 * math.sin(ang) * -1, e[1] + 45 * math.cos(ang))
    s.line(*e, pe[0], pe[1], arrow=True, width=1.6, style=WARN)
    s.text(pe[0] + 6, pe[1] + 12, "∂E/∂θ", "tag", "start")
    s.line(e[0] + 10 * math.cos(ang), e[1] + 10 * math.sin(ang), e[0] + 80 * math.cos(ang),
           e[1] + 80 * math.sin(ang), hi=True, arrow=True, width=1.6)
    s.text(e[0] + 80 * math.cos(ang) + 4, e[1] + 80 * math.sin(ang) - 12, "sᵢ", "lbl", "start")
    s.text(40, 300, "위팔과 아래팔이 일직선 (완전히 펴짐/접힘)", "sub", "start")
    s.text(40, 318, "sᵢ · ∂Eᵢ/∂θᵢ = 0 → Jθ의 i번째 값이 0", "sub", "start")
    s.text(40, 336, "그 방향으로는 움직일 수 없음 (작업영역 경계)", "sub", "start")
    # panel 2: type 2, rods coplanar (here: all horizontal)
    s.text(570, 30, "2형 (병렬·순기구학) 특이점")
    zc = 160
    s.line(440, zc, 700, zc, width=1.0, dash=True)
    s.text(705, zc + 4, "세 로드가 한 평면", "tag", "start")
    for (mx, my, ex, ey, bx) in ((420, 70, 450, zc, 540), (720, 70, 690, zc, 600)):
        s.circle(mx, my, 8, "box")
        s.line(mx, my, ex, ey, hi=True, width=6, cap=True)
        s.line(ex, ey, bx, ey, width=2.2, style=INK)
        s.circle(ex, ey, 5, "hi")
    s.line(535, zc, 605, zc, width=7, style="stroke:var(--accent-dim)", cap=True)
    s.line(570, zc - 8, 570, zc - 60, arrow=True, width=1.6, style=DANGER)
    s.line(570, zc + 8, 570, zc + 60, arrow=True, width=1.6, style=DANGER)
    s.text(582, zc + 64, "모터를 고정해도 위아래로 움직임", "tag", "start")
    s.text(440, 300, "s₁, s₂, s₃ 가 한 평면에 놓임 → det Jx = 0", "sub", "start")
    s.text(440, 318, "순기구학의 두 해(위·아래)가 합쳐지는 곳 (h = 0)", "sub", "start")
    s.text(440, 336, "강성 상실, 작은 힘에도 큰 모터 토크 필요", "sub", "start")
    s.write()


def _runs(cells):
    """Merge horizontal runs of equal category: cells = list of rows [(cat) ...]."""
    out = []
    for r, row in enumerate(cells):
        c0, start = row[0], 0
        for k in range(1, len(row) + 1):
            if k == len(row) or row[k] != c0:
                if c0 is not None:
                    out.append((r, start, k - start, c0))
                if k < len(row):
                    c0, start = row[k], k
    return out


def t06_cond_map():
    d = DeltaDesign.preset("edu_dynamixel")
    z = -0.28
    h = 0.005
    n = 44
    bins = [2.0, 2.5, 3.0, 4.0]
    op = [0.15, 0.35, 0.55, 0.75, 0.95]
    rows = []
    for iy in range(n, -n - 1, -1):
        row = []
        for ix in range(-n, n + 1):
            p = (ix * h, iy * h, z)
            rep = kin.limit_report(d, p)
            if not rep["ok"]:
                row.append(None)
                continue
            cn = kin.condition_number(d, rep["theta"], p)
            row.append(sum(1 for b in bins if cn >= b))
        rows.append(row)
    s = Svg("t06-cond-map", 760, 470, "작업 평면의 조건수 지도")
    S, x0, y0 = 1000.0, 40.0, 25.0
    cell = S * h
    for r, c, w, cat in _runs(rows):
        s.rect(x0 + c * cell, y0 + r * cell, w * cell + 0.3, cell + 0.3,
               "fill:var(--accent);fill-opacity:%.2f" % op[cat])
    cxs, cys = x0 + n * cell, y0 + n * cell
    s.line(cxs - 40, cys, cxs + 40, cys, width=1.0)
    s.line(cxs, cys - 40, cxs, cys + 40, width=1.0)
    s.text(cxs + 44, cys + 4, "x", "tag", "start")
    s.text(cxs, cys - 46, "y", "tag")
    for i in range(3):
        u = kin.arm_axis(i)
        s.text(cxs + 205 * u[0], cys - 205 * u[1] + 4, "팔 %d 방향" % (i + 1), "tag")
    lx = 520
    s.text(lx, 60, "z = −0.28 m 평면 (이펙터 중심)", "lbl", "start")
    s.text(lx, 80, "edu_dynamixel, 한계 안쪽만 칠함", "sub", "start")
    labels = ["κ < 2.0", "2.0 ≤ κ < 2.5", "2.5 ≤ κ < 3.0", "3.0 ≤ κ < 4.0", "κ ≥ 4.0"]
    for k, lab in enumerate(labels):
        s.rect(lx, 110 + 28 * k, 22, 18, "fill:var(--accent);fill-opacity:%.2f" % op[k])
        s.text(lx + 32, 124 + 28 * k, lab, "sub", "start")
    s.text(lx, 270, "κ = σmax / σmin (1 = 등방)", "sub", "start")
    s.text(lx, 290, "중심이 가장 고르고, 가장자리로", "sub", "start")
    s.text(lx, 308, "갈수록 한 방향이 둔해짐", "sub", "start")
    s.text(lx, 336, "눈금: 한 칸 5 mm, 전체 ±0.22 m", "sub", "start")
    s.write()


# --------------------------------------------------------------------- t07
def t07_workspace_section():
    d = DeltaDesign.preset("edu_dynamixel")
    h = 0.005
    xs = [k * h for k in range(-64, 65)]
    zs = [-k * h for k in range(0, 91)]
    kinds = {"ok": "fill:var(--accent);fill-opacity:0.55", "theta": "fill:var(--warn);fill-opacity:0.55",
             "ball": "fill:var(--danger);fill-opacity:0.45", "elbow": "fill:var(--task);fill-opacity:0.55"}
    rows = []
    for z in zs:
        row = []
        for x in xs:
            rep = kin.limit_report(d, (x, 0.0, z))
            pr = rep["problems"]
            if "unreachable" in pr:
                row.append(None)
            elif not pr:
                row.append("ok")
            else:
                kset = {q.rstrip("123") for q in pr}
                row.append("elbow" if "elbow" in kset else "ball" if "ball" in kset else "theta")
        rows.append(row)
    s = Svg("t07-workspace-section", 760, 500, "y = 0 단면의 작업영역과 제한 요인")
    S, x0, y0 = 700.0, 20.0, 40.0
    cell = S * h
    for r, c, w, cat in _runs(rows):
        s.rect(x0 + c * cell - cell / 2, y0 + r * cell - cell / 2, w * cell + 0.3, cell + 0.3, kinds[cat])
    X = lambda x: x0 + (x - xs[0]) * S  # noqa: E731
    Z = lambda z: y0 + (-z) * S  # noqa: E731
    s.line(X(-0.32), Z(0), X(0.32), Z(0), width=1.0)
    s.text(X(-0.32), Z(0) - 8, "z = 0 (모터축 평면)", "tag", "start")
    s.line(X(0), Z(0.0), X(0), Z(-0.45), width=1.0, dash=True)
    wc = kin.work_cylinder(d, 0.1)
    rr = wc["diameter"] / 2
    s.path("M%s %s L%s %s L%s %s L%s %s Z" % (f(X(-rr)), f(Z(wc["z_top"])), f(X(rr)), f(Z(wc["z_top"])),
                                              f(X(rr)), f(Z(wc["z_bottom"])), f(X(-rr)), f(Z(wc["z_bottom"]))),
           width=2.2, style="stroke:var(--text)")
    s.text(X(rr) + 6, Z(wc["z_top"]) - 6, "작업 실린더 D %.0f × H %.0f mm" % (wc["diameter"] * 1000, wc["height"] * 1000),
           "lbl", "start")
    for i, (k, lab) in enumerate((("ok", "사용 가능"), ("theta", "모터각 한계 초과"), ("ball", "볼조인트 각 초과"),
                                  ("elbow", "팔꿈치 각 초과"))):
        s.rect(40, 395 + 22 * i, 18, 14, kinds[k])
        s.text(66, 407 + 22 * i, lab, "sub", "start")
    s.text(260, 407, "빈칸: 팔 길이로 닿지 않음 (Unreachable)", "sub", "start")
    s.text(260, 429, "edu_dynamixel, 이펙터 중심 좌표, 칸 5 mm", "sub", "start")
    s.text(260, 451, "x > 0 쪽이 팔 1 방향 (φ = 0°)", "sub", "start")
    s.write()


# --------------------------------------------------------------------- t08
def t08_lumped_model():
    s = Svg("t08-lumped-model", 760, 380, "동역학 간이 모델: 질량 나누어 붙이기")
    m = (90, 90)
    ang = math.radians(30)
    L = 170
    e = (m[0] + L * math.cos(ang), m[1] + L * math.sin(ang))
    b = (e[0] - 60, e[1] + 190)
    s.circle(*m, 16, "box")
    s.text(m[0], m[1] + 4, "J", "tag")
    s.line(*m, *e, hi=True, width=6, cap=True)
    s.line(*e, *b, width=2.2, style=INK, dash=True)
    mid = ((m[0] + e[0]) / 2, (m[1] + e[1]) / 2)
    s.circle(*mid, 9, "wr")
    s.text(mid[0] + 14, mid[1] - 10, "m_arm @ L/2", "tag", "start")
    s.circle(*e, 11, "wr")
    s.text(e[0] + 16, e[1] - 4, "m_elbow + ½·m_pair @ L", "tag", "start")
    s.line(b[0] - 60, b[1], b[0] + 30, b[1], width=9, style="stroke:var(--accent-dim)", cap=True)
    s.circle(b[0] - 15, b[1], 13, "wr")
    s.text(b[0] - 15, b[1] + 36, "m_p = 판 + 툴 + 페이로드 + 3·½·m_pair", "tag")
    s.text(m[0] - 20, m[1] - 26, "모터 + 감속기 (반사 관성 N²·J)", "tag", "start")
    s.text(m[0] + 6, m[1] + 60, "θ", "lbl")
    s.arc(*m, 42, 0, 30, width=1.1, arrow=True)
    s.line(b[0] + 50, b[1] - 50, b[0] + 50, b[1] + 10, width=1.4, arrow=True)
    s.text(b[0] + 56, b[1] - 20, "g", "lbl", "start")
    s.box(420, 30, 325, 320)
    y = 58
    for txt, cls in (("팔 하나 (출력축 기준)", "lbl"),
                     ("I = m_arm·L²/3 + (m_elbow + ½m_pair)·L²", "sub"),
                     ("      + 구동부 반사 관성", "sub"),
                     ("k_g = m_arm·L/2 + (m_elbow + ½m_pair)·L", "sub"),
                     ("", "sub"),
                     ("이펙터 판 (세 팔 공통)", "lbl"),
                     ("F = m_p·(a − g⃗),  g⃗ = (0, 0, −g)", "sub"),
                     ("", "sub"),
                     ("관절 토크 (τ > 0 = 팔을 아래로)", "lbl"),
                     ("τᵢ = I·θ̈ᵢ − k_g·g·cos θᵢ + (Jᵀ·F)ᵢ", "sub"),
                     ("", "sub"),
                     ("모터축", "lbl"),
                     ("τm = τ / (N·η),  ωm = θ̇·N", "sub")):
        s.text(436, y, txt, cls, "start")
        y += 22
    s.write()


def t08_speed_torque():
    s = Svg("t08-speed-torque", 760, 380, "모터 종류별 속도-토크 곡선 모델")
    x0, y0, W, H = 80, 40, 440, 270
    s.line(x0, y0 + H, x0 + W + 20, y0 + H, arrow=True, width=1.2)
    s.line(x0, y0 + H, x0, y0 - 16, arrow=True, width=1.2)
    s.text(x0 + W + 16, y0 + H + 24, "ω / ωmax", "sub", "end")
    s.text(x0 - 8, y0 - 22, "τ / τpeak", "sub", "start")
    for k in range(0, 5):
        v = k / 4
        s.text(x0 - 8, y0 + H - v * H + 4, "%.2g" % v, "tag", "end")
        s.text(x0 + v * W, y0 + H + 18, "%.2g" % v, "tag")
    curves = (("dxl_xm430_w350", "stroke:var(--accent)", "스마트 서보 / RC 서보"),
              ("stepper_nema17", "stroke:var(--warn)", "스테퍼 (끌어냄 곡선)"),
              ("ac_servo_400w", "stroke:var(--danger)", "AC 서보 (최고속도까지 피크)"))
    base = DeltaDesign.preset("edu_dynamixel")
    for i, (motor, sty, lab) in enumerate(curves):
        d = base.copy(motor=motor, gearbox="none", gear_ratio=1)
        spec = d.motor_spec
        wmax, peak = float(spec["max_speed"]), float(spec["peak_torque"])
        pts = []
        for k in range(0, 201):
            w = wmax * k / 200 * 0.9999
            pts.append((x0 + W * w / wmax, y0 + H - H * dyn.available_torque(d, w) / peak))
        pts.append((x0 + W, y0 + H))
        s.poly(pts, closed=False, style="fill:none;%s;stroke-width:2.4" % sty, width=2.4)
        s.line(560, 70 + 30 * i, 590, 70 + 30 * i, width=2.4, style=sty)
        s.text(600, 75 + 30 * i, lab, "sub", "start")
        # rated torque marker
        rt = float(spec["rated_torque"]) / peak
        s.line(x0, y0 + H - H * rt, x0 + W * 0.15, y0 + H - H * rt, width=1.2, style=sty, dash=True)
    s.text(x0 + W * 0.16, y0 + H - H * 0.33 + 4, "점선: 정격(연속) 토크", "tag", "start")
    s.text(560, 190, "필요한 (ωm, τm) 점이 모두", "sub", "start")
    s.text(560, 210, "곡선 아래에 있어야 하고,", "sub", "start")
    s.text(560, 230, "RMS 토크는 정격 이하여야 함", "sub", "start")
    s.text(560, 262, "곡선: dynamics.available_torque", "tag", "start")
    s.text(560, 280, "카탈로그 대표값 (데이터시트 확인)", "tag", "start")
    s.write()


if __name__ == "__main__":
    t01_serial_vs_parallel()
    t02_structure()
    t02_parallelogram()
    t03_top_view()
    t03_arm_plane()
    t04_ik_arm_plane()
    t05_trilateration()
    t06_singularity()
    t06_cond_map()
    t07_workspace_section()
    t08_lumped_model()
    t08_speed_torque()
