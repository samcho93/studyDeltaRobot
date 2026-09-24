"""Generate the SVG figures used by the control-track chapters (r01-r08).

Run from the repo root:  python content/figures/gen_control.py
Only CSS-variable classes from assets/css/main.css (.fig svg ...) are used, so the
figures follow the light/dark theme.  Writes only files named r0*.svg.
"""
from __future__ import annotations

import math
from pathlib import Path

OUT = Path(__file__).resolve().parent


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Svg:
    def __init__(self, name: str, w: int, h: int, label: str):
        assert name.startswith("r0"), name
        self.name, self.w, self.h, self.label = name, w, h, label
        self.parts: list = []
        self.mid = "arr-" + name  # marker id unique per figure (figures are inlined)

    def add(self, s: str) -> None:
        self.parts.append("  " + s)

    def box(self, x, y, w, h, hi=False, rx=10):
        self.add('<rect class="%s" x="%g" y="%g" width="%g" height="%g" rx="%g"/>'
                 % ("box-hi" if hi else "box", x, y, w, h, rx))

    def text(self, x, y, s, cls="lbl", anchor="middle", extra=""):
        self.add('<text class="%s" x="%g" y="%g" text-anchor="%s"%s>%s</text>'
                 % (cls, x, y, anchor, extra, esc(s)))

    def line(self, x1, y1, x2, y2, hi=False, arrow=False, width=1.6, dash=False, both=False):
        m = "%s%s" % (self.mid, "-hi" if hi else "")
        self.add('<path class="%s" d="M%g %g L%g %g" stroke-width="%g"%s%s%s/>'
                 % ("ln-hi" if hi else "ln", x1, y1, x2, y2, width,
                    ' stroke-dasharray="5 4"' if dash else "",
                    ' marker-end="url(#%s)"' % m if arrow else "",
                    ' marker-start="url(#%s)"' % m if both else ""))

    def path(self, d, hi=False, width=1.6, arrow=False, dash=False):
        self.add('<path class="%s" d="%s" stroke-width="%g"%s%s/>'
                 % ("ln-hi" if hi else "ln", d, width,
                    ' stroke-dasharray="5 4"' if dash else "",
                    ' marker-end="url(#%s%s)"' % (self.mid, "-hi" if hi else "") if arrow else ""))

    def circle(self, cx, cy, r, cls="box"):
        self.add('<circle class="%s" cx="%g" cy="%g" r="%g"/>' % (cls, cx, cy, r))

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


# --------------------------------------------------------------------- r01
def r01_backends():
    s = Svg("r01-backends", 720, 330, "DeltaRobot 하나에 백엔드 네 개")
    s.box(20, 120, 170, 90, hi=True)
    s.text(105, 148, "학생 코드")
    s.text(105, 170, "robot.arch_to(...)", "sub mono")
    s.text(105, 190, "robot.tool_on()", "sub mono")
    s.box(230, 90, 190, 150)
    s.text(325, 116, "DeltaRobot")
    s.text(325, 140, "① 경로 계획 (아치·직선)", "sub")
    s.text(325, 160, "② 속도 프로파일 샘플링", "sub")
    s.text(325, 180, "③ IK + 한계 검사", "sub")
    s.text(325, 200, "④ 모터각 스트림 (10 ms)", "sub")
    s.text(325, 224, "WorkspaceError", "tag mono dg")
    s.line(190, 165, 228, 165, arrow=True, hi=True, width=2)
    rows = [("record", "가상 시간 · Playground", "frames → 시뮬 재생", True),
            ("websim", "PC → 웹 시뮬 실시간", "ws://127.0.0.1:8765", False),
            ("ros2", "delta_driver 노드", "/delta/joint_command", False),
            ("serial", "아두이노 펌웨어", "J d1 d2 d3 (115200)", False)]
    for k, (name, a, b, hi) in enumerate(rows):
        y = 20 + k * 75
        s.path("M420 165 C450 165 450 %g 480 %g" % (y + 30, y + 30), arrow=True, hi=hi)
        s.box(482, y, 218, 62, hi=hi, rx=8)
        s.text(496, y + 22, name, "lbl mono", "start")
        s.text(496, y + 40, a, "sub", "start")
        s.text(496, y + 56, b, "tag mono", "start")
    s.text(325, 290, "backend= 인자만 바꾸면 같은 코드가 시뮬 → PC → ROS 2 → 실물로", "tag")
    s.text(325, 310, "serial·ros2는 모터 속도 초과를 거부, record·websim은 경고만", "sub")
    s.write()


def r01_tcp():
    s = Svg("r01-tcp", 720, 300, "TCP와 이펙터 중심, 좌표 규약")
    # base
    s.box(170, 40, 240, 16, rx=4)
    s.text(290, 32, "베이스 (z = 0, 모터축 평면)", "tag")
    # arms (schematic)
    s.line(200, 48, 170, 110, width=5, hi=True)
    s.line(380, 48, 410, 110, width=5, hi=True)
    s.line(170, 110, 262, 200, width=2)
    s.line(410, 110, 318, 200, width=2)
    s.box(250, 196, 80, 12, rx=3)
    s.circle(290, 202, 4, "hi")
    s.line(290, 208, 290, 252, width=4)
    s.circle(290, 256, 4, "dg")
    s.path("M340 202 L380 202", dash=True)
    s.text(386, 206, "이펙터 중심  robot.effector · kinematics.ik/fk", "tag", "start")
    s.path("M296 256 L380 256", dash=True)
    s.text(386, 260, "TCP (툴 끝)  robot.position · move_to/arch_to", "tag hi", "start")
    s.line(250, 202, 250, 256, both=True, arrow=True)
    s.text(244, 234, "tool.length", "sub mono", "end")
    # axes
    ax, ay = 70, 250
    s.line(ax, ay, ax + 60, ay, arrow=True, hi=True, width=2)
    s.text(ax + 66, ay + 4, "x (팔 1 방향)", "tag", "start")
    s.line(ax, ay, ax, ay - 60, arrow=True, hi=True, width=2)
    s.text(ax, ay - 66, "z (위)", "tag")
    s.text(ax, ay + 24, "작업은 z < 0", "sub")
    s.text(560, 60, "단위: m · rad · kg · s", "lbl")
    s.text(560, 84, "θ = 0: 위팔 수평, 아래로 +", "sub")
    s.text(560, 104, "팔 방위각 φ = 0°, 120°, 240°", "sub")
    s.text(560, 124, "TCP = 이펙터 중심 − (0, 0, tool.length)", "sub mono")
    s.write()


# --------------------------------------------------------------------- r02
def r02_cycle():
    s = Svg("r02-cycle", 720, 290, "색깔 분류 한 사이클의 동작")
    z_tab = 230
    s.line(30, z_tab, 690, z_tab)
    s.text(40, z_tab + 18, "작업대 surface_z", "tag", "start")
    # part and bin
    s.box(110, z_tab - 16, 30, 16, hi=True, rx=2)
    s.text(125, z_tab + 18, "부품 top", "tag")
    s.box(520, z_tab - 14, 110, 14, rx=2)
    s.text(575, z_tab + 18, "상자 (x, y, w, d, h)", "tag")
    # arch path
    s.text(135, 70, "① arch_to(part)", "tag mono", "start")
    s.path("M125 205 L125 110 Q125 90 145 90 L555 90 Q575 90 575 110 L575 200", hi=True, width=2.4, arrow=True)
    s.circle(125, 207, 5, "hi")
    s.text(160, 200, "② tool_on() → 부품 id", "tag mono", "start")
    s.text(160, 216, "   wait(0.1) 흡착 안정", "sub", "start")
    s.text(350, 80, "③ arch_to(bin, place_z)   height = 아치 높이", "tag mono")
    s.circle(575, 202, 5, "hi")
    s.text(565, 180, "④ tool_off() → 상자 판정", "tag mono", "end")
    s.line(660, 92, 660, 200, both=True, arrow=True)
    s.text(652, 140, "height", "sub mono", "end")
    s.text(652, 156, "(높은 쪽 끝 기준)", "sub", "end")
    s.text(360, 270, "place_z = surface_z + bin h + part h + 여유 5 mm   ·   놓는 판정은 x·y가 상자 안인지만 봅니다", "sub")
    s.write()


# --------------------------------------------------------------------- r03
def r03_websim():
    s = Svg("r03-websim", 720, 320, "websim 백엔드 연결 순서")
    s.box(20, 30, 250, 270, hi=True)
    s.text(145, 56, "PC: python my_robot.py")
    s.text(145, 78, "DeltaRobot(backend=\"websim\")", "sub mono")
    s.text(145, 96, "WebSocket 서버 127.0.0.1:8765", "tag mono")
    s.box(450, 30, 250, 270)
    s.text(575, 56, "브라우저: sim/index.html")
    s.text(575, 78, "연결 탭 → [PC Python 연결]", "sub")
    s.text(575, 96, "WebSocket 클라이언트", "tag")
    msgs = [(130, "← 연결 (Origin 검사, 허용 밖이면 403)", False, "l"),
            (160, "← hello {design, scene}", False, "l"),
            (190, "design {design, scene} →", True, "r"),
            (220, "state {t, q[3], tool} → (10 ms마다)", True, "r"),
            (256, "← estop  → EmergencyStop 예외", False, "l")]
    for y, t, hi, d in msgs:
        if d == "r":
            s.line(272, y, 448, y, hi=hi, arrow=True, width=2)
        else:
            s.line(448, y, 272, y, hi=hi, arrow=True, width=2, dash=(y == 256))
        s.text(360, y - 6, t, "tag mono" if y != 256 else "tag mono dg")
    s.text(145, 140, "대기 (wait_connect 60 s)", "sub")
    s.text(145, 200, "설계·장면 확정", "sub")
    s.text(145, 230, "궤적을 실시간으로 전송", "sub")
    s.text(575, 230, "받은 모터각으로 3D 표시", "sub")
    s.text(575, 270, "상단 비상정지 버튼", "sub")
    s.write()


# --------------------------------------------------------------------- r04
def r04_node_graph():
    s = Svg("r04-node-graph", 720, 420, "delta_robot 패키지의 노드와 토픽")
    # nodes
    def node(x, y, w, h, name, sub, hi=False):
        s.box(x, y, w, h, hi=hi, rx=18)
        s.text(x + w / 2, y + h / 2 - 2, name, "lbl mono")
        s.text(x + w / 2, y + h / 2 + 16, sub, "sub")
    node(20, 40, 170, 56, "delta_commander", "goal → 궤적")
    node(20, 170, 170, 56, "deltarobot 클라이언트", "backend=\"ros2\"")
    node(275, 150, 170, 76, "delta_driver", "mock | serial", hi=True)
    node(530, 40, 170, 56, "robot_state_publisher", "URDF → /tf")
    node(530, 150, 170, 56, "rviz2", "RobotModel · TF")
    node(275, 320, 170, 60, "delta_web_bridge", "ws://127.0.0.1:8765")
    s.box(530, 320, 170, 60, rx=8)
    s.text(615, 346, "웹 시뮬레이터")
    s.text(615, 364, "연결 탭 [PC Python 연결]", "sub")
    # edges
    s.path("M190 68 C240 68 240 170 273 170", arrow=True, hi=True, width=2)
    s.text(232, 96, "/delta/joint_command", "tag mono", "middle")
    s.line(190, 198, 273, 198, arrow=True, hi=True, width=2)
    s.text(232, 190, "joint_command", "tag mono")
    s.text(232, 214, "tool_command", "tag mono")
    s.path("M445 175 C490 175 490 68 528 68", arrow=True, width=2)
    s.text(490, 108, "/joint_states", "tag mono", "start")
    s.line(615, 96, 615, 148, arrow=True)
    s.text(622, 128, "/tf", "tag mono", "start")
    s.path("M445 205 L500 205 L500 290 L360 290 L360 318", arrow=True)
    s.text(505, 262, "/joint_states", "tag mono", "start")
    s.text(505, 278, "/delta/tool_state", "tag mono", "start")
    s.line(445, 350, 528, 350, both=True, arrow=True, width=2)
    s.text(486, 342, "WebSocket", "tag")
    s.path("M275 350 L105 350 L105 98", arrow=True, dash=True)
    s.text(110, 300, "/delta/goal", "tag mono", "start")
    s.path("M300 320 L300 228", arrow=True, dash=True)
    s.text(306, 270, "/delta/estop", "tag mono dg", "start")
    s.text(22, 20, "입력: /delta/goal (Point, m) · 서비스 /delta/tool (SetBool)   출력: /delta/busy · /delta/tcp", "tag", "start")
    s.text(360, 408, "실선: 명령·상태 흐름   점선: 웹 시뮬에서 들어오는 goal·비상정지", "sub")
    s.write()


# --------------------------------------------------------------------- r05
def r05_tree():
    s = Svg("r05-tree", 720, 380, "닫힌 사슬을 URDF 트리로 바꾸는 방법")
    # left: closed chain schematic
    s.text(150, 26, "실제 기구: 닫힌 사슬", "lbl")
    s.box(60, 50, 180, 14, rx=4)
    s.text(150, 44, "base_link", "sub mono")
    for x0, x1 in ((80, 60), (150, 150), (220, 240)):
        s.line(x0, 57, x1, 130, width=4, hi=True)
        s.line(x1, 130, 150 + (x1 - 150) * 0.25, 260, width=1.6)
    s.box(120, 256, 60, 12, rx=3)
    s.text(150, 290, "effector (세 팔이 한 판에서 만남)", "sub")
    s.text(150, 320, "→ 링크 하나에 부모가 셋: URDF 불가", "tag dg")
    # right: tree
    x = 330
    s.text(x + 180, 26, "URDF: 트리 + 가상 관절", "lbl")
    s.box(x, 40, 130, 30, hi=True, rx=6)
    s.text(x + 65, 60, "base_link", "lbl mono")
    items = [(90, "motor{i}_joint", "revolute · 능동", True),
             (135, "elbow{i}a_pitch / yaw", "수동 (계산값)", False),
             (180, "elbow{i}b_pitch / yaw", "mimic → a", False)]
    for y, a, b, hi in items:
        s.line(x + 20, 70, x + 20, y + 10)
        s.line(x + 20, y + 10, x + 40, y + 10)
        s.text(x + 46, y + 14, a, "tag mono" + (" hi" if hi else ""), "start")
        s.text(x + 230, y + 14, b, "sub", "start")
    s.text(x + 46, 222, "upper_arm{i} → elbow{i}{a,b}_link → forearm{i}{a,b}  (끝이 열려 있음)", "sub mono", "start")
    s.line(x + 10, 70, x + 10, 262)
    s.line(x + 10, 262, x + 40, 262)
    s.text(x + 46, 266, "effector_x → effector_y → effector_z", "tag mono hi", "start")
    s.text(x + 46, 284, "prismatic · 가상 (FK로 계산)", "sub", "start")
    s.text(x + 46, 308, "→ effector → tool_link → tcp (fixed)", "sub mono", "start")
    s.box(x, 326, 370, 44, rx=8)
    s.text(x + 185, 344, "delta_driver가 모터각 θ로 FK·수동 관절각을 계산해", "sub")
    s.text(x + 185, 362, "/joint_states에 모두 발행 → 끝이 맞아 닫힌 것처럼 보임", "sub")
    s.write()


# --------------------------------------------------------------------- r06
def r06_paths():
    s = Svg("r06-paths", 720, 300, "ROS 2에서 로봇을 움직이는 세 가지 길")
    ys = [40, 120, 200]
    items = [("ros2 topic pub /delta/goal", "geometry_msgs/Point (TCP, m)", "delta_commander가 궤적 생성"),
             ("ros2 service call /delta/tool", "std_srvs/SetBool", "흡착/그리퍼 on·off"),
             ("DeltaRobot(backend=\"ros2\")", "파이썬이 직접 궤적 계획", "/delta/joint_command 스트림")]
    for y, (a, b, c) in zip(ys, items):
        s.box(20, y, 280, 62, hi=(y == 200), rx=8)
        s.text(34, y + 24, a, "lbl mono", "start")
        s.text(34, y + 44, b, "sub mono", "start")
        s.line(300, y + 31, 440, 150, arrow=True, hi=(y == 200), width=2)
        s.text(370, y + 31 + (150 - y - 31) / 2 - 6, c, "tag")
    s.box(442, 110, 130, 80, hi=True, rx=16)
    s.text(507, 146, "delta_driver", "lbl mono")
    s.text(507, 166, "한계·속도 검사", "sub")
    s.line(572, 150, 620, 150, arrow=True, width=2)
    s.box(622, 110, 88, 80, rx=8)
    s.text(666, 146, "mock")
    s.text(666, 166, "/ serial", "sub")
    s.text(360, 286, "어느 길이든 /delta/estop(Bool true)이 들어오면 드라이버가 멈춥니다", "wr", extra=' font-size="12"')
    s.write()


# --------------------------------------------------------------------- r07
def r07_wiring():
    s = Svg("r07-wiring", 720, 360, "RC 서보 델타 배선도")
    s.box(20, 30, 170, 110, hi=True)
    s.text(105, 56, "Arduino Uno/Nano")
    s.text(105, 78, "D9 · D10 · D11 → 서보", "sub mono")
    s.text(105, 96, "D7 → 툴 (MOSFET)", "sub mono")
    s.text(105, 114, "USB ↔ PC 115200", "sub mono")
    s.box(20, 220, 170, 90)
    s.text(105, 248, "PC")
    s.text(105, 268, "backend=\"serial\"", "sub mono")
    s.text(105, 288, "port=\"COM3\" / /dev/ttyACM0", "tag mono")
    s.line(105, 218, 105, 142, both=True, arrow=True, width=2)
    s.text(112, 185, "USB", "tag", "start")
    s.box(270, 220, 170, 90)
    s.text(355, 248, "서보 전원 6 V 5 A")
    s.text(355, 268, "USB 전원 사용 금지", "sub dg")
    s.text(355, 288, "GND를 아두이노와 공통", "tag")
    for k in range(3):
        y = 30 + k * 62
        s.box(520, y, 180, 48, rx=8)
        s.text(610, y + 20, "MG996R 서보 %d" % (k + 1))
        s.text(610, y + 38, "신호 D%d · 6 V · GND" % (9 + k), "sub mono")
        s.line(190, 60 + k * 10, 518, y + 24, hi=True, width=1.6, arrow=True)
        s.line(440, 240, 518, y + 30, width=1.4, dash=True)
    s.box(520, 230, 180, 70, rx=8)
    s.text(610, 256, "툴: 흡착 펌프 / 전자석")
    s.text(610, 276, "MOSFET 모듈 또는 릴레이", "sub")
    s.text(610, 292, "(유도성 부하는 다이오드)", "tag")
    s.line(190, 110, 518, 262, width=1.6, arrow=True)
    s.text(360, 340, "점선: 6 V 서보 전원 — 신호선(실선)과 GND는 반드시 공통으로 연결", "sub")
    s.write()


def r07_calib():
    s = Svg("r07-calib", 720, 280, "모터각 영점과 방향 캘리브레이션")
    cx, cy = 180, 90
    s.box(cx - 60, cy - 40, 60, 80, rx=6)
    s.text(cx - 30, cy + 58, "서보", "sub")
    s.circle(cx, cy, 6, "hi")
    s.line(cx, cy, cx + 150, cy, hi=True, width=6)
    s.text(cx + 160, cy + 4, "θ = 0° : 위팔 수평", "lbl", "start")
    a = math.radians(35)
    s.line(cx, cy, cx + 150 * math.cos(a), cy + 150 * math.sin(a), width=4, dash=True)
    s.text(cx + 150 * math.cos(a) + 8, cy + 150 * math.sin(a) + 4, "θ = +35° : 아래로 (홈 0.35 rad ≈ 20°)", "tag", "start")
    s.path("M%g %g A60 60 0 0 1 %g %g" % (cx + 60, cy, cx + 60 * math.cos(a), cy + 60 * math.sin(a)), arrow=True)
    s.text(cx + 70, cy + 34, "+", "lbl hi", "start")
    s.box(20, 190, 680, 76, rx=8)
    s.text(36, 214, "펌웨어 변환:  서보 명령[°] = 90 + 방향ᵢ × θᵢ + 오프셋ᵢ", "lbl mono", "start")
    s.text(36, 236, "방향ᵢ = +1 또는 −1 (서보 장착 방향) · 오프셋ᵢ = 수평에서 읽은 보정값 · θ는 −40°…+85°로 제한", "sub", "start")
    s.text(36, 256, "세 팔이 모두 수평이면 이펙터는 베이스 아래 정중앙 — 자로 대칭을 확인", "tag", "start")
    s.write()


# --------------------------------------------------------------------- r08
def r08_intercept():
    s = Svg("r08-intercept", 720, 330, "컨베이어 부품과 로봇이 만나는 시각 계산")
    x0, y0, w, h = 70, 30, 420, 230
    s.line(x0, y0 + h, x0 + w, y0 + h, arrow=True)
    s.line(x0, y0 + h, x0, y0, arrow=True)
    s.text(x0 + w, y0 + h + 20, "시간 t", "tag", "end")
    s.text(x0 - 8, y0 + 8, "x", "tag", "end")
    # part line x(t) = x + v t
    s.line(x0, y0 + h - 30, x0 + w - 20, y0 + 40, hi=True, width=2.4)
    s.text(x0 + w - 16, y0 + 36, "부품 x(t) = x₀ + v·(t − t₀)", "tag hi", "start")
    # robot arrival estimates
    s.circle(x0 + 30, y0 + h - 30 - 30 * 190 / 400, 4, "hi")
    s.text(x0 + 30, y0 + h - 60, "지금 t₀", "tag")
    pts = [(170, "Δt₁"), (250, "Δt₂"), (270, "Δt₃ ≈ Δt₂")]
    for k, (tx, lab) in enumerate(pts):
        yy = y0 + h - 30 - (tx) * 190 / 400
        s.line(x0 + tx, y0 + h, x0 + tx, yy, dash=True)
        s.circle(x0 + tx, yy, 4, "dg" if k == 2 else "box")
        s.text(x0 + tx, y0 + h + 18, lab, "sub")
    s.box(510, 30, 200, 230, rx=8)
    s.text(610, 54, "반복 계산 (3~5회)", "lbl")
    rows = ["Δt ← 0", "x_hit = x₀ + v·Δt", "경로 = arch_path(현재, x_hit)", "Δt ← Profile(경로 길이).T",
            "→ 수렴하면 arch_to(x_hit)", "", "v ≪ 로봇 속도라 빨리 수렴"]
    for k, t in enumerate(rows):
        s.text(526, 82 + k * 24, t, "sub mono" if k < 5 else "tag", "start")
    s.text(360, 312, "도착 시각에 부품이 있을 x를 겨냥 → 도착 즉시 tool_on(), 컨베이어 속도로 잠깐 따라가며 흡착", "sub")
    s.write()


if __name__ == "__main__":
    r01_backends()
    r01_tcp()
    r02_cycle()
    r03_websim()
    r04_node_graph()
    r05_tree()
    r06_paths()
    r07_wiring()
    r07_calib()
    r08_intercept()
