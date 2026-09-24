// playground-examples.js — example programs for the Python Playground.
// Every example is executed by tests/test_playground_examples.py with the record backend.
// group: shown as <optgroup> in the example selector.

export const EXAMPLES = [
  {
    id: 'first', group: '기초', title: '첫 동작: 사각형 그리며 움직이기',
    code: `from deltarobot import DeltaRobot

robot = DeltaRobot()          # 시뮬레이터에서 고른 설계 그대로 (없으면 기본 설계)
print(robot)
robot.home()
x, y, z = robot.position
print("홈 TCP 위치 [m]:", round(x, 3), round(y, 3), round(z, 3))

z_work = z - 0.03             # 홈보다 3 cm 아래
d = 0.04                      # 사각형 반 변 4 cm
for px, py in [(d, d), (-d, d), (-d, -d), (d, -d), (d, d)]:
    robot.move_to(px, py, z_work)
    print("  도착 (%.3f, %.3f, %.3f) at t=%.2f s" % (*robot.position, robot.time))
robot.home()
print("모터각 [deg]:", [round(v * 57.2958, 1) for v in robot.joints])
`,
  },
  {
    id: 'ikfk', group: '기초', title: '역기구학·순기구학 확인',
    code: `import math
from deltarobot import DeltaRobot, kinematics as kin

robot = DeltaRobot()
d = robot.design
print("R=%.3f r=%.3f L=%.3f l=%.3f (m)" % (d.base_radius, d.effector_radius, d.upper_arm, d.forearm))

p = (0.03, -0.02, robot.position[2] - 0.02)          # TCP 목표
theta = robot.ik(*p)                                  # TCP 기준 IK
print("IK  θ =", [round(math.degrees(t), 2) for t in theta], "deg")
back = robot.fk(*theta)
print("FK  p =", [round(v, 5) for v in back], " (원래 목표로 돌아와야 함)")

# 라이브러리 kinematics 함수는 '이펙터 중심' 좌표를 씁니다
eff = (p[0], p[1], p[2] + d.tool_length)
print("이펙터 중심 IK =", [round(math.degrees(t), 2) for t in kin.ik(d, eff)])
print("조건수 κ =", round(kin.condition_number(d, theta), 3))
rep = kin.limit_report(d, eff)
print("한계 검사:", "OK" if rep["ok"] else rep["problems"])
`,
  },
  {
    id: 'draw', group: '기초', title: '펜으로 원·별 그리기 (move_path)',
    code: `import math
from deltarobot import DeltaRobot, DeltaDesign

design = DeltaDesign.preset("edu_dynamixel").copy(tool="pen")
robot = DeltaRobot(design, scene="drawing")
z = robot.scene.surface_z + 0.001            # 종이 표면
R = 0.06

circle = [(R * math.cos(a), R * math.sin(a), z) for a in [2 * math.pi * k / 90 for k in range(91)]]
robot.arch_to(*circle[0], height=0.02)
robot.pen_down()
robot.move_path(circle[1:], speed=0.08, accel=0.8)     # 한 번의 속도 프로파일로 연속 주행
robot.pen_up()
print("원: %.2f s" % robot.time)

star = []
for k in range(11):
    a = math.pi / 2 + math.pi * k / 5
    rr = 0.045 if k % 2 == 0 else 0.018
    star.append((rr * math.cos(a), rr * math.sin(a), z))
robot.arch_to(*star[0], height=0.02)
robot.pen_down()
for p in star[1:]:
    robot.move_to(*p, speed=0.08, accel=0.8)          # 꼭짓점마다 멈춤
robot.pen_up()
robot.move_by(dz=0.03)
print("별까지: %.2f s" % robot.time)
`,
  },
  {
    id: 'workspace', group: '기초', title: '작업영역·작업 실린더 계산',
    code: `from deltarobot import DeltaDesign, kinematics as kin

d = DeltaDesign.preset("edu_dynamixel")
cyl = kin.work_cylinder(d, height=0.10)
print("작업 실린더: 지름 %.0f mm × 높이 %.0f mm (이펙터 z %.0f ~ %.0f mm)" % (
    cyl["diameter"] * 1000, cyl["height"] * 1000, cyl["z_top"] * 1000, cyl["z_bottom"] * 1000))
print("높이별 최대 반지름:")
for z, r in cyl["profile"][::4]:
    bar = "#" * int(r * 200)
    print("  z=%6.0f mm  r=%5.0f mm  %s" % (z * 1000, r * 1000, bar))
`,
  },
  {
    id: 'sort', group: '현장 응용', title: '색깔 분류 pick & place',
    code: `from deltarobot import DeltaRobot, WorkspaceError

robot = DeltaRobot(scene="pick_place")
tool = robot.design.tool_spec
print("툴:", tool["name"], "| 부품", len(robot.parts()), "개 | 상자", [b["id"] + ":" + b["color"] for b in robot.bins()])
robot.home()

for part in robot.parts():
    if tool["ferrous_only"] and part["material"] != "steel":
        print("  건너뜀 (전자석은 철만):", part["id"])
        continue
    box = next(b for b in robot.bins() if b["color"] == part["color"])
    z_pick = part["top"] if tool["kind"] != "gripper" else part["z"] + part["h"] / 2
    try:
        robot.arch_to(part["x"], part["y"], z_pick, height=0.03)
        got = robot.tool_on()
        robot.wait(0.1)
        robot.arch_to(box["x"], box["y"], robot.scene.surface_z + box["h"] + part["h"] + 0.02, height=0.03)
        robot.tool_off()
        print("  %s -> 상자 %s (t=%.2f s)" % (got, box["id"], robot.time))
    except WorkspaceError as e:
        print("  실패:", e)

robot.home()
print("결과:", robot.scene.score())
r = robot.analyze()
print("모터 검증: 피크 %.0f%%  RMS %.0f%%  속도 %.0f%%  ->" % (r["peak_ratio"] * 100, r["rms_ratio"] * 100, r["speed_ratio"] * 100),
      "통과" if r["ok"] else r["problems"])
`,
  },
  {
    id: 'conveyor', group: '현장 응용', title: '컨베이어 트래킹 (벨트 속도 맞춰 집기)',
    code: `from deltarobot import DeltaRobot, WorkspaceError

robot = DeltaRobot(scene="conveyor")
conv = robot.scene.conveyor()
v = conv["speed"]
print("컨베이어 속도 %.3f m/s" % v)
robot.home()
tried = set()

while robot.time < 30:
    # 가장 하류(x가 큰) 부품부터, 1.2초 뒤에도 닿는 것만
    cands = sorted((p for p in robot.parts() if p["on"] == "conveyor" and p["id"] not in tried),
                   key=lambda p: -p["x"])
    part = next((p for p in cands if robot.reachable(p["x"] + v * 1.2, p["y"], robot.pick_z(p))), None)
    if part is None:
        robot.wait(0.1)
        continue
    tried.add(part["id"])
    try:
        got = robot.track_pick(part)       # 벨트와 같은 속도로 내려가 잡고, 따라가다 올라옴
    except WorkspaceError as e:
        print("건너뜀:", e)
        continue
    if got:
        box = next(b for b in robot.bins() if b["color"] == part["color"])
        robot.arch_to(box["x"], box["y"], robot.scene.surface_z + box["h"] + part["h"] + 0.02)
        robot.tool_off()
        print("  %s -> 상자 %s (t=%.1f s)" % (got, box["id"], robot.time))

print("결과:", robot.scene.score())
`,
  },
  {
    id: 'tray', group: '현장 응용', title: '제과 포장: 쿠키를 트레이 3×4 칸에 담기',
    code: `# [현장] 제과 포장 라인 — 흩어진 쿠키를 흡착컵으로 집어 트레이(3×4 칸)에 담기
#  · 사용자 정의 씬(부품·트레이 배치)을 dict로 만들어 DeltaRobot(scene=...)에 넘깁니다.
#  · 다음 쿠키를 '지금 위치에서 가장 가까운 것'으로 골라 이동 거리를 줄이고, 분당 처리량(PPM)을 계산합니다.
import math, random
from deltarobot import DeltaRobot, DeltaDesign
from deltarobot.scene import default_scene

design = DeltaDesign.preset("edu_dynamixel").copy(tool="suction_20", payload=0.03)
sc = default_scene(design, "pick_place")          # 작업대 높이·크기를 설계에 맞춘 기본 씬
rc = sc["work_radius"]                            # 작업 실린더 반지름 [m]
s, h = 0.02, 0.008                                # 쿠키: 20 mm × 8 mm
pitch = 0.026                                     # 트레이 칸 간격 26 mm

random.seed(7)
sc["parts"] = []
for k in range(12):                               # 공급 컨베이어 끝에 쌓인 쿠키 (살짝 흐트러짐)
    x = -0.62 * rc + (k % 3) * 0.2 * rc + random.uniform(-0.01, 0.01)
    y = (k // 3 - 1.5) * 0.28 * rc + random.uniform(-0.008, 0.008)
    sc["parts"].append({"id": "k%d" % (k + 1), "x": round(x, 4), "y": round(y, 4), "size": s, "h": h,
                        "color": "brown", "material": "food"})
tray = {"id": "T", "x": round(0.45 * rc, 4), "y": 0.0, "w": 3 * pitch + 0.01, "d": 4 * pitch + 0.01,
        "h": 0.006, "color": "white"}
sc["bins"] = [tray]

robot = DeltaRobot(design, scene=sc)
robot.set_speed(0.5, 5.0)
robot.home()
slots = [(tray["x"] + (i - 1) * pitch, tray["y"] + (j - 1.5) * pitch) for j in range(4) for i in range(3)]
z_place = sc["surface_z"] + tray["h"] + h + 0.004

t0 = robot.time
todo = robot.parts()
for sx, sy in slots:
    # 다음에 집을 쿠키: 로봇이 지금 있는 곳(직전 칸)에서 가장 가까운 것 (greedy nearest neighbor)
    x, y, _ = robot.position
    part = min(todo, key=lambda p: math.hypot(p["x"] - x, p["y"] - y))
    todo.remove(part)
    robot.arch_to(part["x"], part["y"], robot.pick_z(part), height=0.02)
    got = robot.tool_on()
    robot.wait(0.05)                              # 진공 형성 시간
    robot.arch_to(sx, sy, z_place, height=0.02)
    robot.tool_off()
    robot.wait(0.03)                              # 진공 파기
    print("  %s -> 칸 (%.0f, %.0f) mm  t=%.2f s" % (got, sx * 1000, sy * 1000, robot.time))
robot.home()

n = robot.scene.score()["per_bin"]["T"]
ppm = n / (robot.time - t0) * 60
print("트레이 %d/12 칸 채움 | 작업 %.1f s | 처리량 %.0f 개/분" % (n, robot.time - t0, ppm))
r = robot.analyze()
print("모터 여유: 피크 %.0f%%, RMS %.0f%% ->" % (r["peak_ratio"] * 100, r["rms_ratio"] * 100), "통과" if r["ok"] else r["problems"])
`,
  },
  {
    id: 'reject', group: '현장 응용', title: '품질 검사: 불량품만 컨베이어에서 배출',
    code: `# [현장] 품질 검사 라인 — 비전 카메라가 판정한 불량품(빨강)만 컨베이어에서 골라 배출
#  · 카메라는 벨트 상류(x_cam)에 있어서, 그 지점을 지난 부품만 양품/불량을 알 수 있습니다.
#  · 양품(초록)은 그대로 흘려보내고, 불량만 벨트 속도에 맞춰 집어(track_pick) 불량함에 넣습니다.
import random
from deltarobot import DeltaRobot, DeltaDesign, WorkspaceError
from deltarobot.scene import default_scene

design = DeltaDesign.preset("edu_dynamixel").copy(tool="suction_20")
sc = default_scene(design, "conveyor")
conv = sc["conveyor"]
rc = sc["work_radius"]
random.seed(11)
spawn = []
for n in range(24):                                   # 0.9초 간격으로 들어오는 제품, 약 25%가 불량
    bad = random.random() < 0.25
    spawn.append({"t": round(0.5 + 0.9 * n, 3), "y_off": round(random.uniform(-0.1, 0.1) * rc, 4),
                  "color": "red" if bad else "green", "material": "plastic"})
conv["spawn"] = spawn
sc["bins"] = [{"id": "NG", "x": round(0.1 * rc, 4), "y": round(0.5 * rc, 4), "w": round(0.4 * rc, 4),
               "d": round(0.3 * rc, 4), "h": 0.01, "color": "red"}]

robot = DeltaRobot(design, scene=sc)
v = conv["speed"]
x_cam = conv["x_start"] + 0.15 * (conv["x_end"] - conv["x_start"])
print("벨트 %.0f mm/s, 카메라 위치 x=%.0f mm, 제품 %d개 (불량 %d개)" % (
    v * 1000, x_cam * 1000, len(spawn), sum(s["color"] == "red" for s in spawn)))
robot.home()
seen, picked = set(), []

while robot.time < 26:
    # 카메라를 지난 불량품 중 가장 하류의 것 (곧 사라질 것부터)
    bad = [p for p in robot.parts() if p["on"] == "conveyor" and p["x"] >= x_cam
           and p["color"] == "red" and p["id"] not in seen]
    bad.sort(key=lambda p: -p["x"])
    part = next((p for p in bad if robot.reachable(p["x"] + v * 1.2, p["y"], robot.pick_z(p))), None)
    if part is None:
        robot.wait(0.05)
        continue
    seen.add(part["id"])
    try:
        got = robot.track_pick(part, hover=0.015, descent_time=0.25)
    except WorkspaceError as e:
        print("  놓침 %s: %s" % (part["id"], e))
        continue
    if got:
        box = robot.bins()[0]
        robot.arch_to(box["x"], box["y"], sc["surface_z"] + box["h"] + conv["h"] + 0.015)
        robot.tool_off()
        picked.append(got)
        print("  [%.1f s] 불량 %s 배출" % (robot.time, got))
robot.home()

total_bad = sum(s["color"] == "red" for s in spawn)
print("불량 %d개 중 %d개 배출, 놓친 불량 %d개" % (total_bad, len(picked), total_bad - len(picked)))
color_of = {"c%d" % (n + 1): sp["color"] for n, sp in enumerate(spawn)}
wrong = [e["id"] for e in robot.scene.events if e["type"] == "place" and color_of[e["id"]] != "red"]
print("오배출(양품을 불량함에 넣은 수): %d" % len(wrong))
`,
  },
  {
    id: 'pcb', group: '현장 응용', title: '전자 조립: 칩 부품 PCB 정밀 배치 (2단 접근)',
    code: `# [현장] 전자 조립 — 피더의 칩 부품을 PCB 좌표표(배치 데이터)대로 정밀 배치
#  · 빠르게 이동(아치) → 목표 10 mm 위에서 멈춤 → 느린 속도로 수직 접근 (2단 접근)
#  · 실제 칩 마운터처럼 'PCB 원점 + 부품 좌표' 표를 로봇 좌표로 바꿔 씁니다.
from deltarobot import DeltaRobot, DeltaDesign
from deltarobot.scene import default_scene

design = DeltaDesign.preset("edu_dynamixel").copy(tool="suction_20", payload=0.01)
sc = default_scene(design, "pick_place")
rc = sc["work_radius"]
chip, chip_h = 0.012, 0.004
# PCB (초록 기판) — 원점은 기판 왼쪽 아래 모서리
pcb = {"id": "PCB", "x": round(0.3 * rc, 4), "y": 0.0, "w": 0.09, "d": 0.07, "h": 0.0016, "color": "green"}
origin = (pcb["x"] - pcb["w"] / 2, pcb["y"] - pcb["d"] / 2)
# 배치 데이터 (부품 이름, 기판 좌표 x, y [mm]) — CAD에서 내보낸 pick & place 파일과 같은 형식
placement = [("U1", 25, 35), ("U2", 65, 35), ("C1", 12, 12), ("C2", 45, 12), ("C3", 78, 12),
             ("R1", 12, 58), ("R2", 45, 58), ("R3", 78, 58)]
# 피더: 테이프에서 부품이 한 줄로 공급됨
sc["parts"] = [{"id": name, "x": round(-0.6 * rc + 0.0, 4), "y": round((k - 3.5) * 0.02, 4), "size": chip,
                "h": chip_h, "color": "black" if name[0] == "U" else "yellow", "material": "plastic"}
               for k, (name, _, _) in enumerate(placement)]
sc["bins"] = [pcb]

robot = DeltaRobot(design, scene=sc)
robot.home()
FAST, SLOW, APPROACH = 0.5, 0.03, 0.010        # 이동 0.5 m/s, 접근 30 mm/s, 접근 거리 10 mm


def approach_and(x, y, z, action):
    """(x, y, z+APPROACH)까지 빠르게, 그 뒤 z까지 천천히 내려가 action() 후 천천히 올라옴."""
    robot.arch_to(x, y, z + APPROACH, height=0.015, speed=FAST, accel=5.0)
    robot.move_to(x, y, z, speed=SLOW, accel=0.5)
    result = action()
    robot.wait(0.05)
    robot.move_to(x, y, z + APPROACH, speed=SLOW, accel=0.5)
    return result


t0 = robot.time
for name, bx, by in placement:
    part = next(p for p in robot.parts() if p["id"] == name)
    t_start = robot.time
    got = approach_and(part["x"], part["y"], part["top"], robot.tool_on)
    tx, ty = origin[0] + bx / 1000, origin[1] + by / 1000
    z_on_board = sc["surface_z"] + pcb["h"] + chip_h
    approach_and(tx, ty, z_on_board, robot.tool_off)
    print("  %-3s 피더 -> 기판 (%2d, %2d) mm = 로봇 (%.1f, %.1f) mm   %.2f s" % (
        got, bx, by, tx * 1000, ty * 1000, robot.time - t_start))
robot.home()

placed = robot.scene.score()["per_bin"]["PCB"]
print("배치 %d/%d | 총 %.1f s, 부품당 %.2f s" % (placed, len(placement), robot.time - t0, (robot.time - t0) / len(placement)))
print("느린 접근 구간 비율: %.0f%% (정밀도를 위해 쓰는 시간)" % (
    100 * len(placement) * 4 * (APPROACH / SLOW) / (robot.time - t0)))
`,
  },
  {
    id: 'vision', group: '현장 응용', title: '비전 가이드: 카메라 좌표 → 로봇 좌표 보정',
    code: `# [현장] 비전 가이드 픽킹 — 카메라 픽셀 좌표를 로봇 좌표로 바꾸는 2D 캘리브레이션
#  · 천장 카메라는 약간 회전·이동되어 있고 축척도 모릅니다 (실제 현장과 같음).
#  · 작업대의 기준 마크 3개를 로봇으로 찍어(티칭) 픽셀↔로봇 대응을 얻고, 아핀 변환을 풉니다.
#  · 그 변환으로 카메라가 찾은 부품 위치를 로봇 좌표로 바꿔 색깔별로 분류합니다.
import math, random
from deltarobot import DeltaRobot, DeltaDesign

robot = DeltaRobot(DeltaDesign.preset("edu_dynamixel"), scene="pick_place")
rc = robot.scene.data["work_radius"]
random.seed(5)

# ---- 로봇은 모르는 '진짜' 카메라 모델 (시뮬레이션용): 회전 4°, 0.35 mm/px, 원점 이동
ANG, MM_PER_PX, CX, CY = math.radians(4.0), 0.35, 640, 480
def camera_sees(x, y):
    """로봇 좌표 [m] -> 이미지 픽셀 (+ 0.3 px 검출 잡음)."""
    u = (math.cos(ANG) * x + math.sin(ANG) * y) * 1000 / MM_PER_PX + CX
    v = (math.sin(ANG) * x - math.cos(ANG) * y) * 1000 / MM_PER_PX + CY     # 이미지 v축은 아래 방향
    return u + random.gauss(0, 0.3), v + random.gauss(0, 0.3)

# ---- 1) 캘리브레이션: 기준 마크 3개를 로봇 TCP로 찍고, 카메라에서 같은 마크의 픽셀을 읽음
marks = [(-0.5 * rc, -0.5 * rc), (0.5 * rc, -0.5 * rc), (0.0, 0.55 * rc)]
pix = [camera_sees(*m) for m in marks]
z_touch = robot.scene.surface_z + 0.005
for mx, my in marks:
    robot.arch_to(mx, my, z_touch, height=0.02)       # 티칭: 마크 위로 가서 좌표 확인
    print("  마크 로봇 (%.1f, %.1f) mm  <->  픽셀 (%.1f, %.1f)" % (mx * 1000, my * 1000, *pix[marks.index((mx, my))]))

def solve3(M, b):
    """3x3 연립방정식 (크래머 공식)."""
    det = lambda A: (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                     + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))
    D = det(M)
    out = []
    for c in range(3):
        A = [row[:] for row in M]
        for r in range(3):
            A[r][c] = b[r]
        out.append(det(A) / D)
    return out

# x = a·u + b·v + c,  y = d·u + e·v + f
M = [[u, v, 1.0] for u, v in pix]
ax = solve3(M, [m[0] for m in marks])
ay = solve3(M, [m[1] for m in marks])
to_robot = lambda u, v: (ax[0] * u + ax[1] * v + ax[2], ay[0] * u + ay[1] * v + ay[2])
scale = math.hypot(ax[0], ay[0]) * 1000
print("추정: %.3f mm/px, 회전 %.2f°  (진짜: %.3f mm/px, %.2f°)" % (
    scale, math.degrees(math.atan2(ay[0], ax[0])), MM_PER_PX, math.degrees(ANG)))

# ---- 2) 비전 검출 -> 로봇 좌표 -> 분류
robot.home()
errs = []
for part in robot.parts():
    u, v = camera_sees(part["x"], part["y"])               # 카메라가 본 픽셀 좌표 (로봇은 이것만 앎)
    x, y = to_robot(u, v)                                   # 높이(z)는 작업대 높이로 이미 알고 있음
    errs.append(math.hypot(x - part["x"], y - part["y"]) * 1000)
    robot.arch_to(x, y, robot.pick_z(part), height=0.03)
    got = robot.tool_on()
    box = next(b for b in robot.bins() if b["color"] == part["color"])
    robot.arch_to(box["x"], box["y"], robot.scene.surface_z + box["h"] + part["h"] + 0.02, height=0.03)
    robot.tool_off()
    print("  픽셀 (%6.1f, %6.1f) -> (%6.1f, %6.1f) mm  오차 %.2f mm  %s -> %s" % (
        u, v, x * 1000, y * 1000, errs[-1], got, box["id"]))
robot.home()
print("위치 오차 평균 %.2f mm, 최대 %.2f mm | 결과 %s" % (sum(errs) / len(errs), max(errs), robot.scene.score()))
`,
  },
  {
    id: 'kitting', group: '현장 응용', title: '물류 키팅: 주문서대로 세트 구성·결품 보고',
    code: `# [현장] 물류 키팅 — 주문서대로 부품을 골라 주문별 상자에 세트로 담기
#  · 재고 선반의 부품(색 = 품목)을 주문 A·B·C에 필요한 수량만큼 담습니다.
#  · 재고가 모자라면 담을 수 있는 만큼만 담고 '결품'을 보고합니다 (현장 WMS와 같은 흐름).
from deltarobot import DeltaRobot, DeltaDesign
from deltarobot.scene import default_scene

design = DeltaDesign.preset("edu_dynamixel").copy(tool="suction_20")
sc = default_scene(design, "pick_place")
rc = sc["work_radius"]
size, h = sc["parts"][0]["size"], sc["parts"][0]["h"]
stock = {"red": 4, "blue": 3, "yellow": 3}                 # 선반 재고
sc["parts"] = []
for row, (item, n) in enumerate(stock.items()):
    for k in range(n):
        sc["parts"].append({"id": "%s%d" % (item[0].upper(), k + 1), "x": round(-0.62 * rc + k * 0.17 * rc, 4),
                            "y": round((row - 1) * 0.3 * rc, 4), "size": size, "h": h,
                            "color": item, "material": "plastic"})
sc["bins"] = [{"id": o, "x": round(0.5 * rc, 4), "y": round((j - 1) * 0.36 * rc, 4), "w": round(0.3 * rc, 4),
               "d": round(0.3 * rc, 4), "h": round(0.8 * h, 4), "color": "white"} for j, o in enumerate("ABC")]
orders = {"A": {"red": 2, "blue": 1}, "B": {"red": 1, "yellow": 2}, "C": {"red": 2, "blue": 2, "yellow": 1}}

robot = DeltaRobot(design, scene=sc)
robot.set_speed(0.4, 4.0)
robot.home()
packed = {o: {} for o in orders}
short = []
for oid, need in orders.items():
    box = next(b for b in robot.bins() if b["id"] == oid)
    slot = 0
    for item, qty in need.items():
        for _ in range(qty):
            avail = [p for p in robot.parts() if p["color"] == item and p["on"] == "table"]
            if not avail:
                short.append((oid, item))
                continue
            part = min(avail, key=lambda p: abs(p["x"] - box["x"]))  # 상자에 가까운 것부터
            robot.arch_to(part["x"], part["y"], robot.pick_z(part), height=0.025)
            robot.tool_on()
            # 상자 안에서 칸을 나눠 겹치지 않게 (2×3 배열)
            ox, oy = ((slot % 2) - 0.5) * 0.4 * box["w"], ((slot // 2) - 1) * 0.3 * box["d"]
            robot.arch_to(box["x"] + ox, box["y"] + oy, sc["surface_z"] + box["h"] + h + 0.01, height=0.025)
            robot.tool_off()
            slot += 1
            packed[oid][item] = packed[oid].get(item, 0) + 1
robot.home()

for oid, need in orders.items():
    ok = packed[oid] == need
    print("주문 %s: 필요 %s / 담음 %s  %s" % (oid, need, packed[oid], "✔ 출하 가능" if ok else "✖ 결품"))
if short:
    print("결품 목록:", ", ".join("%s-%s" % s for s in short), "→ 재고 보충 요청")
print("작업 시간 %.1f s, 상자별 수량 %s" % (robot.time, robot.scene.score()["per_bin"]))
`,
  },
  {
    id: 'glue', group: '현장 응용', title: '접착제 도포: 일정 속도 경로와 비드 품질',
    code: `# [현장] 접착제·실러 도포(디스펜싱) — 비드 두께가 고르려면 '일정한 속도'가 핵심
#  · 토출량이 일정할 때 비드 단면적 = 토출량 / 속도. 속도가 흔들리면 비드가 굵어지거나 끊깁니다.
#  · 방법 A: 변마다 move_to (모서리에서 정지)  vs  방법 B: 모서리를 둥글린 경로를 move_path로 연속 주행
import math
from deltarobot import DeltaRobot, DeltaDesign

design = DeltaDesign.preset("edu_dynamixel").copy(tool="pen")     # 펜 = 디스펜서 노즐
W, H, RC = 0.05, 0.06, 0.010          # 도포할 사각 테두리 50×60 mm, 모서리 반지름 10 mm
V = 0.04                              # 목표 도포 속도 40 mm/s
Q = 40.0                              # 토출량 40 mm³/s → 목표 비드 단면적 1.0 mm²


def rounded_rect(w, h, r, z, n=8):
    pts = []
    corners = [(w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180), (w / 2 - r, -h / 2 + r, 270)]
    for cx, cy, a0 in corners:
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
    pts.append(pts[0])
    return pts


def bead_report(robot, t_from, label):
    """도포 중(툴 ON) 프레임에서 속도를 구해 비드 단면적 분포를 봅니다."""
    fr = [f for f in robot.backend.frames if f["t"] >= t_from]
    speeds = []
    for a, b in zip(fr, fr[1:]):
        if a["tool"] and b["tool"] and b["t"] > a["t"]:
            pa, pb = robot.fk(*a["q"]), robot.fk(*b["q"])
            speeds.append(math.dist(pa, pb) / (b["t"] - a["t"]))
    dt = [b["t"] - a["t"] for a, b in zip(fr, fr[1:]) if a["tool"] and b["tool"]]
    good = sum(d for s, d in zip(speeds, dt) if abs(s - V) <= 0.1 * V) / sum(dt)
    slow = sum(d for s, d in zip(speeds, dt) if s < 0.5 * V)
    blobs = sum(1 for s0, s1 in zip(speeds, speeds[1:]) if s0 >= 0.5 * V > s1)  # 저속으로 떨어진 횟수
    print("%s: 도포 %.2f s | 목표 속도 ±10%% 구간 %.0f%% | 저속(<50%%) %.2f s, 뭉침 %d곳 (저속 비드 %.1f mm² 이상)" % (
        label, sum(dt), good * 100, slow, blobs + 1, 2 * Q / (V * 1000)))


robot = DeltaRobot(design, scene="drawing")
z = robot.scene.surface_z + 0.001
dx = 0.035                                  # 두 도형을 좌우로 나란히 (종이 위)

# 방법 A: 모서리에서 멈추는 직선 4개
sq = [(-dx + x, y, z) for x, y in [(W / 2, H / 2), (-W / 2, H / 2), (-W / 2, -H / 2), (W / 2, -H / 2), (W / 2, H / 2)]]
robot.arch_to(*sq[0], height=0.02)
t = robot.time
robot.pen_down()
for p in sq[1:]:
    robot.move_to(*p, speed=V, accel=0.4)
robot.pen_up()
bead_report(robot, t, "A 직선+정지")

# 방법 B: 둥근 모서리 경로를 한 번의 속도 프로파일로
path = [(dx + x, y, z) for x, y, _ in rounded_rect(W, H, RC, z)]
robot.arch_to(*path[0], height=0.02)
t = robot.time
robot.pen_down()
robot.move_path(path[1:], speed=V, accel=0.4)
robot.pen_up()
robot.move_by(dz=0.03)
bead_report(robot, t, "B 둥근 모서리 연속")
length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
print("※ 시작·끝의 가감속 구간은 어느 방법이든 남습니다 → 현장에서는 토출량을 속도에 비례시킵니다(속도 연동 토출).")
print("B 경로 길이 %.0f mm, 필요한 접착제 %.0f mm³" % (length * 1000, length * 1000 * Q / (V * 1000)))
`,
  },
  {
    id: 'safe', group: '현장 응용', title: '안전한 프로그램: 도달 검사·miss 재시도·작업 로그',
    code: `# [현장] 멈추지 않는 프로그램 — 도달 불가 목표, 집기 실패(miss), 재시도, 작업 로그
#  · 현장 로봇은 예외 하나로 라인 전체가 서면 안 됩니다. 미리 검사하고, 안 되면 건너뛰고, 기록을 남깁니다.
#  · 여기서는 부품 절반이 공급 중 14~17 mm 밀려 있다고 가정해 '집기 miss'를 만들고, 주변을 탐색해 다시 집습니다.
import math, random
from deltarobot import DeltaRobot, DeltaDesign, WorkspaceError

robot = DeltaRobot(DeltaDesign.preset("edu_dynamixel").copy(tool="suction_20"), scene="pick_place")
rc = robot.scene.data["work_radius"]
log = []
def note(msg):
    log.append("[%6.2f s] %s" % (robot.time, msg))
    print(log[-1])

random.seed(2)
robot.home()
jobs = [(p, random.uniform(0.014, 0.017) * (1 if k % 2 else 0)) for k, p in enumerate(robot.parts())]
jobs.append(({"id": "far", "x": 1.5 * rc, "y": 0.0, "z": robot.scene.surface_z, "h": 0.01,
              "top": robot.scene.surface_z + 0.01, "color": "red"}, 0.0))     # 작업영역 밖 주문

SEARCH = [(0, 0)] + [(k * dx, k * dy) for k in (0.008, 0.016) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
for part, err in jobs:
    ang = random.uniform(0, 2 * math.pi)
    x, y = part["x"] - err * math.cos(ang), part["y"] - err * math.sin(ang)   # 로봇이 알고 있는 (어긋난) 좌표
    z = robot.pick_z(part)
    if not robot.reachable(x, y, z):
        note("%s: 작업영역 밖 (%.0f, %.0f) mm → 건너뛰고 상위 시스템에 보고" % (part["id"], x * 1000, y * 1000))
        continue
    got = None
    for tries, (ox, oy) in enumerate(SEARCH, 1):         # 제자리 → 8 mm 십자 → 16 mm 십자
        if not robot.reachable(x + ox, y + oy, z):
            continue
        robot.arch_to(x + ox, y + oy, z, height=0.02)
        got = robot.tool_on()                            # None 이면 진공이 안 잡힘 (miss)
        if got:
            break
        robot.tool_off()
        robot.move_by(dz=0.01)
    if not got:
        note("%s: %d번 시도해도 못 집음 → 작업자 호출" % (part["id"], tries))
        continue
    if tries > 1:
        note("%s: %d번째 시도에 집음 (보정 %+.0f, %+.0f mm)" % (got, tries, ox * 1000, oy * 1000))
    box = next(b for b in robot.bins() if b["color"] == part["color"])
    try:
        robot.arch_to(box["x"], box["y"], robot.scene.surface_z + box["h"] + part["h"] + 0.02, height=0.03)
    except WorkspaceError as e:
        note("%s: 상자로 갈 수 없음 (%s) → 제자리에 내려놓음" % (got, e))
    robot.tool_off()
robot.home()
note("완료: %s" % robot.scene.score())
print("로그 %d줄 — 실제 현장에서는 파일/DB로 남겨 불량 원인 분석에 씁니다." % len(log))
`,
  },
  {
    id: 'cycle', group: '설계·분석', title: '표준 사이클(25-305-25) 벤치마크 · PPM',
    code: `# [설계] 업계 표준 사이클 벤치마크 — '25-305-25 mm' 아치 왕복으로 분당 픽 수(PPM) 비교
#  · 델타로봇 카탈로그의 사이클 타임은 보통 25 mm 올라가 305 mm 옮기고 25 mm 내려오는 왕복으로 잽니다.
#  · 교육용 로봇은 작업영역이 작으니 수평 거리를 작업 지름의 60%로 줄여 같은 방식으로 비교합니다.
from deltarobot import DeltaRobot, DeltaDesign, kinematics as kin

print("%-18s %8s %8s %8s %7s %7s  %s" % ("프리셋", "수평mm", "속도m/s", "사이클s", "PPM", "피크%", "판정"))
for name in ["edu_servo", "printer_stepper", "edu_dynamixel", "industrial_picker"]:
    d = DeltaDesign.preset(name)
    cyl = kin.work_cylinder(d, 0.1)
    W = min(0.305, 0.6 * cyl["diameter"])          # 수평 이동 거리
    robot = DeltaRobot(d)
    robot.home()
    z = robot.position[2] - 0.03
    robot.move_to(-W / 2, 0, z)
    t0 = robot.time
    for _ in range(3):                             # 3번 왕복 = 6 픽
        robot.arch_to(W / 2, 0, z, height=0.025)
        robot.wait(0.05)                           # 툴 ON/OFF 시간
        robot.arch_to(-W / 2, 0, z, height=0.025)
        robot.wait(0.05)
    cyc = (robot.time - t0) / 3
    r = robot.analyze()
    print("%-18s %8.0f %8.2f %8.2f %7.0f %7.0f  %s" % (
        name, W * 1000, robot.speed, cyc, 2 * 60 / cyc, r["peak_ratio"] * 100, "통과" if r["ok"] else "✖ 토크/속도 초과"))
print("※ 마지막 로봇(industrial_picker)의 동작이 시뮬레이터에 재생됩니다.")
`,
  },
  {
    id: 'motor', group: '설계·분석', title: '모터·감속기 선정: 후보 비교 합격 판정',
    code: `# [설계] 모터·감속기 선정 — 같은 기구, 같은 작업에 후보 조합을 돌려보고 합격하는 것 고르기
#  · 요구: 페이로드 300 g을 120 mm 아치로 0.6 m/s, 6 m/s² 로 왕복 (분당 약 60 사이클)
#  · 피크 토크·RMS 토크·최고 속도·속도-토크 곡선 비율이 모두 100% 이하(안전율 1.2 포함)여야 합격입니다.
#    (곡선% = 빠르게 돌 때 모터가 낼 수 있는 토크 대비 필요 토크)
from deltarobot import DeltaRobot, DeltaDesign, catalog

cat = catalog()
base = DeltaDesign.preset("edu_dynamixel").copy(payload=0.3)
candidates = [("rc_servo_mg996r", "none", 1), ("dxl_xm430_w350", "none", 1), ("dxl_xh540_w270", "none", 1),
              ("stepper_nema17", "belt", 4), ("stepper_nema23", "belt", 3), ("bldc_qdd", "planetary", 6),
              ("ac_servo_200w", "planetary", 10)]


def run_task(design):
    robot = DeltaRobot(design)
    robot.set_speed(0.6, 6.0)
    robot.home()
    z = robot.position[2] - 0.03
    robot.move_to(-0.06, 0, z)
    for _ in range(2):
        robot.arch_to(0.06, 0, z, height=0.025)
        robot.arch_to(-0.06, 0, z, height=0.025)
    return robot, robot.analyze()


print("%-26s %-14s %6s %6s %6s %6s %7s  %s" % ("모터", "감속", "피크%", "RMS%", "속도%", "곡선%", "관성비", "판정"))
winner = None
for motor, gear, ratio in candidates:
    d = base.copy(motor=motor, gearbox=gear, gear_ratio=ratio)
    robot, r = run_task(d)
    verdict = "합격" if r["ok"] else "불합격"
    print("%-26s %-14s %6.0f %6.0f %6.0f %6.0f %7.1f  %s" % (cat["motors"][motor]["name"][:26], "%s %d:1" % (gear, ratio),
          r["peak_ratio"] * 100, r["rms_ratio"] * 100, r["speed_ratio"] * 100, r["curve_ratio"] * 100,
          r["inertia_ratio"], verdict))
    if r["ok"] and winner is None:
        winner = (motor, gear, ratio)

print("선정 (작은 것부터 나열한 목록에서 첫 합격 조합):", winner)
if winner:
    run_task(base.copy(motor=winner[0], gearbox=winner[1], gear_ratio=winner[2]))   # 시뮬레이터 재생용
print("※ 카탈로그 값은 대표값입니다. 실제 선정은 제조사 데이터시트로 확인하세요.")
`,
  },
  {
    id: 'payload', group: '설계·분석', title: '페이로드별 최고 속도 (이분 탐색)',
    code: `# [설계] 페이로드별 최고 속도 찾기 — 이분 탐색으로 '모터가 버티는 한계 속도' 계산
#  · 무거운 물건일수록 가속을 줄여야 합니다. 얼마나 줄여야 하는지 숫자로 확인합니다.
#  · 속도 v, 가속 a = 10·v 로 묶어 한 변수(v)만 찾습니다.
import contextlib, io
from deltarobot import DeltaRobot, DeltaDesign


def passes(design, v):
    with contextlib.redirect_stdout(io.StringIO()):   # 시험 주행 중 속도 경고는 숨김
        return trial(design, v)


def trial(design, v):
    robot = DeltaRobot(design)
    robot.set_speed(v, 10 * v)
    robot.home()
    z = robot.position[2] - 0.03
    robot.move_to(-0.06, 0, z)
    robot.arch_to(0.06, 0, z, height=0.025)
    robot.arch_to(-0.06, 0, z, height=0.025)
    return robot.analyze()["ok"], robot.time


base = DeltaDesign.preset("edu_dynamixel")
print("%10s %12s %12s %10s" % ("페이로드", "최고속도", "왕복시간", "PPM"))
for m in [0.05, 0.5, 1.5]:
    d = base.copy(payload=m)
    lo, hi = 0.02, 1.5
    if not passes(d, lo)[0]:
        print("%8.0f g %12s" % (m * 1000, "정지 유지도 불가 (중력 토크 초과)"))
        continue
    for _ in range(6):                       # 6번이면 (1.5-0.02)/2^6 ≈ 23 mm/s 정밀도
        mid = (lo + hi) / 2
        if passes(d, mid)[0]:
            lo = mid
        else:
            hi = mid
    ok, t = passes(d, lo)
    print("%8.0f g %9.2f m/s %10.2f s %10.0f" % (m * 1000, lo, t, 2 * 60 / t))
print("※ 마지막으로 계산한 조건의 동작이 시뮬레이터에 재생됩니다.")
`,
  },
  {
    id: 'profiles', group: '설계·분석', title: '속도 프로파일 비교 (토크·시간)',
    code: `from deltarobot import DeltaRobot, DeltaDesign
from deltarobot.trajectory import PROFILES

design = DeltaDesign.preset("edu_dynamixel")
print("%-10s %8s %8s %8s" % ("profile", "time[s]", "peak%", "rms%"))
for prof in PROFILES:
    robot = DeltaRobot(design)
    robot.set_profile(prof)
    robot.home()
    x, y, z = robot.position
    t0 = robot.time
    for _ in range(3):
        robot.arch_to(0.08, 0, z - 0.03, height=0.025)
        robot.arch_to(-0.08, 0, z - 0.03, height=0.025)
    r = robot.analyze()
    print("%-10s %8.2f %8.0f %8.0f" % (prof, robot.time - t0, r["peak_ratio"] * 100, r["rms_ratio"] * 100))
`,
  },
  {
    id: 'sweep', group: '설계·분석', title: '설계 파라미터 스윕 (아래팔 길이)',
    code: `from deltarobot import DeltaDesign, kinematics as kin, dynamics as dyn

base = DeltaDesign.preset("edu_dynamixel")
print("%8s %10s %12s %10s" % ("l [mm]", "D [mm]", "정적토크", "κ(홈)"))
for l in [0.20, 0.25, 0.30, 0.35, 0.40]:
    d = base.copy(forearm=l)
    cyl = kin.work_cylinder(d, 0.10)
    h = d.home_theta
    p = kin.fk(d, (h, h, h))
    tau = dyn.static_torques(d, p)[0]
    print("%8.0f %10.0f %10.3f Nm %10.2f" % (l * 1000, cyl["diameter"] * 1000, tau, kin.condition_number(d, (h, h, h))))
`,
  },
];

export const DEFAULT_CODE = EXAMPLES[0].code;
