// playground-examples.js — example programs for the Python Playground.
// Every example is executed by tests/test_playground_examples.py with the record backend.

export const EXAMPLES = [
  {
    id: 'first', title: '① 첫 동작: 사각형 그리며 움직이기',
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
    id: 'ikfk', title: '② 역기구학·순기구학 확인',
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
    id: 'sort', title: '③ 색깔 분류 pick & place',
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
    id: 'conveyor', title: '④ 컨베이어 트래킹 (벨트 속도 맞춰 집기)',
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
    id: 'draw', title: '⑤ 펜으로 원·별 그리기 (move_path)',
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
    id: 'profiles', title: '⑥ 속도 프로파일 비교 (토크·시간)',
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
    id: 'workspace', title: '⑦ 작업영역·작업 실린더 계산',
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
    id: 'sweep', title: '⑧ 설계 파라미터 스윕 (아래팔 길이)',
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
