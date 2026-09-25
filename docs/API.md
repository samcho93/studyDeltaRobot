# deltarobot API 및 도구 화면 참조

강의 본문·예제가 따라야 하는 **정확한 이름**입니다. 코드가 바뀌면 이 문서도 같이 고칩니다.

## 1. 설치·가져오기

```bash
pip install -e python            # 저장소 루트에서 (의존성 없음, 표준 라이브러리만)
pip install -e "python[websim]"  # websim 백엔드 (websockets>=12)
pip install -e "python[serial]"  # serial 백엔드 (pyserial)
```
Playground(Pyodide)에서는 설치 없이 `import deltarobot`이 됩니다.

## 2. 좌표·단위 규약

- 단위: m, rad, kg, s. 베이스 중심이 원점, **z 위쪽**, 작업은 z<0.
- 팔 1·2·3의 방위각 φ = 0°, 120°, 240° (팔 1이 +x).
- 모터각 θ: 위팔이 수평이면 0, **아래로 내려가면 +**.
- `DeltaRobot`의 위치 인자는 모두 **TCP(툴 끝)** 좌표. `deltarobot.kinematics`의 함수는 **이펙터 중심** 좌표.
  TCP = 이펙터 중심 − (0, 0, tool.length).

## 3. DeltaDesign (`deltarobot.design`)

| 필드 | 의미 | 기본값 |
|---|---|---|
| `base_radius` (R) | 베이스 중심 → 모터축 | 0.1 |
| `effector_radius` (r) | 이펙터 중심 → 볼조인트 쌍 중심 | 0.035 |
| `upper_arm` (L) | 위팔 길이 | 0.13 |
| `forearm` (l) | 아래팔(평행사변형 로드) 길이 | 0.3 |
| `forearm_spacing` (w) | 두 로드 간격 | 0.05 |
| `theta_min`, `theta_max` | 모터각 한계 [rad] | −0.7, 1.5 |
| `ball_joint_limit` | 로드가 팔 평면에서 벗어날 수 있는 최대각 [rad] | 0.6 |
| `upper_arm_material`, `forearm_material` | 카탈로그 재질 id | al_tube, cfrp_tube |
| `motor`, `gearbox`, `gear_ratio` | 카탈로그 모터·감속기 id, 감속비 | dxl_xm430_w350, none, 1 |
| `tool` | 카탈로그 툴 id | gripper_servo |
| `payload`, `effector_mass` | 페이로드, 이펙터 판 질량 [kg] | 0.2, 0.06 |

- `DeltaDesign.preset(name)` — 프리셋: `edu_servo`, `edu_dynamixel`, `printer_stepper`, `industrial_picker`
- `DeltaDesign.load("design.json")`, `.from_dict(d)`, `.to_dict()`, `.copy(**changes)`, `.validate()` → 문제 목록(list[str])
- 파생값(프로퍼티): `upper_arm_mass`, `elbow_mass`, `forearm_pair_mass`, `moving_plate_mass`, `arm_inertia`,
  `drive_inertia`, `gravity_moment`, `ratio`, `efficiency`, `tool_length`, `motor_spec`, `gearbox_spec`, `tool_spec`, `home_theta`
- `deltarobot.catalog()` — 카탈로그 dict: `motors`, `gearboxes`, `materials`, `tools`, `presets`

카탈로그 id
- 모터: `rc_servo_mg996r`, `dxl_xm430_w350`, `dxl_xh540_w270`, `stepper_nema17`, `stepper_nema23`, `bldc_qdd`, `ac_servo_200w`, `ac_servo_400w`, `ac_servo_750w`
  (RC 서보·Dynamixel은 `integrated_gear: true` → 외부 감속기 무시)
- 감속기: `none`[1], `belt`[2,3,4,5], `planetary`[3,5,6,9,10,15,20,30,50], `cycloidal`[11,29,59], `harmonic`[30,50,80,100]
- 재질: `cfrp_tube`, `al_tube`, `pla_print`, `al_profile`
- 툴: `suction_20`, `suction_40`, `gripper_servo`, `gripper_pneumatic`, `electromagnet`(철 부품만), `pen`

## 4. 기구학 (`deltarobot.kinematics`, 이펙터 중심 좌표)

`ik(d, p)` → (θ1,θ2,θ3) · `fk(d, theta)` → (x,y,z) · `Unreachable` 예외 ·
`jacobian(d, theta)` (v = J θ̇) · `joint_velocity(d, theta, v)` · `condition_number(d, theta)` ·
`passive_angles(d, theta)` · `elbow_angle(d, i, theta_i, p)` · `limit_report(d, p)` → `{"ok","theta","problems","ball","elbow"}` ·
`reachable(d, p)` · `workspace_points(d, step)` · `work_cylinder(d, height)` → `{"diameter","height","z_top","z_bottom","profile"}` ·
상수 `PHI`, `ELBOW_MIN`(20°), `ELBOW_MAX`(165°)

## 5. 동역학 (`deltarobot.dynamics`)

`joint_torques(d, theta, theta_dd, p=None, a=(0,0,0))` — 출력축 토크 [N·m], 부호 + = 팔을 아래로 ·
`static_torques(d, p)` · `available_torque(d, omega_motor)` (속도-토크 곡선) ·
`analyze(d, frames)` — frames `[{t,q,tool}]` → 속도·가속도·토크 추가 ·
`evaluate(d, samples, safety_factor=1.2)` → `peak_ratio`, `rms_ratio`, `speed_ratio`, `curve_ratio`, `inertia_ratio`, `ok`, `problems` ·
`inertia_ratio(d)`

## 6. 궤적 (`deltarobot.trajectory`)

`PROFILES = ("trapezoid","scurve","quintic","cycloidal")` · `Profile(kind, d, vmax, amax)` → `.T`, `.at(t)`→(s,ṡ,s̈) ·
`line_path(p0,p1)` · `arch_path(p0,p1,height,radius=0)` · `sample_path(path, kind, vmax, amax, dt)` · `sample_joint(q0,q1,kind,wmax,alpha,dt)`

## 7. DeltaRobot (`from deltarobot import DeltaRobot`)

```python
robot = DeltaRobot(design=None, backend="auto", scene=None, **backend_options)
#   design : 프리셋 이름 | DeltaDesign | dict | "파일.json" | None(연결된 시뮬의 설계 또는 기본값)
#   backend: "auto"(=record) | "record" | "websim" | "serial" | "ros2"
#   scene  : None(시뮬의 장면 또는 pick_place) | "pick_place" | "conveyor" | "drawing" | "empty" | dict
```

| 메서드/속성 | 설명 |
|---|---|
| `home()` | 세 모터 모두 `home_theta`(0.35 rad)로 관절 보간 이동 |
| `move_to(x, y, z, speed=None, accel=None, profile=None)` | TCP 직선 이동 |
| `move_by(dx=0, dy=0, dz=0)` | 상대 직선 이동 |
| `move_path(points, speed=None, accel=None, profile=None)` | 여러 TCP 점을 잇는 꺾은선을 **하나의 속도 프로파일**로 (모서리에서 멈추지 않음 — 그리기·도포용) |
| `arch_to(x, y, z, height=0.03, radius=0)` | pick & place 아치(문) 궤적 |
| `track_pick(part, hover=0.02, descent_time=0.3, dwell=0.1, height=0.03, z=None)` | **컨베이어 트래킹 pick**: 부품 앞쪽 상공으로 아치 이동 → 벨트 속도로 가속하며 하강(접촉 순간 위치·속도 일치) → `tool_on` → `dwell`초 동행 → 감속하며 상승. 잡은 id 반환 |
| `pick_z(part)` | 부품을 잡을 TCP 높이 (흡착·전자석 = 윗면, 그리퍼 = 높이 중간) |
| `move_joints(t1, t2, t3, degrees=False)` | 관절 공간 이동 |
| `move_joint_to(x, y, z)` | 목표는 TCP, 경로는 관절 보간 |
| `tool_on()` / `tool_off()` | 흡착/그리퍼/전자석/펜. 별칭 `suction_on/off`, `grip/release`, `magnet_on/off`, `pen_down/up`. 잡은 부품 id 반환 |
| `wait(seconds)` | 대기 (Playground에서는 가상 시간) |
| `set_speed(speed=None, accel=None)` | m/s, m/s² |
| `set_profile(name)` | 속도 프로파일 |
| `position` / `joints` / `effector` / `time` / `holding` | TCP 위치, 모터각, 이펙터 중심, 경과 시간, 잡고 있는 부품 id |
| `ik(x,y,z)` / `fk(t1,t2,t3)` / `reachable(x,y,z)` | TCP 기준 기구학 |
| `parts()` / `bins()` | 현재 시각의 부품 목록(dict: id,x,y,z,top,size,h,color,material,on) / 상자 목록(id,x,y,w,d,h,color) |
| `scene.bin("A")`, `scene.score()`, `scene.conveyor()` | 상자 찾기, 분류 결과 `{"per_bin","correct","total"}`, 컨베이어 정보(y,width,x_start,x_end,speed) |
| `analyze(safety_factor=1.2)` | 기록된 동작 전체의 모터 검증 (record 백엔드) |
| `close()` | 연결 종료 (with 문 지원) |

예외: `WorkspaceError`(작업영역·관절 한계·속도 초과), `EmergencyStop`(시뮬/드라이버의 비상정지), `ValueError`(설계 오류).
기본 속도 `speed = 0.25·(L+l)/0.43` m/s, `accel = 10·speed`, 프로파일 `scurve`.

## 8. 백엔드

| 이름 | 용도 | 옵션 |
|---|---|---|
| `record` | 가상 시간 기록(Playground, PC 드라이런) | `time_limit=600`, `verbose=False` |
| `websim` | PC 파이썬 → 웹 시뮬레이터 실시간 (ws://127.0.0.1:8765) | `host`, `port=8765`, `wait_connect=60` |
| `serial` | 아두이노 펌웨어 (`J d1 d2 d3` / `T 0|1` / `E` / `R` / `?`) | `port="COM3"`, `baud=115200`, `rate=50`, `max_joint_speed=3.49` (펌웨어 200°/s) |
| `ros2` | `/delta/joint_command`(JointState), `/delta/tool_command`(Bool), `/delta/estop`(Bool), 시작 시 `/delta/session`(String JSON: design·scene, latched) → web_bridge 시계 동기 | `node_name` |

`serial`·`ros2`는 관절 속도가 모터 한계를 넘으면 `WorkspaceError`로 **거부**, `record`·`websim`은 경고만 출력.

- `serial` 백엔드: `robot.backend.estop()` → `E`(서보 힘 풀림·툴 끔, 이후 명령 무시), `robot.backend.reset()` → `R`(마지막 각도로 서보 재연결, 백엔드의 비상정지 상태도 해제).
- `ros2` 백엔드는 `ros2_ws/src/delta_robot` 의 `delta_driver` 가 떠 있어야 합니다 (`ros2 launch delta_robot display.launch.py`).
- 메시지 형식 전체(WebSocket·시리얼·ROS 2 토픽): `docs/PROTOCOL.md`.

## 9. CLI

```bash
python -m deltarobot.urdf --preset edu_dynamixel -o delta.urdf
python -m deltarobot.urdf --design my_design.json -o delta.urdf
```

`deltarobot.urdf.joint_state(design, theta, tool=0)` — URDF의 모든 독립 관절값(모터 3 + 수동 6 + 가상 이펙터 3,
그리퍼 툴이면 `gripper_finger_a`: 열림 0 / 닫힘 −`GRIPPER_STROKE`(0.006 m)). JS: `jointState(d, theta, tool)`.

## 10. 웹 도구 화면

### 강의 페이지 실습 패널 (`assets/js/site.js`)
- 강의 본문의 실습은 페이지를 떠나지 않고 **오른쪽 실습 패널**(iframe)에서 열립니다. 좁은 화면(≤ 860 px)에서는 아래쪽 시트로 열립니다.
  - ` ```python run ` 코드 블록의 **▶ 오른쪽에서 실행** → 패널의 Playground가 코드를 받아 바로 실행하고, 동작을 시뮬레이터에서 재생합니다. 패널에 Playground가 이미 떠 있으면 다시 불러오지 않고 `postMessage({type:'load-code', code, run:true})`로 코드만 바꿉니다(Pyodide 재로딩 없음).
  - 본문 안의 `sim/…`, `tools/…` 링크(`@btn[...]`, 머리의 시뮬레이터·Playground 버튼 포함) → 같은 패널에서 열림.
  - Ctrl/⌘/Shift/가운데 클릭은 평소처럼 새 탭. 패널 머리의 **↗ 새 탭**은 지금 화면을 크게 엽니다. 왼쪽 가장자리를 끌어 폭을 바꾸면 기억합니다.
- 패널 안의 도구는 `?dock=1`로 열립니다: `theme.js`가 `<html class="docked">`를 붙여 머리글을 숨기고, Playground는 시뮬레이터(위)·편집기·출력(아래) 세로 배치가 됩니다. 테마는 강의 페이지를 따라갑니다.
- 1600 px보다 좁은 화면에서 패널이 열리면 왼쪽 목차 사이드바는 접히고 ☰ 버튼으로 엽니다. 사이드바의 도구 링크는 그대로 도구 페이지로 이동합니다.

### 시뮬레이터 `sim/index.html`
- 탭 **설계**: 프리셋 선택, 치수 슬라이더(R, r, L, l, w), 모터각 한계, 재질, 모터, 감속기·감속비, 툴, 페이로드, 이펙터 질량,
  파생값 표(질량·관성·정적 토크), **[설계 검증]** 버튼(작업 실린더 D×H, 표준 사이클의 모터 검증 결과 ✔/✖)
- 탭 **조그**: TCP x/y/z 슬라이더, 모터 θ1~θ3 슬라이더(순기구학), 표시: 모터각·조건수·볼조인트각·팔꿈치각·정적 토크, 한계 초과 시 빨간색
  · **집기·옮기기**: [가까운 부품 집기](3D 화면에서 부품을 클릭하면 그 부품) → 부품 위로 이동·하강·툴 ON·상승을 부드럽게 수행,
  [상자 A/B/C로 옮기기], [여기에 놓기]. 키보드: ←→ x, ↑↓ y, Q/E·PgUp/PgDn z, G 집기/놓기. 잡은 부품은 조그로 함께 움직임.
  조그를 시작하면 재생 중이던 타임라인은 멈추고(▶로 다시 재생) 조그가 자세를 결정합니다.
  · **조이스틱**(3D 화면 오른쪽 아래, 시뮬레이터·URDF 뷰어 공통 `assets/js/joystick.js`): XY 패드 = 수평 이동, Z 레버 = 위/아래,
  속도 슬라이더(mm/s), 버튼 = 집기/놓기(URDF 뷰어는 툴 ON/OFF). 손을 떼면 멈추는 속도 제어이며 작업영역 경계에서는 가능한 축으로만
  미끄러집니다. USB/블루투스 게임패드도 인식: 왼쪽 스틱 x/y, 오른쪽 스틱 세로 z, A(×) 버튼.
- 탭 **작업**: 장면(빈 작업대 / 분류 / 컨베이어 / 그리기), 데모 프로그램(Adept 사이클 / 분류 데모 / 컨베이어 데모 / 원·별 그리기),
  궤적 설정(프로파일, 속도, 가속도, 아치 높이), 재생·일시정지·처음으로, 재생 배속
- 탭 **분석**: 그래프(모터각 θ, 모터 속도 ω, 모터축 토크 τ — 한계선 표시), 검증 결과(피크/RMS/속도/관성비), 사이클 시간·분당 사이클,
  작업영역 점구름 표시, 작업 실린더 표시
- 탭 **연결**: [PC Python 연결] (ws://127.0.0.1:8765, websim 백엔드·ROS 2 web_bridge 공용), 타임라인 JSON 불러오기,
  설계 JSON 내보내기/불러오기, URDF 내려받기, 공유 링크 복사
- 로봇은 기본적으로 **설계값으로 생성한 URDF**(`assets/js/delta/urdf.js`, Python `deltarobot.urdf`와 동일)를 urdf-loader로 불러와 그리고,
  매 프레임 `jointState()`로 모터·수동·가상 관절값만 넣습니다 — URDF 뷰어·ROS 2 RViz와 같은 모델. 왼쪽 아래 **URDF 모델**을 끄거나
  `?model=mesh`로 열면 three.js 도형으로 그린 기본 모델을 씁니다.
- 상단: 비상정지 버튼(연결 시 `{"type":"estop"}` 전송), 테마, 도구 링크
- URL: `?preset=edu_servo`, `?scene=conveyor`, `#design=<base64url JSON>`, `?embed=1`(Playground 내장용)
- 현재 설계는 `localStorage['studydelta.design.v1']`에 저장되어 URDF 뷰어·Playground가 같이 씁니다.

### Python Playground `tools/playground.html`
- Pyodide에서 `deltarobot`을 import. 실행하면 `record` 백엔드가 가상 시간으로 동작을 기록 → 오른쪽 내장 시뮬레이터가 재생.
- 예제 목록(기초 4 · 현장 응용 9 · 설계·분석 5, `tools/playground-examples.js`의 `group`으로 묶음), 출력 창, [실행](Ctrl+Enter), [정지], 결과 요약(총 시간, 모터 검증), `print` 출력.
- 예제를 고르면 내장 시뮬레이터가 **대기 상태**가 됩니다: 재생을 멈추고, 그 예제가 쓸 설계·작업 셀(부품·상자)을 홈 자세로 보여 준 뒤 ▶ 실행을 기다립니다. 실행 중이던 코드는 정지합니다. 시작 설정은 `tools/playground-setups.json`(`tools/make_example_setups.py`, `build.py`가 갱신)에 있고, 설계를 지정하지 않는 예제(`DeltaRobot()`)는 시뮬레이터 설계를 그대로 쓰고 씬 종류만 바꿉니다. embed 메시지 `{type:'standby', setup}`.
- 사용자 정의 씬: `default_scene(design, kind)`로 만든 dict의 `parts`·`bins`·`conveyor.spawn`을 바꿔 `DeltaRobot(scene=dict)`로 넘기면 시뮬레이터가 그대로 그립니다. 부품·상자 색: red, blue, green, yellow, orange, purple, brown, black, white, gray.
- 설계는 시뮬레이터에서 마지막으로 쓴 설계(없으면 코드의 프리셋). 코드에 `DeltaRobot("edu_servo")`처럼 쓰면 그 설계가 우선.
- `time.sleep()`은 가상 시간으로 바뀝니다(브라우저가 멈추지 않음). 가상 시간 600 s를 넘으면 중단.

### URDF 뷰어 `tools/urdf-viewer.html`
- 현재 설계(또는 프리셋)로 URDF를 생성해 three.js + urdf-loader로 표시. 모터 θ1~θ3 슬라이더 → 수동 관절 자동 계산.
- [수동 관절 계산 끄기] 토글: 닫힌 사슬이 풀려 보이는 이유 확인. 링크 트리, URDF 텍스트 보기, 내려받기.

### 기구학 실험실 `tools/kinematics-lab.html`
- 2D 캔버스: 위에서 본 베이스/이펙터 삼각형 + 선택한 팔의 측면(팔 평면) 그림.
- IK 모드: 목표점을 드래그 → 팔 평면에서 원(위팔) ∩ 원(아래팔 투영, 반지름 √(l²−y'²))의 교점, 두 해 표시, 계산 단계 숫자.
- FK 모드: θ1~θ3 슬라이더 → 세 구의 교점, 두 해(위/아래) 표시.
