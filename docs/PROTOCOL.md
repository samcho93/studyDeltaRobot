# 통신 프로토콜

studyDeltaRobot 의 웹 시뮬레이터·PC Python·ROS 2·아두이노가 주고받는 메시지를 정리합니다.
코드가 바뀌면 이 문서도 같이 고칩니다.

## 1. 웹 시뮬레이터 ↔ PC (WebSocket)

같은 프로토콜을 두 서버가 구현합니다. 시뮬레이터 입장에서는 둘이 구별되지 않습니다.

| 서버 | 구현 | 실행 |
|---|---|---|
| websim 백엔드 (기준 구현) | `python/deltarobot/backends/websim.py` | `DeltaRobot(backend="websim")` |
| ROS 2 web bridge | `ros2_ws/src/delta_robot/delta_robot/web_bridge_node.py` | `ros2 launch delta_robot display.launch.py web_bridge:=true` |

### 연결

- 주소: **`ws://127.0.0.1:8765`** (서버는 루프백에만 바인드. websim `host`/`port` 옵션, bridge `host`/`port` 파라미터로 변경 가능)
- 방향: PC 가 **서버**, 브라우저(시뮬레이터 `연결` 탭의 **[PC Python 연결]**)가 **클라이언트**.
- 메시지: 텍스트 프레임 하나에 JSON 객체 하나. 모든 객체에 문자열 `"type"` 필드가 있습니다.
  알 수 없는 `type`·JSON 이 아닌 프레임은 **무시**합니다(연결을 끊지 않음).
- Origin 검사(`deltarobot.backends.websim.origin_allowed`, bridge 도 같은 함수를 import):
  - 허용: `https://samcho93.github.io`, `http://localhost`, `http://127.0.0.1`, `https://localhost`, `https://127.0.0.1`
    (각각 뒤에 `:포트` 또는 `/경로` 가 붙어도 허용)
  - `Origin` 헤더가 없는 클라이언트(테스트 스크립트, 비브라우저 도구)는 허용
  - `Origin: null`(file:// 로 연 페이지, 샌드박스 iframe)과 그 밖의 사이트는 HTTP **403** 으로 거절

### 순서

```
브라우저                                  PC (websim / web bridge)
   | --- 연결 (Origin 검사) --------------> |
   | --- {"type":"hello", ...} -----------> |
   | <-- {"type":"design", ...} ----------- |
   | <-- {"type":"state", ...}  (반복) ----- |
   | --- {"type":"estop"}  (언제든) -------> |
   | --- {"type":"goal", ...} (bridge 전용) > |
```

### 브라우저 → PC

#### `hello`
```json
{"type": "hello", "design": {...DeltaDesign dict...}, "scene": {...scene dict...}}
```
| 필드 | 타입 | 설명 |
|---|---|---|
| `design` | object, 선택 | 시뮬레이터가 지금 보여 주는 설계 (`DeltaDesign.to_dict()` 형식) |
| `scene` | object, 선택 | 시뮬레이터의 현재 장면 (`deltarobot.scene` 형식) |

- websim: `DeltaRobot(design=None)` 이면 이 설계를, `scene=None` 이면 이 장면을 씁니다. hello 가 `wait_connect`(기본 60 s) 안에 오지 않으면 `TimeoutError`.
- web bridge: 시뮬레이터 설계는 **무시**하고 ROS 쪽 설계(`preset`/`design_file` 파라미터)로 `design` 을 답합니다.

#### `estop`
```json
{"type": "estop"}
```
- websim: 다음 명령에서 `EmergencyStop` 예외 → 학생 프로그램이 멈춥니다.
- web bridge: `/delta/estop` 에 `std_msgs/Bool(True)` 발행 → 드라이버가 정지·명령 무시, 커맨더가 대기열 삭제.
  해제는 ROS 쪽에서: `ros2 topic pub --once /delta/estop std_msgs/msg/Bool "{data: false}"`.

#### `goal` (web bridge 전용 확장)
```json
{"type": "goal", "xyz": [0.03, 0.0, -0.25]}
```
| 필드 | 타입 | 설명 |
|---|---|---|
| `xyz` | [number, number, number] | **TCP(툴 끝)** 목표 위치 [m], 베이스 좌표 (z 위쪽, 작업은 z<0) |

- bridge 가 `/delta/goal` (`geometry_msgs/Point`) 로 발행 → `delta_commander` 가 직선 궤적을 계획합니다.
  작업영역·관절 한계·속도 검사는 커맨더/드라이버가 하며, 거부되면 ROS 로그에만 남습니다(응답 메시지 없음).
- 형식이 틀린 goal(길이≠3, 숫자 아님, NaN/Inf)은 무시합니다. websim 백엔드는 이 메시지를 무시합니다.

### PC → 브라우저

#### `design`
```json
{"type": "design", "design": {...DeltaDesign dict...}, "scene": {...scene dict...}}
```
- hello 직후 한 번(websim 은 `DeltaRobot` 생성 시). 시뮬레이터는 이 설계·장면으로 화면을 다시 만듭니다.
- web bridge 의 `scene` 은 `default_scene(design, <scene 파라미터, 기본 "pick_place">)`.

#### `state`
```json
{"type": "state", "t": 12.34, "q": [0.35, 0.35, 0.35], "tool": 0}
```
| 필드 | 타입 | 설명 |
|---|---|---|
| `t` | number | 서버 시작(websim: 로봇 시작) 이후 시간 [s], 소수 4자리 |
| `q` | [θ1, θ2, θ3] | 모터각 [rad], 0 = 위팔 수평, + = 아래, 소수 6자리 |
| `tool` | 0 \| 1 | 툴 꺼짐/켜짐 |

- websim: 궤적 샘플마다(10 ms) 전송. web bridge: `/joint_states` 의 `motor1_joint..motor3_joint` 와 `/delta/tool_state` 를 **최대 50 Hz** 로 전송(hello 한 클라이언트에게만).
- 시뮬레이터는 `q` 로 순기구학을 풀어 이펙터를 그리고, `tool` 이 0→1 로 바뀌는 순간 부품 잡기를 판정합니다.

## 2. PC ↔ 아두이노 (시리얼)

115200 baud, ASCII, 한 줄에 명령 하나(`\n`, `\r` 무시). 구현: `firmware/delta_servo/delta_servo.ino`,
호스트: `python/deltarobot/backends/serial_backend.py`, ROS 2 `delta_driver` (`hardware:=serial`).

| 명령 | 의미 | 응답 |
|---|---|---|
| `J d1 d2 d3` | 모터 목표각 [deg] (0 = 수평, + = 아래). 펌웨어가 −40°~85° 로 자르고 슬루율 제한 | 없음 |
| `T 0\|1` | 툴 끔/켬 | 없음 |
| `E` | 비상정지: 서보 detach, 툴 끔, `J`/`T` 무시 | `ESTOP` |
| `R` | 비상정지 해제: 마지막 각도로 서보 re-attach | `READY` |
| `?` | 상태 | `OK d1 d2 d3 tool` |

- 호스트는 `J` 를 **최대 50 Hz** 로 보냅니다(serial 백엔드 `rate=50`, 드라이버도 50 Hz 제한).
- 오류 응답: `ERR <이유>`. 부팅 메시지: `READY delta_servo`.

## 3. ROS 2 토픽·서비스 (`delta_robot` 패키지)

| 이름 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/delta/joint_command` | `sensor_msgs/JointState` | → driver | `motor1_joint..motor3_joint` [rad] |
| `/delta/tool_command` | `std_msgs/Bool` | → driver | 툴 |
| `/delta/estop` | `std_msgs/Bool` | → driver, commander | True = 정지·명령 무시, False = 해제 |
| `/joint_states` | `sensor_msgs/JointState` | driver → | 능동 + 수동(elbow*a_pitch/yaw) + effector_x/y/z |
| `/delta/tool_state` | `std_msgs/Bool` | driver → | 툴 상태 |
| `/delta/tcp` | `geometry_msgs/PointStamped` | driver → | TCP 위치, frame `base_link` |
| `/delta/goal` | `geometry_msgs/Point` | → commander | TCP 목표 [m] |
| `/delta/busy` | `std_msgs/Bool` | commander → | 궤적 스트리밍 중 |
| `/delta/tool` | `std_srvs/SetBool` (서비스) | → commander | 툴 on/off (진행 중인 동작 뒤에 실행) |
| `/robot_description` | `std_msgs/String` | robot_state_publisher → | 설계로 생성한 URDF |
