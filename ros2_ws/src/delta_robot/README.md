# delta_robot — ROS 2 패키지

studyDeltaRobot 강의의 회전형 델타 로봇을 ROS 2 에서 돌리는 `ament_python` 패키지입니다.
**ROS 2 Humble (Ubuntu 22.04)** 과 **Jazzy (Ubuntu 24.04)** 를 대상으로 합니다.
기구학·URDF·궤적 계산은 모두 저장소의 `deltarobot` 라이브러리를 그대로 씁니다(복사하지 않음).

| 실행 파일 | 역할 |
|---|---|
| `delta_driver` | 모터 드라이버. `mock`(가상 모터) 또는 `serial`(아두이노 펌웨어). 관절 한계·속도 제한·비상정지를 **여기서** 강제 |
| `delta_commander` | `/delta/goal`(TCP 목표) → 직선 궤적 + IK → `/delta/joint_command` 100 Hz 스트리밍, `/delta/tool` 서비스 |
| `delta_web_bridge` | 웹 시뮬레이터의 **[PC Python 연결]** 버튼으로 ROS 로봇을 보여 주는 WebSocket 서버(ws://127.0.0.1:8765) |
| `delta_pick_place_demo` | `DeltaRobot(preset, backend="ros2")` 로 하는 pick & place 예제 |

## 1. 설치

```bash
# ROS 2 (Humble 또는 Jazzy) 가 설치되어 있다고 가정
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt install ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-rviz2 python3-serial

# 저장소 받기
git clone https://github.com/samcho93/studyDeltaRobot.git ~/studyDeltaRobot
cd ~/studyDeltaRobot

# deltarobot 라이브러리 설치 (노드들이 import 합니다)
pip install -e "python[websim,serial]"                          # Ubuntu 22.04 (Humble)
pip install -e "python[websim,serial]" --break-system-packages  # Ubuntu 24.04 (Jazzy), 시스템 파이썬
```

Ubuntu 24.04 에서 시스템 파이썬을 건드리고 싶지 않으면 ROS 패키지가 보이는 가상환경을 씁니다.

```bash
python3 -m venv --system-site-packages ~/delta_venv
source ~/delta_venv/bin/activate
pip install -e "python[websim,serial]"
# colcon 빌드·실행 전에 항상 이 venv 를 activate 합니다
```

확인: `python3 -c "import deltarobot, websockets; print(websockets.__version__)"` → **12 이상**이어야 합니다.
apt 의 `python3-websockets`(22.04: 9.x, 24.04: 10.x)는 너무 오래되어 web bridge 가 동작하지 않으므로 pip 버전을 씁니다.

## 2. 빌드

```bash
cd ~/studyDeltaRobot/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. 실행 — 가상 모터 + RViz + 웹 시뮬레이터

```bash
ros2 launch delta_robot display.launch.py preset:=edu_servo web_bridge:=true
```

- RViz 에 URDF 로봇이 뜹니다(고정 프레임 `base_link`). URDF 는 launch 시점에 설계로 생성됩니다(`deltarobot.urdf.generate`).
- 브라우저에서 시뮬레이터(`https://samcho93.github.io/studyDeltaRobot/sim/`) → `연결` 탭 → **[PC Python 연결]** 을 누르면
  시뮬레이터가 ROS 쪽 설계로 바뀌고 로봇 움직임을 따라 합니다. 시뮬레이터의 비상정지 버튼 → `/delta/estop`.

launch 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `preset` | `edu_dynamixel` | `edu_servo`, `edu_dynamixel`, `printer_stepper`, `industrial_picker` |
| `design_file` | `''` | 시뮬레이터에서 내보낸 설계 JSON (preset 보다 우선). 예: `config/design_example.json` |
| `use_rviz` | `true` (bringup: `false`) | RViz 실행 |
| `web_bridge` | `false` | WebSocket 브리지 실행 |
| `hardware` | `mock` (bringup: `serial`) | `mock` \| `serial` |
| `port` | `/dev/ttyACM0` | 아두이노 포트 |
| `scene` | `pick_place` | 시뮬레이터로 보낼 장면 |

## 4. 움직여 보기

다른 터미널에서(`source install/setup.bash` 후):

```bash
# TCP(툴 끝)를 직선으로 이동 [m]
ros2 topic pub --once /delta/goal geometry_msgs/msg/Point "{x: 0.03, y: 0.0, z: -0.25}"

# 툴 켜기 / 끄기 (진행 중인 이동이 끝난 뒤 실행)
ros2 service call /delta/tool std_srvs/srv/SetBool "{data: true}"
ros2 service call /delta/tool std_srvs/srv/SetBool "{data: false}"

# 상태 보기
ros2 topic echo /delta/tcp
ros2 topic echo /delta/busy

# 비상정지 / 해제
ros2 topic pub --once /delta/estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /delta/estop std_msgs/msg/Bool "{data: false}"
```

- 목표가 작업영역·관절 한계 밖이거나 경로 중간이 한계를 넘으면 커맨더가 **거부**하고 에러 로그를 남깁니다.
  관절 속도가 모터 한계를 넘는 경우도 거부합니다 → `speed` 파라미터를 낮추세요.
  (예: `ros2 param set /delta_commander speed 0.1`, 0 이면 설계 기본 속도)
- 커맨더 파라미터: `profile`(`trapezoid|scurve|quintic|cycloidal`, 기본 `scurve`), `speed` [m/s], `accel` [m/s²].
- 적당한 z 는 설계마다 다릅니다. 위 goal 예시는 `edu_servo` 기준입니다.
  홈 자세의 TCP z 는 `edu_servo` ≈ −0.246 m, `edu_dynamixel` ≈ −0.349 m 이고, 중심축(x=y=0)에서 닿는 z 는
  대략 `edu_servo` −0.17 ~ −0.35 m, `edu_dynamixel` −0.25 ~ −0.49 m 입니다. 가장자리로 갈수록 좁아지므로
  시뮬레이터 설계 탭의 작업 실린더를 확인하세요.

## 5. 라이브러리 코드를 ROS 2 로 (`backend="ros2"`)

강의의 Python 코드를 백엔드만 바꿔 그대로 실행합니다. `delta_driver` 가 떠 있어야 하며 **launch 와 같은 설계**를 써야 합니다.

```python
from deltarobot import DeltaRobot

robot = DeltaRobot("edu_servo", backend="ros2")
robot.home()
robot.move_to(0.03, 0.0, -0.25)
robot.tool_on(); robot.wait(0.3); robot.tool_off()
robot.close()
```

```bash
ros2 run delta_robot delta_pick_place_demo --preset edu_servo --cycles 3
```

- `ros2` 백엔드는 `/delta/joint_command` 로 직접 스트리밍합니다(커맨더를 거치지 않음). 커맨더에 goal 을 동시에 보내지 마세요.
- 관절 속도가 모터 한계를 넘는 동작은 라이브러리가 `WorkspaceError` 로 거부합니다. 드라이버도 한 번 더 속도를 제한합니다.
- `/delta/estop` 이 True 가 되면 프로그램에서 `EmergencyStop` 예외가 납니다.

## 6. 실물 (아두이노 + RC 서보)

```bash
sudo usermod -aG dialout $USER     # 한 번만, 다시 로그인
ros2 launch delta_robot bringup.launch.py preset:=edu_servo port:=/dev/ttyACM0
```

- 펌웨어·배선·보정: `firmware/README.md`. 드라이버는 `J`(최대 50 Hz)·`T`·`E`·`R` 을 보내며,
  서보는 위치 피드백이 없으므로 `/joint_states` 는 **명령한(속도 제한된) 각도**입니다.
- 드라이버는 시작할 때 `R`(비상정지 해제)과 `T 0` 을 보냅니다. 처음에는 반드시 로드를 뺀 상태에서 확인하세요.

## 7. 노드 요약

| 노드 | 구독 | 발행 | 파라미터 |
|---|---|---|---|
| `delta_driver` | `/delta/joint_command`, `/delta/tool_command`, `/delta/estop` | `/joint_states`, `/delta/tool_state`, `/delta/tcp` | `preset`, `design_file`, `hardware`, `port`, `baud`, `rate`(100) |
| `delta_commander` | `/delta/goal`, `/joint_states`, `/delta/estop` | `/delta/joint_command`, `/delta/tool_command`, `/delta/busy` · 서비스 `/delta/tool` | `preset`, `design_file`, `profile`, `speed`, `accel` |
| `delta_web_bridge` | `/joint_states`, `/delta/tool_state` | `/delta/estop`, `/delta/goal` | `preset`, `design_file`, `scene`, `host`, `port` |

- 드라이버(mock): 각 모터는 `모터 max_speed / 감속비` 속도로 명령을 따라가고, θ 한계로 잘리며, 순기구학이 안 풀리는 조합은 거부(경고 로그).
  비상정지 중에는 현재 자세를 유지하고 명령을 무시합니다. 1 s 동안 명령이 없으면 마지막 목표를 유지(로그 1회).
- 웹 브리지 프로토콜(hello/design/state/estop/goal): `docs/PROTOCOL.md`.
- ROS 없이 테스트할 수 있는 로직은 `delta_robot/core.py` 에 있습니다: 저장소 루트에서 `python -m pytest tests/test_ros2_core.py`.

## 8. 자주 나는 오류

| 증상 | 원인·해결 |
|---|---|
| `ModuleNotFoundError: deltarobot` | 라이브러리 미설치. §1 의 `pip install -e python` (venv 를 썼다면 activate) |
| `externally-managed-environment` | Ubuntu 24.04. `--break-system-packages` 또는 `--system-site-packages` venv |
| RViz 에 로봇이 안 보임 | Fixed Frame 이 `base_link` 인지, RobotModel 의 Description Topic 이 `/robot_description` 인지 확인 |
| 시뮬레이터 연결 실패 | `web_bridge:=true` 로 실행했는지, 8765 포트를 다른 프로그램(websim)이 쓰고 있지 않은지 확인 |
| goal 이 거부됨 | 작업영역 밖 / 경로 중간 한계 / 속도 초과. 로그 메시지와 `/delta/tcp` 로 현재 위치 확인 |
| `/dev/ttyACM0` 권한 오류 | `dialout` 그룹 추가 후 재로그인 |
