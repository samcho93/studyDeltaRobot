# studyDeltaRobot — 델타로봇 이론 · 설계 · 시뮬레이션 · ROS 2 인터랙티브 강의

델타로봇(회전형 3자유도 병렬로봇)을 **기구학 이론부터 설계 검증, 파이썬 제어, ROS 2, 실물 아두이노까지** 브라우저에서
배우는 한국어 강의 사이트입니다. GitHub Pages로 배포되는 정적 사이트이며 서버가 필요 없습니다.

- **사이트**: https://samcho93.github.io/studyDeltaRobot/
- **트랙**: 기초 이론·기구학(T01–T08) · 설계·시뮬레이션(D01–D11, 응용실습 3개 포함) · Python·ROS 2·실물(R01–R08)

## 도구

| 도구 | 경로 | 내용 |
|---|---|---|
| 3D 시뮬레이터 | `sim/` | 암 길이·반지름·재질·모터·감속기·엔드이펙터·페이로드를 골라 바로 시뮬레이션. 조그(IK/FK), 데모 프로그램(Adept 사이클·분류·컨베이어 트래킹·그리기), 모터 토크/속도 그래프와 검증, 작업영역 점구름·작업 실린더, PC Python/ROS 2 연결, 설계 JSON·URDF 내보내기 |
| Python Playground | `tools/playground.html` | Pyodide에서 `deltarobot` 라이브러리를 실행 → 기록된 동작을 내장 시뮬레이터로 재생 + 모터 검증 |
| URDF 뷰어 | `tools/urdf-viewer.html` | 설계값으로 생성한 URDF를 urdf-loader로 표시, 수동 관절을 계산해 닫힌 사슬 재현 |
| 기구학 실험실 | `tools/kinematics-lab.html` | IK(원 ∩ 원)·FK(세 구의 교점)를 2D 그림과 단계별 숫자로 |

## deltarobot 파이썬 라이브러리

```bash
pip install -e python                 # 표준 라이브러리만 사용
pip install -e "python[websim]"       # PC → 웹 시뮬레이터 실시간 연결
pip install -e "python[serial]"       # 아두이노 실물
```

```python
from deltarobot import DeltaRobot
robot = DeltaRobot("edu_dynamixel", scene="pick_place")   # backend="record"(기본) | "websim" | "serial" | "ros2"
robot.home()
part = robot.parts()[0]
robot.arch_to(part["x"], part["y"], part["top"], height=0.03)
robot.tool_on()
box = robot.scene.bin("A")
robot.arch_to(box["x"], box["y"], part["top"] + 0.02)
robot.tool_off()
print(robot.analyze())          # 피크/RMS 토크, 속도, 관성비 검증
```

같은 코드를 `backend`만 바꿔 Playground(가상 시간 기록) → PC(websim) → ROS 2 → 실물(serial)로 옮깁니다.
API 전체: [`docs/API.md`](docs/API.md) · 통신 규약: [`docs/PROTOCOL.md`](docs/PROTOCOL.md)

```bash
python -m deltarobot.urdf --preset edu_dynamixel -o delta.urdf     # URDF 생성 (웹 생성기와 글자 하나까지 동일)
```

## ROS 2 (Humble / Jazzy)

`ros2_ws/src/delta_robot` — `delta_driver`(mock/serial, `/joint_states`에 수동 관절까지 발행), `delta_commander`(`/delta/goal`, `/delta/tool`),
`delta_web_bridge`(웹 시뮬레이터를 ROS 화면으로), launch·RViz 설정. 자세한 절차는 패키지 README와 R04–R06 강의를 보세요.

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch delta_robot display.launch.py preset:=edu_servo web_bridge:=true
```

## 실물

`firmware/delta_servo/` — Arduino Uno/Nano + MG996R ×3 스케치(시리얼 `J/T/E/R/?` 프로토콜, 캘리브레이션, 각도·속도 제한). R07 참고.

## 로컬 빌드·미리보기·테스트

```bash
python build.py            # content/*.md → lessons/*.html, index.html (+ Playground manifest 갱신)
python build.py --serve    # http://localhost:8000
python -m pytest tests     # 기구학·동역학·URDF, Python↔JS 일치(node 필요), 데모·예제, ROS 2 core
```

`lessons/`와 `index.html`은 빌드 산출물입니다. 본문은 `content/`에서 고칩니다. 프로젝트 규약은 [`CLAUDE.md`](CLAUDE.md).

## 참고

모터·감속기·툴 사양은 교육용 **대표값**이며 실제 선정 시 제조사 데이터시트를 확인하세요. 실물 실습은 교수 입회 하에 진행하세요.
사용 라이브러리: three.js 0.186.0 (MIT), urdf-loader 0.13.1 (Apache-2.0), Pyodide 0.29.5 (MPL-2.0), CodeMirror 6 (MIT).
