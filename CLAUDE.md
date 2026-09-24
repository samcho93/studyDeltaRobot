# CLAUDE.md — studyDeltaRobot : 델타로봇 인터랙티브 강의 사이트

> 이 저장소에서 작업할 때 따르는 **프로젝트 헌장**. 구조·규칙·좌표 규약을 지킨다.
> 모호한 점은 추측으로 구조를 바꾸지 말고 `docs/DECISIONS.md`에 남긴다.

## 0. 한 줄 요약

GitHub Pages로 배포되는 **정적 강의 사이트**에서
① 델타로봇 이론·기구학 강의, ② **설계 파라미터(암 길이·반지름·모터·감속기·엔드이펙터·재질·페이로드)를 골라
바로 시뮬레이션**하는 3D 시뮬레이터, ③ 설계값으로 생성하는 **URDF**와 뷰어,
④ Pyodide Python Playground(`deltarobot` 라이브러리), ⑤ PC Python / **ROS 2** / 실물(아두이노) 연동을 제공한다.

- 참고 구현: `samcho93/studyGo2` (content/*.md → build.py → lessons/*.html, sim/, tools/ 구조 계승)
- 배포 URL: `https://samcho93.github.io/studyDeltaRobot/`
- 언어: 본문·UI는 한국어, 코드·커밋은 영어(주석 한국어 허용).

## 1. 핵심 설계 원칙

1. **Static-first**: 서버 없이 GitHub Pages만으로 동작. npm 빌드 체인 금지. 라이브러리는 jsdelivr 버전 고정 URL.
2. **단일 진실 공급원**
   - 커리큘럼: `content/curriculum.json`
   - 부품 카탈로그(모터·감속기·재질·툴·프리셋): `python/deltarobot/data/catalog.json` — JS도 이 파일을 fetch 한다.
   - 기구학/동역학/궤적/URDF: Python(`python/deltarobot/`)과 JS(`assets/js/delta/`) **두 구현이 같은 결과**를 내야 한다.
     `tests/test_js_parity.py`가 node로 JS를 실행해 Python과 비교한다. 한쪽을 고치면 다른 쪽도 고친다.
3. **같은 코드, 여러 타깃**: 학생 Python 코드는 `DeltaRobot(backend=...)`만 바꿔 Playground(시뮬 기록) → PC(websim) → ROS 2 → 실물(serial)로 옮긴다.
4. **안전**: 실물 백엔드(serial)와 ROS 2 드라이버는 관절 한계·속도 제한·작업영역 검사를 **라이브러리/드라이버 쪽에서** 강제한다.

## 2. 좌표·파라미터 규약 (절대 바꾸지 말 것)

- 월드(=base) 좌표: 원점은 베이스 중심, 모터 축들이 놓인 평면. **z 위쪽**, 작업은 z<0. 단위 SI(m, rad, kg, s).
- 팔 i=1,2,3 의 방위각 φᵢ = 0°, 120°, 240° (팔 1이 +x 방향).
- 설계 파라미터: `R` base_radius(중심→모터축), `r` effector_radius(중심→볼조인트 쌍 중심),
  `L` upper_arm, `l` forearm(평행사변형 로드 길이), `w` forearm_spacing(로드 간격).
- 모터각 θᵢ: 위팔이 수평일 때 0, **아래로 내려가면 +**. 팔 좌표계에서 팔꿈치 E = (R + L cosθ, 0, −L sinθ).
- 역기구학 해 선택: 두 해 중 cosθ가 큰 쪽(팔꿈치가 바깥쪽). 순기구학 해 선택: z가 작은(아래) 쪽.
- 툴 중심점 TCP = 이펙터 중심 + (0,0,−tool.length).

## 3. 디렉터리 구조

```
studyDeltaRobot/
├── build.py                 # content/*.md → lessons/*.html, index.html
├── content/curriculum.json  # 트랙: theory(T) · design(D) · control(R)
├── content/{theory,design,control}/*.md, content/figures/*.svg (+ gen_figures.py)
├── lessons/ index.html      # 빌드 산출물 — 직접 수정 금지
├── sim/                     # 3D 시뮬레이터 (설계·조그·작업·분석·연결 탭)
├── tools/                   # urdf-viewer, playground(+worker), kinematics-lab
├── assets/js/delta/         # JS 코어: kinematics, design, dynamics, trajectory, scene, urdf
├── python/deltarobot/       # Python 라이브러리 (pip install -e python)
│   ├── data/catalog.json    #   ★ 부품 카탈로그
│   └── backends/            #   record(Playground) / websim / serial / ros2 / mock
├── ros2_ws/src/delta_robot/ # ROS 2 (ament_python): driver, commander, web_bridge, launch, rviz
├── firmware/delta_servo/    # 아두이노 스케치 (시리얼 프로토콜)
├── tests/                   # pytest + JS parity + 링크 검사
└── docs/                    # DECISIONS, PROTOCOL, API, SOURCES
```

## 4. 챕터 Markdown 규칙

front matter(`id, track, title, duration, level, requires, tools`) 후 섹션:
`## 학습 목표` → 이론/실습 → `## 자주 나는 오류와 해결`(필수) → `## 과제` → `## 참고자료`(필수).
커스텀 블록 `:::tip|info|warning|danger|safety|check|task|mission`, 그림 `@fig[name] 캡션`, 버튼 `@btn[~/sim/index.html] 라벨`,
실행 코드 ` ```python run `(Playground 열기 버튼), ` ```python robot `(실물 전용 배지), 수식 `$...$`.
실물 실습 섹션(`## 실습 (실물)`)에는 `:::safety` 필수.

## 5. 개발 명령

```bash
python build.py            # 빌드   (--serve 미리보기 :8000, --check 검사)
python -m pytest tests/    # Python + JS parity 테스트 (node 필요)
python tests/check_links.py
python python/make_manifest.py   # Playground에 마운트할 파일 목록 갱신 (python/ 수정 후)
```

## 6. 하지 말 것

- `lessons/`, `index.html` 직접 수정 · 서버 필요 기능 · `@latest` CDN
- Python/JS 한쪽만 수정해 parity 깨기 · 카탈로그 값을 코드에 하드코딩
- 카탈로그 모터 사양을 특정 제품의 보증값처럼 서술 (대표값이며 "데이터시트 확인" 표기)
- 실물 안전 제한(속도·관절 한계)을 기본값으로 완화
