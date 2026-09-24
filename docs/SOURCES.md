# 출처 · 라이브러리 · 라이선스

## 웹 라이브러리 (CDN, 버전 고정)

| 라이브러리 | 버전 | 용도 | 라이선스 |
|---|---|---|---|
| three.js | 0.186.0 | 3D 렌더링 (시뮬레이터, URDF 뷰어) | MIT |
| urdf-loader (gkjohnson) | 0.13.1 | URDF 파싱·표시 | Apache-2.0 |
| Pyodide | 0.29.5 | 브라우저 Python (Playground) | MPL-2.0 |
| CodeMirror | 6 (codemirror 6.0.2 등) | 코드 편집기 | MIT |

## 참고 문헌 (강의 본문에서 인용)

- R. Clavel, "DELTA, a fast robot with parallel geometry", Proc. 18th Int. Symp. on Industrial Robots, 1988.
- R. Clavel, *Conception d'un robot parallèle rapide à 4 degrés de liberté*, PhD thesis, EPFL, 1991.
- L. Codourey, "Dynamic modeling of parallel robots for computed-torque control implementation", *Int. J. Robotics Research* 17(12), 1998.
- J.-P. Merlet, *Parallel Robots*, 2nd ed., Springer, 2006.
- B. Siciliano, O. Khatib (eds.), *Springer Handbook of Robotics* — Parallel Mechanisms chapter.

## 도구·표준

- ROS 2 문서: https://docs.ros.org/
- URDF: https://wiki.ros.org/urdf/XML
- websockets (Python): https://websockets.readthedocs.io/
- Arduino Servo 라이브러리: https://docs.arduino.cc/libraries/servo/

## 이 저장소의 코드

- `python/deltarobot/`, `assets/js/delta/`, `sim/`, `tools/`, `ros2_ws/`, `firmware/` — 이 강의를 위해 작성.
- 사이트 빌더·공통 스타일은 자매 강의 studyGo2 / studySOArm101에서 계승.
