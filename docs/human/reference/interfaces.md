# 모듈 인터페이스

sar-robot 모듈의 구현 상태와 코드 위치를 정리한 문서입니다. 좌표·단위 규칙과 함수 형식의 원본은 docs [과제와 구현 기준](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/sar-과제-구현-기준.md)입니다. 좌표 규칙은 6장, 파일 구조와 함수 형식은 7장에 있습니다. 인터페이스를 바꾸려면 사용하는 모듈 담당자와 먼저 합의하고 원본 문서, `CONTEXT.md`, 이 문서를 같은 시점에 갱신하십시오.

모든 경로는 `controllers/sar_main/` 기준입니다.

## 구현된 인터페이스

| 파일 | 함수·클래스 | 입력 | 출력 |
|---|---|---|---|
| `sar/robot_io.py` | `wheel_speeds(v, w)` | 속도 명령 [m/s, rad/s] | 좌우 바퀴 각속도 [rad/s]. 최대값 초과 시 비율 유지 축소 |
| `sar/robot_io.py` | `RobotIO.step()` | 없음 | 시뮬레이션 계속 여부 `bool` |
| `sar/robot_io.py` | `RobotIO.time()` | 없음 | 시뮬레이션 시간 [s] |
| `sar/robot_io.py` | `RobotIO.encoders()` | 없음 | 좌우 바퀴 누적 회전각 [rad] |
| `sar/robot_io.py` | `RobotIO.compass()` | 없음 | 나침반 벡터. 장치가 없으면 `None` |
| `sar/robot_io.py` | `RobotIO.lidar()` | 없음 | 거리 360개 [m]. 장치가 없으면 `None` |
| `sar/robot_io.py` | `RobotIO.camera_bgr()` | 없음 | `(480, 640, 3)` BGR 이미지. 없으면 `None` |
| `sar/robot_io.py` | `RobotIO.drive(v, w)` | 속도 명령 [m/s, rad/s] | 없음 |
| `sar/odometry.py` | `Odometry(x, y, theta)` | 시작 pose | 객체 |
| `sar/odometry.py` | `Odometry.update(enc_l, enc_r, compass=None)` | 누적 회전각 [rad], 보정된 나침반 방향 [rad] | 없음. 첫 호출은 기준값 저장 |
| `sar/odometry.py` | `Odometry.pose()` | 없음 | `(x, y, theta)` |
| `sar/odometry.py` | `Odometry.heading_var()` | 없음 | 방향 분산 P [rad²] |
| `sar/odometry.py` | `Odometry.correct(dx, dy, dth)` | 보정값 [m, m, rad] | 없음 |

LDS-01 라이다 인덱스는 180이 정면, 90이 왼쪽, 270이 오른쪽, 0이 뒤입니다. `compass()`는 원시 벡터를 반환합니다. 방향 [rad]으로 변환하고 부호·오프셋을 보정하는 작업은 미션의 INIT_SPIN 단계에서 수행합니다.

## 미구현 모듈

| 파일 | 주요 인터페이스 | 담당 |
|---|---|---|
| `sar/mission.py` | `Mission.tick()` | 통합 |
| `sar/grid_map.py` | `GridMap.update()`, `to_cell()`, `to_world()`, `layers()`, `frontiers()` | 계획 |
| `sar/planner.py` | `plan()`, `choose_frontier()` | 계획 |
| `sar/perception.py` | `TargetDetector.detect()`, `to_world()`, `Confirm.update()` | 인지 |
| `sar/viz.py` | 지도·경로·대상 위치 그림 저장 | 인지 |
| `sar/local_control.py` | `pure_pursuit()`, `safety_filter()` | 행동 |

구현하면 위 표에서 "구현된 인터페이스" 표로 옮기십시오. 함수 형식은 원본 문서 7.2절을 따르십시오.
