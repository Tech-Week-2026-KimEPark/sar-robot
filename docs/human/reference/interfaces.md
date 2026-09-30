# 모듈 인터페이스

sar-robot 모듈 사이에서 사용하는 좌표·단위 규칙과 함수 형식입니다. 인터페이스를 바꾸려면 사용하는 모듈 담당자와 먼저 합의하고 이 문서를 같은 PR에서 갱신하십시오.

## 좌표·단위 규칙

| 항목 | 규칙 |
|---|---|
| 월드 좌표 | `(x, y)` [m] |
| 방향 | `theta` [rad], 반시계 방향이 양수, 범위 [-π, π) |
| 로봇 기준 좌표 | x는 전방 거리, y는 좌측 거리 |
| 격자 지도 | `grid[row][col]`, row는 y, col은 x에 대응 |
| 격자 값 | -1 미확인, 0 빈칸, 1 벽 |
| 격자 해상도 | 0.05 m/cell (`config.GRID_RESOLUTION`) |
| 격자 원점 | `grid[0][0]` 좌하단 모서리의 월드 좌표 `origin` |
| 속도 명령 | 선속도 `v` [m/s], 각속도 `w` [rad/s], 좌회전이 양수 |
| 이미지 | BGR `uint8` 배열 `(height, width, 3)` |

격자 값 -1/0/1은 강의 자료의 Trinary 표현(-1/0/100)과 다릅니다. 팀 코드는 -1/0/1을 사용합니다.

Webots 월드 좌표축은 월드마다 다를 수 있습니다. 대회 월드를 받으면 D가 좌표축과 회전 방향을 측정하고 A가 `CONTEXT.md`에 기록합니다.

## 구현된 인터페이스

| 모듈 | 함수·클래스 | 입력 | 출력 |
|---|---|---|---|
| `sar.geometry` | `Pose` | `x`, `y`, `theta` | 불변 데이터 클래스 |
| `sar.geometry` | `normalize_angle(angle)` | rad | [-π, π) 범위 rad |
| `sar.geometry` | `world_to_grid(x, y, origin, resolution)` | m | `(row, col)` |
| `sar.geometry` | `grid_to_world(row, col, origin, resolution)` | 격자 인덱스 | 격자 중심 `(x, y)` |
| `sar.geometry` | `to_robot_frame(px, py, pose)` | 월드 좌표 점, `Pose` | 로봇 기준 `(x, y)` |
| `sar.localization.odometry` | `WheelOdometry.update(left_angle, right_angle)` | 바퀴 누적 회전각 [rad] | `Pose` |
| `sar.robot_io` | `RobotIO.step()` | 없음 | 시뮬레이션 계속 여부 `bool` |
| `sar.robot_io` | `RobotIO.lidar_ranges()` | 없음 | 거리 목록 [m] |
| `sar.robot_io` | `RobotIO.wheel_angles()` | 없음 | `(left, right)` [rad] |
| `sar.robot_io` | `RobotIO.camera_bgr()` | 없음 | BGR 이미지 |
| `sar.robot_io` | `RobotIO.set_wheel_speeds(v_left, v_right)` | 바퀴 선속도 [m/s] | 없음. `MAX_WHEEL_SPEED`로 제한 |

실습 월드의 LDS-01 LiDAR는 360개 거리값을 반환합니다. Intro 실습 코드 기준 인덱스 180이 전방, 0이 후방, 90이 좌측, 270이 우측입니다.

## 예정 인터페이스

해커톤 준비 자료의 인터페이스를 `src/sar` 패키지 구조에 맞춘 형식입니다. 구현하면 위 표로 옮기십시오.

| 모듈 | 함수 | 입력 | 출력 | 담당 |
|---|---|---|---|---|
| `sar.perception` | `detect(frame_bgr)` | BGR 이미지 | `{"err": -1~1, "area": float}` 또는 `None` | C |
| `sar.planning` | `astar(grid, start_rc, goal_rc)` | 격자, 시작·목표 `(row, col)` | `[(row, col), ...]` 또는 `None` | B |
| `sar.planning` | `next_frontier_path(grid, robot_rc)` | 격자, 로봇 `(row, col)` | `[(row, col), ...]` 또는 `None` | B |
| `sar.control` | `go_to(pose, target_xy)` | `Pose`, 목표 `(x, y)` | `(v, w, dist)` | D |
| `sar.mission` | `tick()` | 없음 | 없음. 10 Hz 호출(`config.MISSION_HZ`) | A |

`detect()`의 `err`는 대상 중심의 화면 가로 위치입니다. -1은 화면 왼쪽 끝, 0은 중앙, 1은 오른쪽 끝입니다.
