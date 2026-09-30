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
| `sar.perception` | `TargetDetector(color, model_path)` | 대상 색(`HSV_RANGES` 키), YOLO 가중치 경로 또는 `None` | 검출기. YOLO는 생성 시 1회 로드 |
| `sar.perception` | `TargetDetector.detect(bgr)` | BGR 이미지 또는 `None` | 가장 가까운 대상 검출 결과 `dict` 또는 `None` |
| `sar.perception` | `TargetDetector.detect_all(bgr)` | BGR 이미지 또는 `None` | 대상 검출 결과 목록. 가까운 순서 |
| `sar.perception` | `TargetDetector.to_world(det, pose)` | 검출 결과, `Pose` 또는 `(x, y, theta)` | 대상 월드 좌표 `(x, y)` |
| `sar.perception` | `Confirm.update(seen, xy, dist=None)` | 검출 여부, 대상 월드 좌표, 거리 [m] | `(확정 여부, 위치 추정 또는 None)` |
| `sar.perception` | `is_excluded(xy, found)` | 대상 월드 좌표, 구조 완료 위치 목록 | `FOUND_EXCLUDE_RADIUS` 이내 여부 |
| `sar.viz` | `render_map(grid, to_cell, trajectory, path, rescued, start, pose, title)` | 공개값 격자(-1/0/1), 월드→격자 변환 함수, 월드 좌표 목록 | BGR 지도 그림 |
| `sar.viz` | `save_map(path, image)` | 저장 경로, 그림 | 저장 성공 여부 `bool` |

실습 월드의 LDS-01 LiDAR는 360개 거리값을 반환합니다. Intro 실습 코드 기준 인덱스 180이 전방, 0이 후방, 90이 좌측, 270이 우측입니다.

### 대상 검출 결과

`detect()`와 `detect_all()`의 결과 항목은 다음 키를 가집니다.

| 키 | 단위 | 내용 |
|---|---|---|
| `cx`, `cy` | px | 대상 중심 이미지 좌표 |
| `w`, `h` | px | 상자 폭·높이. 색 분할 검출은 외접원 지름 |
| `conf` | 0~1 | YOLO 신뢰도. 색 분할 검출은 원형도 |
| `cls` | 정수 | COCO 클래스 번호. 색 분할 검출은 `None` |
| `color` | 문자열 | 대상 색 |
| `dist` | m | 카메라에서 대상까지 직선거리 |
| `bearing` | rad | 정면 기준 방위각. 왼쪽이 양수 |
| `source` | 문자열 | `"yolo"` 또는 `"color"` (YOLO 미검출로 색 분할 사용) |

거리와 방위각 계산식은 다음과 같습니다. $W$는 이미지 폭, $\phi_h$는 수평 화각(`CAMERA_FOV`), $D$는 사과 지름(`TARGET_DIAMETER`), $w$는 상자 폭과 높이 중 큰 값입니다.

$$
f = \frac{W / 2}{\tan(\phi_h / 2)}, \qquad
\beta = -\arctan\frac{c_x - W/2}{f}, \qquad
d = \frac{f D}{w \cos\beta}
$$

과제 기준 문서의 $d \approx f D / w$는 광축 방향 깊이입니다. 화면 가장자리의 대상은 직선거리와 차이가 커서 $\cos\beta$로 나눕니다. 화면 끝에서 잘린 상자는 한 변만 줄어들므로 긴 변을 지름으로 사용합니다.

### 검출 조건

| 조건 | 설정값 | 내용 |
|---|---|---|
| YOLO 후보 | `YOLO_CLASSES`, `YOLO_CONF` | apple, orange, sports ball |
| 색 판별 | `HSV_RANGES`, `COLOR_RATIO_MIN` | YOLO 상자 안 대상 색 픽셀 비율 하한 |
| 색 분할 대체 | `USE_COLOR_FALLBACK`, `MIN_BLOB_AREA`, `MIN_CIRCULARITY` | YOLO 대상이 없을 때만 실행 |
| 높이 제외 | `HORIZON_MARGIN` | 중심이 화면 가운데선보다 이 값 이상 위면 식탁 위 물체로 제외 |
| 연속 확인 | `CONFIRM_FRAMES`, `CONFIRM_MATCH_RADIUS` | 연속 검출 위치가 반경을 벗어나면 기록 초기화 |
| 위치 추정 | `CONFIRM_MIN_DIST` | 가중치 $1 / \max(d, d_{\min})^2$의 거리 가중 평균 |

`detect()`는 호출할 때마다 YOLO를 실행합니다. 미션 루프에서 `YOLO_EVERY` step마다 호출하십시오. YOLO 로드에 실패하면 `load_error`에 사유를 기록하고 색 분할만 사용합니다.

## 예정 인터페이스

해커톤 준비 자료의 인터페이스를 `src/sar` 패키지 구조에 맞춘 형식입니다. 구현하면 위 표로 옮기십시오.

| 모듈 | 함수 | 입력 | 출력 | 담당 |
|---|---|---|---|---|
| `sar.planning` | `astar(grid, start_rc, goal_rc)` | 격자, 시작·목표 `(row, col)` | `[(row, col), ...]` 또는 `None` | B |
| `sar.planning` | `next_frontier_path(grid, robot_rc)` | 격자, 로봇 `(row, col)` | `[(row, col), ...]` 또는 `None` | B |
| `sar.control` | `go_to(pose, target_xy)` | `Pose`, 목표 `(x, y)` | `(v, w, dist)` | D |
| `sar.mission` | `tick()` | 없음 | 없음. 10 Hz 호출(`config.MISSION_HZ`) | A |
