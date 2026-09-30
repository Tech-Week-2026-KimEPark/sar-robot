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
| `sar/odometry.py` | `Odometry.set_wheel_separation_scale(scale)` | 스케일 배율 (기본 1.0) | 없음. 이후 `update()`의 회전각(dth) 계산에만 반영. `mission.fit_compass()`가 INIT_SPIN에서 측정한 오도메트리/나침반 회전 비율을 전달 |
| `sar/perception.py` | `TargetDetector(color, model_path)` | 대상 색(`HSV_RANGES` 키), YOLO 가중치 경로 또는 `None` | 검출기. YOLO는 생성 시 1회 로드 |
| `sar/perception.py` | `TargetDetector.detect(bgr)` | BGR 이미지 또는 `None` | 가장 가까운 대상 검출 결과 `dict` 또는 `None` |
| `sar/perception.py` | `TargetDetector.detect_all(bgr)` | BGR 이미지 또는 `None` | 대상 검출 결과 목록. 가까운 순서 |
| `sar/perception.py` | `TargetDetector.to_world(det, pose)` | 검출 결과, `(x, y, theta)` | 대상 월드 좌표 `(x, y)` |
| `sar/perception.py` | `Confirm.update(seen, xy, dist=None)` | 검출 여부, 대상 월드 좌표, 거리 [m] | `(확정 여부, 위치 추정 또는 None)` |
| `sar/perception.py` | `is_excluded(xy, found)` | 대상 월드 좌표, 구조 완료 위치 목록 | `FOUND_EXCLUDE_RADIUS` 이내 여부 |
| `sar/viz.py` | `render_map(grid, to_cell, trajectory, path, rescued, start, pose, title)` | 공개값 격자(-1/0/1), 월드→격자 변환 함수, 월드 좌표 목록 | BGR 지도 그림 |
| `sar/viz.py` | `save_map(path, image)` | 저장 경로, 그림 | 저장 성공 여부 `bool` |
| `sar/grid_map.py` | `GridMap(center_x, center_y, size, res)` | 지도 중심 [m]. 기본값은 시작점, `MAP_SIZE`, `MAP_RES` | 지도 객체. 모든 칸 모름 |
| `sar/grid_map.py` | `GridMap.update(pose, ranges)` | `(x, y, theta)`, 라이다 거리 목록 | 없음. `None`이면 무시 |
| `sar/grid_map.py` | `GridMap.to_cell(x, y)` | 월드 좌표 [m] | `(row, col)` |
| `sar/grid_map.py` | `GridMap.to_world(row, col)` | 칸 | 칸 중심 `(x, y)` [m] |
| `sar/grid_map.py` | `GridMap.layers()` | 없음 | `(occ, blocked, soft, unknown)` bool 배열 |
| `sar/grid_map.py` | `GridMap.frontiers()` | 없음 | `[(칸 수, [(row, col), ...]), ...]`. 칸 수 내림차순 |
| `sar/grid_map.py` | `GridMap.public()` (추가) | 없음 | 공개값 격자 int8 (-1/0/1). `render_map` 입력 |
| `sar/grid_map.py` | `GridMap.clearance()` (추가) | 없음 | 가장 가까운 장애물까지 거리 [m] 배열 |
| `sar/planner.py` | `plan(grid, start_xy, goal_xy, allow_unknown=False, field=None)` | 지도, 시작·목표 `(x, y)` | 0.1 m 간격 경로 `[(x, y), ...]`. 없으면 `None` |
| `sar/planner.py` | `choose_frontier(grid, pose, blacklist)` | 지도, `(x, y, theta)`, 제외 좌표 목록 | 프론티어 목표 `(x, y)`. 없으면 `None` |
| `sar/planner.py` | `DistanceField(grid, origin_xy)` (추가) | 지도, 기준점 `(x, y)` | 기준점까지 거리 지도 |
| `sar/planner.py` | `DistanceField.distance(xy)` (추가) | 월드 좌표 | 기준점까지 경로 비용 [m]. 도달 불가면 `None` |
| `sar/planner.py` | `DistanceField.path(xy)` (추가) | 월드 좌표 | xy에서 기준점까지 경로. 도달 불가면 `None` |
| `sar/local_control.py` | `pure_pursuit(pose, path, lookahead=config.LOOKAHEAD)` | pose `(x, y, theta)`, 경로 `[(x, y), ...]` | `(v, w, reached)`. 목표 각도 차이 55° 이상이면 제자리 회전 |
| `sar/local_control.py` | `safety_filter(v, w, ranges)` | 속도 명령, 라이다 360개 | `(v, w, blocked)`. 정면 ±25° 콘 안 `config.STOP_DIST` 이내면 정지 |
| `sar/mission.py` | `Mission(io, odom, grid, detector, log=None)` | `RobotIO`, `Odometry`, `GridMap`, `TargetDetector`, 로그 함수 | 미션 객체. 시작점은 생성 시 `odom.pose()` |
| `sar/mission.py` | `Mission.tick()` | 없음 | 없음. 매 step 1회 호출 |
| `sar/mission.py` | `fit_compass(samples)` (추가) | 시작 회전 `[(오도메트리 방향, 나침반 원시각), ...]` | `(sign, offset, scale)`. 회전량 부족·비율 불일치면 `None` |
| `sar/mission.py` | `compass_heading(vec, sign, offset)` (추가) | 나침반 벡터, 보정값 | 보정된 방향 [rad] |

LDS-01 라이다 인덱스는 180이 정면, 90이 왼쪽, 270이 오른쪽, 0이 뒤입니다. `compass()`는 원시 벡터를 반환합니다. 방향 [rad]으로 변환하고 부호·오프셋을 보정하는 작업은 미션의 INIT_SPIN 단계에서 수행합니다.

`Mission`의 상태 전환과 공통 규칙은 docs [mission 기능 설명](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/explanation/features/mission.md)에 있습니다. 로그는 `log` 함수로 전달하며 `sar_main.py`가 `print`를 넘깁니다.

## 지도와 경로 계획 사용 규칙

"(추가)" 표시 항목은 원본 문서 7.2절에 없는 추가 인터페이스입니다. 기존 함수 형식은 바꾸지 않았습니다.

| 항목 | 규칙 |
|---|---|
| 호출 주기 | `GridMap.update()`는 매 step. `plan()`, `choose_frontier()`, `DistanceField`는 목표 변경, 경로 차단, 2초 주기에만 호출 |
| 경로 형식 | 첫 점은 `start_xy`, 점 간격은 `PATH_STEP`. `pure_pursuit`에 그대로 전달 |
| 통과 불가 시작·목표 | `SNAP_RADIUS` 안의 가장 가까운 통과 가능 칸을 경유. 목표를 옮긴 경우 경로 끝점은 대체 칸 중심 |
| 복귀 | `mission`은 `plan(grid, pose, 시작점, allow_unknown=True)`을 2초 주기로 호출. `DistanceField`는 복귀 거리 판단이 필요할 때 사용 |
| A* 가속 | `plan(..., field=f)`에 목표 기준 `DistanceField`를 전달하면 휴리스틱으로 사용. 결과 경로 비용은 같음 |
| 지도 그림 | `render_map(grid.public(), grid.to_cell, ...)` |

동작 원리와 검증 결과는 docs [grid_map 기능 설명](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/explanation/features/grid_map.md)과 [planner 기능 설명](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/explanation/features/planner.md)에 있습니다.

## 대상 검출 결과

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

원본 문서 6장의 $d \approx f D / w$는 광축 방향 깊이입니다. 화면 가장자리의 대상은 직선거리와 차이가 커서 $\cos\beta$로 나눕니다. 화면 끝에서 잘린 상자는 한 변만 줄어들므로 긴 변을 지름으로 사용합니다.

## 검출 조건

| 조건 | 설정값 | 내용 |
|---|---|---|
| YOLO 후보 | `YOLO_CLASSES`, `YOLO_CONF` | apple, orange, sports ball |
| 색 판별 | `HSV_RANGES`, `COLOR_RATIO_MIN` | YOLO 상자 안 대상 색 픽셀 비율 하한 |
| 색 분할 대체 | `USE_COLOR_FALLBACK`, `MIN_BLOB_AREA`, `MIN_CIRCULARITY` | YOLO 대상이 없을 때만 실행 |
| 높이 제외 | `HORIZON_MARGIN` | 중심이 화면 가운데선보다 이 값 이상 위면 식탁 위 물체로 제외 |
| 연속 확인 | `CONFIRM_FRAMES`, `CONFIRM_MATCH_RADIUS` | 연속 검출 위치가 반경을 벗어나면 기록 초기화 |
| 위치 추정 | `CONFIRM_MIN_DIST` | 가중치 $1 / \max(d, d_{\min})^2$의 거리 가중 평균 |

`detect()`는 호출할 때마다 YOLO를 실행합니다. 미션 루프에서 `YOLO_EVERY` step마다 호출하십시오. YOLO 로드에 실패하면 `load_error`에 사유를 기록하고 색 분할만 사용합니다.

측정·튜닝용으로 `TargetDetector`는 마지막 `detect_all()` 호출의 YOLO 결과를 보관합니다. `last_yolo`는 색 판별에서 탈락한 상자를 포함한 원본 상자 목록입니다. 각 항목은 `xyxy`, `conf`, `cls`, `ratio`(상자 안 대상 색 비율) 키를 가집니다. `last_yolo_ms`는 YOLO 추론 1회 시간 [ms]입니다.
