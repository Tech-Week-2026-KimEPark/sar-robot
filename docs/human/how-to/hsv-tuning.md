# HSV 임계값 튜닝과 인식 성능 측정

이 문서는 인식 담당이 Webots 월드에서 대상 사과의 HSV 범위를 조정하고 인식 성능을 측정하는 절차입니다. 측정 결과로 `config.py`의 인식 설정값을 정합니다. 대회 월드의 조명이 연습 월드와 다르면 이 절차를 다시 수행하십시오.

## 준비

- [개발 환경 설정](dev-setup.md)을 마치고 Webots Python command를 팀 기준 패키지가 설치된 Python 3.10으로 설정함
- YOLO 가중치 `models/YOLO/yolo11n.pt`가 있음. 없으면 색 분할만 사용함
- `config.py`의 `TARGET_COLOR`가 튜닝할 색이고 `START_X`, `START_Y`, `START_THETA`가 월드의 로봇 시작 pose와 같음

## 튜닝 컨트롤러 실행

1. Webots에서 sar-robot의 월드를 여십시오.
2. 로봇 노드의 `controller` 필드를 `hsv_tuner`로 바꾸십시오.
3. Reset 후 시뮬레이션을 시작하십시오.
4. `hsv_tuner` 창을 클릭해 포커스를 옮기십시오. 키 입력은 이 창에서만 동작합니다.

| 키 | 동작 |
|---|---|
| `w`, `s` | 전진, 후진 |
| `a`, `d` | 좌회전, 우회전 |
| `space` | 정지 |
| `m` | 측정값을 CSV에 추가하고 프레임 저장 |
| `g` | 자동 접근. 제자리 회전으로 대상을 찾아 정렬한 뒤 접근하며 3 m, 2 m, 1 m와 정지 지점에서 자동 측정 |
| `o` | 회전 측정. 제자리 1회전하며 30°마다 자동 측정 |
| `p` | 원본 프레임만 저장 |
| `r` | 현재 범위를 `HSV_RANGES` 형식으로 출력 |
| `q` | 종료. 창 닫기 버튼도 같음 |

조종 키는 누른 뒤 다른 키를 누를 때까지 같은 속도를 유지합니다. 자동 주행 중 조종 키를 누르면 즉시 수동 조종으로 돌아갑니다. 수동·자동 모두 라이다 안전 필터가 정면 장애물 앞에서 전진을 멈춥니다. 화면 왼쪽 위에 현재 모드(`manual`, `AutoApproach`, `SpinSurvey`)와 자동 접근 단계가 표시됩니다.

튜닝 중에는 월드를 저장하지 마십시오. 저장하면 `controller` 필드와 시뮬레이션 상태 값이 월드 파일에 기록됩니다. 저장했다면 `git restore worlds/<월드>.wbt`로 되돌리십시오.

## HSV 범위 조정

1. 로봇을 움직여 대상 사과와 다른 색 사과가 화면에 보이게 하십시오.
2. 트랙바를 조정해 오른쪽 마스크 화면에 대상 사과만 흰색으로 남게 하십시오.
3. `r` 키로 콘솔에 출력된 범위를 `config.py`의 `HSV_RANGES`에 붙여 넣으십시오.

H 하한이 상한보다 크면 0을 넘는 범위로 처리합니다. 빨강의 초기값은 H 하한 170, 상한 8입니다. 이 값은 (0~8) ∪ (170~179) 범위입니다.

## 인식 성능 측정

1. 대상 사과가 3 m보다 멀리 있는 위치로 로봇을 옮기십시오. 사과가 화면에 보이지 않아도 됩니다.
2. `g` 키를 누르십시오. 로봇이 사과를 찾아 접근하며 3 m, 2 m, 1 m와 0.5 m 정지 지점에서 자동으로 측정합니다.
3. 다른 색 사과와 식탁 위 과일이 주변에 있는 위치에서 `o` 키를 누르십시오. 1회전하며 30°마다 측정합니다.
4. 필요한 지점은 `m` 키로 직접 측정하십시오.
5. 측정을 마치면 CSV를 열어 아래 항목을 확인하십시오.

자동 접근은 한 바퀴를 돌아도 대상을 찾지 못하면 종료합니다. 접근 중 대상을 2초 이상 놓치면 다시 탐색합니다. 측정 지점은 추정 거리 기준이며, 출발할 때 이미 지난 지점은 건너뜁니다.

측정 기록은 `controllers/hsv_tuner/output/measurements.csv`, 프레임은 `controllers/hsv_tuner/frames/`에 저장됩니다. 두 폴더는 Git에서 제외합니다. `note` 열에는 측정 사유(`manual`, `auto 2.0m d=1.99m`, `survey 30deg` 등)가 기록됩니다. 측정 1회는 다음 행으로 기록됩니다.

| `kind` | 내용 |
|---|---|
| `target` | 최종 대상 검출 1개당 1행. 거리, 방위각, 대상 월드 좌표 포함 |
| `yolo_raw` | 색 판별 전 YOLO 상자 1개당 1행. `color_ratio`는 상자 안 대상 색 비율 |
| `none` | 검출 없음 |

| 확인 항목 | CSV 열 | 조정할 설정값 |
|---|---|---|
| 색별 사과의 YOLO 클래스·신뢰도 | `yolo_raw` 행의 `cls_name`, `conf` | `YOLO_CLASSES`, `YOLO_CONF` |
| 대상 사과와 다른 색 사과의 색 비율 차이 | `yolo_raw` 행의 `color_ratio` | `COLOR_RATIO_MIN` |
| YOLO가 놓쳐 색 분할로 찾은 거리 | `target` 행의 `source`, `dist` | `MIN_BLOB_AREA` |
| 추정 거리 오차 | `target` 행의 `dist`와 실제 거리 | `TARGET_DIAMETER`, `CAMERA_FOV` |
| 대상 위치 오차 | `target` 행의 `world_x`, `world_y`와 사과 실제 좌표 | 위 항목 조정 후 재측정 |
| YOLO 추론 시간 | `yolo_ms` | `YOLO_EVERY` |

사과 실제 좌표는 [과제와 구현 기준](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/sar-과제-구현-기준.md) 5.2절에 있습니다. 로봇 위치는 엔코더 오도메트리로 추정하므로 오래 주행하면 오차가 누적됩니다. 위치 오차를 측정할 때는 Reset 직후 짧게 이동한 뒤 측정하십시오.

콘솔에는 1초마다 다음 형식의 요약이 출력됩니다. `rtf`는 시뮬레이션 시간을 실제 시간으로 나눈 실시간 배율입니다. 1보다 많이 작으면 YOLO 추론이 시뮬레이션을 늦추는 상태이므로 `YOLO_EVERY`를 늘리십시오.

```text
[t=12.0s] pose=(-1.52, -7.48, 178deg) yolo=72ms rtf=0.85 target=apple conf=0.62 d=1.23m b=-5deg at (-2.75, -7.37) raw=[apple 0.62 r=0.55]
```

## 결과 확인

- 왼쪽 화면에서 대상 사과에 상자가 표시됨. 초록 상자는 YOLO 검출, 하늘색 상자는 색 분할 검출
- 1 m, 2 m, 3 m 측정에서 추정 거리가 실제 거리와 비슷함
- 다른 색 사과, 나무 바닥, 식탁 위 과일에 대상 상자가 표시되지 않음

식탁 위 과일이 검출되면 `HORIZON_MARGIN`을 줄이십시오. 멀리 있는 사과를 놓치면 `MIN_BLOB_AREA`를 줄이십시오.

## 관련 자료

- 검출 조건과 설정값: [모듈 인터페이스](../reference/interfaces.md)
- 튜닝 컨트롤러: `controllers/hsv_tuner/hsv_tuner.py`
- HSV 초기값의 근거: [과제와 구현 기준](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/sar-과제-구현-기준.md) 11장
