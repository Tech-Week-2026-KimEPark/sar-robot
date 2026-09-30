# HSV 임계값 튜닝

이 문서는 인식 담당이 Webots 월드에서 대상 사과의 HSV 범위를 조정하고 `config.py`에 반영하는 절차입니다. 대회 월드의 조명이 연습 월드와 다르면 이 절차를 수행하십시오.

## 준비

- [개발 환경 설정](dev-setup.md)을 마치고 Webots Python command를 저장소 `.venv`의 Python으로 설정함
- YOLO 가중치 `models/YOLO/yolo11n.pt`가 있음. 없으면 색 분할만 사용함
- `config.py`의 `TARGET_COLOR`가 튜닝할 색임

## 순서

1. Webots에서 월드를 여십시오.
2. 로봇 노드의 `controller` 필드를 `hsv_tuner`로 바꾸고 시뮬레이션을 시작하십시오.
3. `hsv_tuner` 창을 클릭해 포커스를 옮기십시오. 키 입력은 이 창에서만 동작합니다.
4. 키보드로 로봇을 움직여 대상 사과와 다른 색 사과가 화면에 보이게 하십시오.
5. 트랙바를 조정해 오른쪽 마스크 화면에 대상 사과만 흰색으로 남게 하십시오.
6. `r` 키를 눌러 콘솔에 출력된 범위를 `config.py`의 `HSV_RANGES`에 붙여 넣으십시오.
7. 사과까지 거리 1 m, 2 m, 3 m에서 `p` 키로 프레임을 저장하십시오.
8. 튜닝을 마치면 로봇의 `controller` 필드를 원래 값으로 되돌리십시오.

| 키 | 동작 |
|---|---|
| `w`, `s` | 전진, 후진 |
| `a`, `d` | 좌회전, 우회전 |
| `space` | 정지 |
| `p` | 원본 프레임 저장 (`controllers/hsv_tuner/frames/`) |
| `r` | 현재 범위를 `HSV_RANGES` 형식으로 출력 |
| `q` | 종료 |

H 하한이 상한보다 크면 0을 넘는 범위로 처리합니다. 빨강의 초기값은 H 하한 170, 상한 8입니다. 이 값은 (0~8) ∪ (170~179) 범위입니다.

## 결과 확인

- 왼쪽 화면에서 대상 사과에 상자가 표시됨. 초록 상자는 YOLO 검출, 하늘색 상자는 색 분할 검출
- 상자 위 거리 값이 실제 거리와 비슷함
- 다른 색 사과, 나무 바닥, 식탁 위 과일에 상자가 표시되지 않음

거리 값이 크게 다르면 `TARGET_DIAMETER`와 `CAMERA_FOV`를 확인하십시오. 식탁 위 과일이 검출되면 `HORIZON_MARGIN`을 줄이십시오. 멀리 있는 사과를 놓치면 `MIN_BLOB_AREA`를 줄이십시오.

## 관련 자료

- 검출 조건과 설정값: [모듈 인터페이스](../reference/interfaces.md)
- 튜닝 컨트롤러: `controllers/hsv_tuner/hsv_tuner.py`
- HSV 초기값의 근거: [과제와 구현 기준](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/sar-과제-구현-기준.md) 11장
