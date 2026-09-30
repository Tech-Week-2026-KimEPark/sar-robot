# Intro 과정 환경 기준

sar-robot 개발 환경의 기준인 Intro 과정 환경을 정리한 문서입니다. 원본은 Intro 저장소의 실습 노트북 `TECH-WEEK-26_Physical-AI.ipynb`(강사 원본, blob `17e823f`)와 과정 안내 이미지입니다. 2026-09-30에 확인했습니다.

sar-robot은 Intro의 패키지 버전, 로봇 상수, 실습 파일을 그대로 사용합니다. Intro에 버전이 없는 패키지는 Intro 고정 버전과 호환되는 버전을 확인해 고정했습니다.

## 실행 환경

| 항목 | Intro 기준 | sar-robot 적용 값 | 근거 |
|---|---|---|---|
| OS | Ubuntu 22.04. Windows·macOS 사용 가능 | macOS 26.6.2(Apple Silicon)에서 확인 | 과정 안내 이미지 |
| Python | 3.10.12 | 3.10 (macOS Homebrew 3.10.21) | 노트북 kernel 정보 |
| 시뮬레이터 | Webots R2025a | Webots R2025a | 노트북 5장, PROTO 파일 헤더 |
| GPU | 선택 사항 | 선택 사항. macOS는 MPS 사용 가능 | 과정 안내 이미지 |

Python은 3.10.x 버전이 같으면 됩니다. 3.10.12와 3.10.21의 차이는 패치 번호뿐입니다.

## 패키지 버전

| 패키지 | Intro 기준 | sar-robot 고정 값 | 설치 파일 |
|---|---|---|---|
| `numpy` | 1.23.5 | 1.23.5 | `requirements.txt` |
| `opencv-python` | 4.8.0.74 | 4.8.0.74 | `requirements.txt` |
| `scikit-image` | 0.19.3 | 0.19.3 | `requirements.txt` |
| `matplotlib` | 3.7.5 | 3.7.5 | `requirements.txt` |
| `scipy` | 버전 없음 (`cKDTree` 사용) | 1.15.3 | `requirements.txt` |
| `requests` | 버전 없음 | 2.34.2 | `requirements.txt` |
| `torch` | 2.8.0 | 2.8.0 | `requirements-yolo.txt` |
| `torchvision` | 0.23.0 | 0.23.0 | `requirements-yolo.txt` |
| `ultralytics` | 버전 없음 | 8.4.166 | `requirements-yolo.txt` |

scipy 1.15.3은 Python 3.10을 지원하는 마지막 계열입니다. ultralytics 8.4.166은 numpy 1.23.5와 함께 설치해 `pip check` 오류가 없음을 확인했습니다.

### PyTorch 설치 방법

노트북은 연산 장치에 따라 PyTorch 설치 명령을 선택하도록 안내합니다.

| 장치 | 설치 명령 |
|---|---|
| macOS (Apple Silicon) | `pip install -r requirements-yolo.txt` |
| Linux·Windows CPU | `pip install torch==2.8.0 torchvision==0.23.0 --index-url https://download.pytorch.org/whl/cpu` 후 `pip install -r requirements-yolo.txt` |
| NVIDIA GPU | `pip install torch==2.8.0 torchvision==0.23.0 --index-url https://download.pytorch.org/whl/<CUDA_version>` 후 `pip install -r requirements-yolo.txt` |

Linux에서 PyPI 기본 torch를 설치하면 CUDA 포함 버전이 설치되어 용량이 커집니다. GPU가 없으면 CPU 명령을 사용하십시오.

## YOLO 모델

| 항목 | 값 |
|---|---|
| 모델 | YOLO11n (`yolo11n.pt`, 5.4 MB) |
| 경로 | `models/YOLO/yolo11n.pt` (Git 제외) |
| 준비 방법 | `YOLO("models/YOLO/yolo11n.pt")` 최초 호출 시 자동 다운로드 |
| 학습 데이터 | COCO 80개 클래스. 사과는 47번 |

노트북이 YOLO11n을 선택한 이유는 라이브러리 의존성 안정성, 실시간 동작, 빠른 로딩입니다. 연산 장치는 다음과 같이 지정합니다.

```python
model.to("cuda")  # NVIDIA GPU
model.to("mps")  # Apple Silicon GPU
model.to("cpu")  # CPU
```

## 로봇 상수

| 이름 | 값 | sar-robot 위치 |
|---|---|---|
| `WHEEL_RADIUS` | 0.033 m | `controllers/sar_main/sar/config.py` |
| `WHEEL_SEPARATION` | 0.160 m | `controllers/sar_main/sar/config.py` |
| `ROBOT_RADIUS` | 0.105 m | `controllers/sar_main/sar/config.py` |

`WHEEL_RADIUS`와 `WHEEL_SEPARATION`은 Webots R2025a `TurtleBot3Burger.proto`의 바퀴 반지름과 바퀴 anchor 값과 일치합니다.

## 실습 파일

Intro의 실습 파일을 sar-robot에 복사했습니다. 복사한 컨트롤러는 Intro 원본과 동일하게 유지하며 ruff 검사에서 제외합니다.

| 분류 | 파일 | 용도 |
|---|---|---|
| 컨트롤러 | `tb3_teleop` | 키보드 주행 |
| 컨트롤러 | `tb3_lidar` | LiDAR 거리 출력 |
| 컨트롤러 | `tb3_teleop_sensors` | LiDAR, Encoder, 가속도계, 자이로, 나침반, 카메라 값 확인 |
| 컨트롤러 | `tb3_ground_truth` | Supervisor 기반 실제 위치 확인 |
| 컨트롤러 | `tb3_cam` | 카메라 이미지 확인 |
| 컨트롤러 | `tb3_segmentation` | LAB 색상 분할 |
| 컨트롤러 | `tb3_teleop_cam` | 키보드 주행 중 색상 분할 |
| 컨트롤러 | `tb3_teleop_yolo` | 키보드 주행 중 YOLO 탐지 |
| 월드 | `apartment.wbt`, `breakroom_*.wbt` 5개, `empty.wbt` | 실습 월드. 숨김 파일 `.*.wbproj`는 Webots 화면 상태 |
| PROTO | `RedApple`, `GreenApple`, `OrangeApple`, `PurpleApple` | 목표 물체 |

`sar_main` 컨트롤러와 `worlds/sar_dev.wbt`는 sar-robot에서 추가한 파일입니다.

`tests/test_intro_sync.py`는 Intro와 sar-robot의 Git 인덱스 blob 해시를 비교합니다. Intro 저장소가 형제 폴더 `../PNU-TECHWEEK-260930`에 있으면 `pytest`에서 누락·변경 파일을 검출합니다. 2026-09-30 기준 Intro 추적 파일 27개가 모두 동일합니다.

Webots는 월드를 열 때 `worlds/.*.wbproj`(창 배치 상태)를 자동으로 수정합니다. 이 변경은 커밋하지 말고 `git checkout -- worlds/.*.wbproj`로 되돌리십시오.

Intro 실습 파일이 갱신되면 저장소 루트에서 다음 명령으로 차이를 확인한 뒤 다시 복사하십시오.

```bash
diff -r -x .DS_Store ../PNU-TECHWEEK-260930/controllers controllers
diff -r -x .DS_Store ../PNU-TECHWEEK-260930/worlds worlds
```

## Intro와 다르게 적용한 항목

| 항목 | Intro | sar-robot | 이유 |
|---|---|---|---|
| 패키지 설치 위치 | 시스템 Python에 `pip install` | 저장소 `.venv` | 팀원 PC마다 같은 버전 유지 |
| Webots Python | 시스템 `python3` | Preferences의 Python command를 `.venv/bin/python`으로 설정 | macOS 기본 `python3`는 3.9이며 패키지 없음 |
| 팀 코드 import | 없음 | 컨트롤러 폴더 안 `sar/` 패키지. 설치 없음 | 컨트롤러 폴더 1개로 제출·실행 (`CONTEXT.md` 구조) |
| Webots 원격 PROTO·에셋 | 자동 다운로드 | `scripts/prefetch_webots_assets.py`로 사전 캐시 | macOS Webots R2025a의 다운로드 오류(`error code: 399`, `error code: 2`) |

설치 절차는 [개발 환경 설정](../how-to/dev-setup.md)에 있습니다.
