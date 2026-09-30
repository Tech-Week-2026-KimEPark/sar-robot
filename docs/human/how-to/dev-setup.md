# 개발 환경 설정

팀원이 macOS 또는 Ubuntu에서 sar-robot을 clone하고 테스트와 Webots 시뮬레이션을 실행하기까지의 절차입니다. 2026-09-30에 macOS 26.6.2(Apple Silicon), Python 3.12.7, Webots R2025a로 확인했습니다.

## 준비

| 도구 | 버전 | 확인 명령 |
|---|---|---|
| Git, GitHub 계정 | 조직 `Tech-Week-2026-KimEPark` 멤버 | `gh auth status` |
| Python | 3.12 | `python3.12 --version` |
| Webots | R2025a | `/Applications/Webots.app/Contents/MacOS/webots --version` |

macOS에서 Python 3.12가 없으면 `brew install python@3.12`로 설치하십시오. Ubuntu 22.04는 deadsnakes PPA로 설치하십시오.

## 순서

### 1. 저장소와 가상환경

```bash
git clone https://github.com/Tech-Week-2026-KimEPark/sar-robot.git
cd sar-robot
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements-dev.txt
```

가상환경 경로는 반드시 저장소 루트의 `.venv`로 만드십시오. Webots 컨트롤러 설정(`runtime.ini`)이 이 경로를 사용합니다.

YOLO를 사용하는 작업에는 추가 의존성을 설치하십시오. torch를 포함해 약 1 GB입니다.

```bash
.venv/bin/python -m pip install -r requirements-yolo.txt
```

`models/YOLO/yolo11n.pt`는 Git에 포함하지 않습니다. Intro 저장소 안내에 따라 이 폴더에 준비하십시오.

### 2. 테스트와 검사

```bash
.venv/bin/python -m pytest -q
.venv/bin/ruff check .
.venv/bin/ruff format --check .
```

### 3. Webots 설치

Homebrew의 `webots` cask는 2026-09-01부터 Gatekeeper 문제로 비활성화되었습니다. GitHub 릴리스의 dmg로 설치하십시오.

```bash
curl -fLO https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a.dmg
hdiutil attach -nobrowse webots-R2025a.dmg
cp -R /Volumes/Webots/Webots.app /Applications/
hdiutil detach /Volumes/Webots
```

Webots.app은 Apple 공증을 받지 않은 앱입니다. Finder에서 처음 실행하면 macOS가 실행을 차단합니다. 시스템 설정 > 개인정보 보호 및 보안에서 "그래도 열기"를 선택하십시오.

Ubuntu는 [Webots 설치 안내](https://cyberbotics.com/doc/guide/installation-procedure)의 R2025a deb 패키지를 사용하십시오.

### 4. 시뮬레이션 실행

1. Webots를 실행하고 File > Open World에서 `worlds/sar_dev.wbt`를 여십시오.
2. 첫 실행 시 월드의 외부 PROTO와 텍스처를 인터넷에서 내려받습니다. 완료까지 수 분이 걸릴 수 있습니다.
3. 시뮬레이션을 시작하십시오. 로봇 컨트롤러는 `sar_mission`입니다.

`controllers/sar_mission/runtime.ini`는 다음 2가지를 설정합니다. Webots 환경설정의 Python command는 변경하지 않아도 됩니다.

| 설정 | 값 | 목적 |
|---|---|---|
| `PYTHONPATH` | `../../src` | `sar` 패키지 import |
| `[python] COMMAND` | `../../.venv/bin/python` | 가상환경의 numpy·OpenCV 사용 |

## 결과 확인

- 테스트: `10 passed` 이상 출력
- Webots 콘솔: 1초마다 `t=1.0s pose=(0.00, 0.00, 0.00) lidar_points=360 front=...m` 형식의 로그 출력. 로봇은 정지 상태

Webots 콘솔 로그는 작성 시점에 확인하지 못했습니다. 처음 확인한 팀원이 이 문서의 확인 결과를 갱신하십시오.

| 증상 | 확인할 내용 |
|---|---|
| `ModuleNotFoundError: No module named 'sar'` | `runtime.ini`가 컨트롤러 폴더에 있는지, `src/sar/__init__.py` 존재 여부 |
| `ModuleNotFoundError: No module named 'numpy'` | 저장소 루트 `.venv` 존재 여부, `requirements.txt` 설치 여부 |
| 월드 로딩이 멈춤 | 인터넷 연결, Webots 콘솔의 PROTO 다운로드 오류 |
| `lidar_points=0` | 월드의 로봇 `extensionSlot`에 `RobotisLds01` 존재 여부 |

## 관련 자료

- 의존성 버전: `requirements.txt`, `requirements-dev.txt`, `requirements-yolo.txt`
- 코드 규칙: [코드 작성 규칙](../reference/code-conventions.md)
- 구조 설명: [코드 구조](../explanation/architecture.md)
