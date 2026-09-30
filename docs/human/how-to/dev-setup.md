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

가상환경 경로는 반드시 저장소 루트의 `.venv`로 만드십시오. Webots 컨트롤러가 이 경로의 Python으로 재실행합니다.

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

Webots R2025a는 월드의 텍스처·메시를 raw.githubusercontent.com에서 내려받습니다. 이때 Webots에 포함된 Qt 6.5.3의 HTTP/2 처리 오류로 `error code: 399: Server is unable to maintain the header compression context` 경고가 발생하고 텍스처가 표시되지 않습니다. 월드를 열기 전에 에셋을 Webots 캐시에 저장하십시오.

```bash
.venv/bin/python scripts/prefetch_webots_assets.py
```

이 스크립트는 `worlds/webots_assets.txt`의 URL을 HTTP/1.1로 내려받습니다. 저장 위치는 Webots 캐시 폴더(macOS `~/Library/Caches/Cyberbotics/Webots/assets/`)이며 파일 이름은 URL의 SHA1 값입니다. 다른 월드에서 같은 경고가 나오면 Webots 콘솔 내용을 파일로 저장해 인자로 전달하십시오. 콘솔 로그의 `Cannot download` URL을 추출해 캐시에 저장합니다.

```bash
.venv/bin/python scripts/prefetch_webots_assets.py webots_console.txt
```

에셋 저장 후 다음 순서로 실행하십시오.

1. Webots를 실행하고 File > Open World에서 `worlds/sar_dev.wbt`를 여십시오. 이미 열려 있으면 File > Reload World를 선택하십시오.
2. 시뮬레이션을 시작하십시오. 로봇 컨트롤러는 `sar_mission`입니다.

Webots는 환경설정의 Python command(기본 `python3`)로 컨트롤러를 실행합니다. `sar_mission.py`는 실행 직후 저장소 `.venv`의 Python으로 자신을 다시 실행하고 `src/`를 import 경로에 추가합니다. 따라서 Webots 환경설정은 변경하지 않아도 됩니다. macOS에서 Finder로 실행한 Webots는 셸의 PATH를 사용하지 않으므로 `/usr/bin/python3`(Xcode Command Line Tools)를 사용합니다.

## 결과 확인

- 테스트: `12 passed` 이상 출력
- 에셋 스크립트: `92 URLs, 0 failed` 출력
- Webots 콘솔: 1초마다 `t=1.0s pose=(0.00, 0.00, 0.00) lidar_points=360 front=...m` 형식의 로그 출력. 로봇은 정지 상태

| 증상 | 확인할 내용 |
|---|---|
| `Cannot download ... error code: 399` | `scripts/prefetch_webots_assets.py` 실행 후 Reload World |
| `.venv/bin/python 없음` | 저장소 루트에서 가상환경 생성 절차 실행 |
| `Unable to find the 'python' executable` | Webots > Preferences > Python command에 `.venv/bin/python`의 절대 경로 입력 |
| `ModuleNotFoundError: No module named 'numpy'` | `requirements.txt` 설치 여부 |
| `lidar_points=0` | 월드의 로봇 `extensionSlot`에 `RobotisLds01` 존재 여부 |

## 관련 자료

- 의존성 버전: `requirements.txt`, `requirements-dev.txt`, `requirements-yolo.txt`
- 코드 규칙: [코드 작성 규칙](../reference/code-conventions.md)
- 구조 설명: [코드 구조](../explanation/architecture.md)
