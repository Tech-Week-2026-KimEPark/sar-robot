# 개발 환경 설정

팀원이 macOS 또는 Ubuntu에서 sar-robot을 clone하고 테스트와 Webots 시뮬레이션을 실행하기까지의 절차입니다. 환경은 [Intro 과정 환경 기준](../reference/intro-environment.md)과 같습니다. 2026-09-30에 macOS 26.6.2(Apple Silicon), Python 3.10.21, Webots R2025a로 확인했습니다.

## 준비

| 도구 | 버전 | 확인 명령 |
|---|---|---|
| Git, GitHub 계정 | 조직 `Tech-Week-2026-KimEPark` 멤버 | `gh auth status` |
| Python | 3.10 | `python3.10 --version` |
| Webots | R2025a | `/Applications/Webots.app/Contents/MacOS/webots --version` |

Python 3.10이 없으면 설치하십시오. Ubuntu 22.04는 시스템 Python이 3.10이므로 `python3.10-venv` 패키지만 설치하십시오.

```bash
brew install python@3.10              # macOS
sudo apt install python3.10-venv      # Ubuntu 22.04
```

## 순서

### 1. 저장소와 가상환경

```bash
git clone https://github.com/Tech-Week-2026-KimEPark/sar-robot.git
cd sar-robot
python3.10 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-dev.txt -r requirements-yolo.txt
.venv/bin/python -m pip install -e .
```

- 가상환경 경로는 저장소 루트의 `.venv`로 만드십시오.
- `requirements-yolo.txt`는 torch를 포함하며 macOS 기준 약 1 GB입니다. Linux·Windows는 [PyTorch 설치 방법](../reference/intro-environment.md#pytorch-설치-방법)의 장치별 명령을 먼저 실행하십시오.
- `pip install -e .`는 `src/sar` 패키지를 가상환경에 등록합니다. 컨트롤러는 이 등록으로 `from sar import ...`를 실행합니다.

YOLO 모델 `models/YOLO/yolo11n.pt`는 Git에 포함하지 않습니다. 다음 명령으로 내려받으십시오.

```bash
.venv/bin/python -c "from ultralytics import YOLO; YOLO('models/YOLO/yolo11n.pt')"
```

### 2. 테스트와 검사

```bash
.venv/bin/python -m pip check
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

Ubuntu는 Intro 노트북의 명령으로 설치하십시오.

```bash
wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt install ./webots_2025a_amd64.deb
```

### 4. Webots Python 설정

Webots는 Preferences의 Python command로 모든 컨트롤러를 실행합니다. 기본값 `python3`는 macOS에서 패키지가 없는 Python 3.9입니다. 저장소 `.venv`의 Python 절대 경로로 변경하십시오.

1. Webots > Preferences > General > Python command에 다음 명령의 출력값을 입력하십시오.

   ```bash
   echo "$(pwd)/.venv/bin/python"
   ```

2. Webots를 종료하고 다시 실행하십시오.

macOS에서는 터미널에서 설정할 수도 있습니다. Webots를 종료한 상태에서 저장소 루트에서 실행하십시오.

```bash
defaults write com.cyberbotics.Webots-R2025a General.pythonCommand "$(pwd)/.venv/bin/python"
```

이 설정은 Intro 실습 컨트롤러(`tb3_*`)와 `sar_mission`에 모두 적용됩니다.

### 5. 원격 에셋 사전 캐시

Webots R2025a는 월드의 텍스처·메시를 raw.githubusercontent.com에서 내려받습니다. macOS에서는 Webots에 포함된 Qt 6.5.3의 HTTP/2 처리 오류로 `error code: 399: Server is unable to maintain the header compression context` 경고가 발생하고 텍스처가 표시되지 않습니다. 월드를 열기 전에 에셋을 Webots 캐시에 저장하십시오.

```bash
.venv/bin/python scripts/prefetch_webots_assets.py
```

이 스크립트는 `worlds/webots_assets.txt`의 URL을 HTTP/1.1로 내려받습니다. 저장 위치는 Webots 캐시 폴더(macOS `~/Library/Caches/Cyberbotics/Webots/assets/`)이며 파일 이름은 URL의 SHA1 값입니다. 목록은 `sar_dev.wbt` 기준입니다. 다른 월드에서 같은 경고가 나오면 Webots 콘솔 내용을 파일로 저장해 인자로 전달하십시오.

```bash
.venv/bin/python scripts/prefetch_webots_assets.py webots_console.txt
```

### 6. 시뮬레이션 실행

1. Webots에서 File > Open World로 `worlds/sar_dev.wbt`를 여십시오. 이미 열려 있으면 File > Reload World를 선택하십시오.
2. 시뮬레이션을 시작하십시오. 로봇 컨트롤러는 `sar_mission`입니다.

Intro 실습 월드(`breakroom_teleop.wbt` 등)도 같은 방법으로 실행합니다.

## 결과 확인

- `pip check`: `No broken requirements found.`
- 테스트: `12 passed` 이상 출력
- 에셋 스크립트: `92 URLs, 0 failed` 출력
- Webots 콘솔: 1초마다 `t=1.0s pose=(0.00, 0.00, 0.00) lidar_points=360 front=...m` 형식의 로그 출력. 로봇은 정지 상태

| 증상 | 확인할 내용 |
|---|---|
| `Cannot download ... error code: 399` | `scripts/prefetch_webots_assets.py` 실행 후 Reload World |
| `ModuleNotFoundError: No module named 'sar'` | Webots Python command가 `.venv/bin/python`인지, `pip install -e .` 실행 여부 |
| `ModuleNotFoundError: No module named 'numpy'` | Webots Python command 설정 여부 |
| `Unable to find the 'python' executable` | Python command 경로 오타 여부 |
| `lidar_points=0` | 월드의 로봇 `extensionSlot`에 `RobotisLds01` 존재 여부 |

## 관련 자료

- 버전 기준: [Intro 과정 환경 기준](../reference/intro-environment.md)
- 의존성 파일: `requirements.txt`, `requirements-dev.txt`, `requirements-yolo.txt`
- 코드 규칙: [코드 작성 규칙](../reference/code-conventions.md)
- 구조 설명: [코드 구조](../explanation/architecture.md)
