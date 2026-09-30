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
```

- 가상환경 경로는 저장소 루트의 `.venv`로 만드십시오.
- `requirements-yolo.txt`는 torch를 포함하며 macOS 기준 약 1 GB입니다. Linux·Windows는 [PyTorch 설치 방법](../reference/intro-environment.md#pytorch-설치-방법)의 장치별 명령을 먼저 실행하십시오.
- 팀 코드는 `controllers/sar_main/sar/`에 있으며 설치하지 않습니다. Webots가 컨트롤러 폴더를 import 경로에 포함합니다.
- 이전 구조에서 `pip install -e .`를 실행했다면 `.venv/bin/python -m pip uninstall -y sar-robot`로 제거하십시오.

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

이 설정은 Intro 실습 컨트롤러(`tb3_*`)와 `sar_main`에 모두 적용됩니다.

### 5. 원격 에셋 사전 캐시

Webots R2025a는 월드의 PROTO·텍스처·메시를 raw.githubusercontent.com에서 내려받습니다. macOS에서는 Webots에 포함된 Qt 6.5.3의 네트워크 처리 오류로 다운로드가 실패합니다.

| 콘솔 메시지 | 결과 |
|---|---|
| `Error downloading EXTERNPROTO ... error code: 2: Connection closed` | 해당 PROTO 노드 누락(`Skipped unknown ... node`) |
| `Cannot download ... error code: 399: Server is unable to maintain the header compression context` | 텍스처·메시 미표시 |

월드를 열기 전에 저장소 루트에서 다음 명령으로 Webots 캐시를 채우십시오.

```bash
.venv/bin/python scripts/prefetch_webots_assets.py
```

이 스크립트는 `worlds/*.wbt` 7개의 EXTERNPROTO를 재귀로 따라가며 PROTO와 에셋 URL을 수집합니다. 템플릿 변수로 이름을 만드는 텍스처는 GitHub 폴더 목록에서 이름 패턴이 맞는 파일을 모두 수집합니다. 수집한 파일은 HTTP/1.1로 내려받아 Webots 캐시 폴더(macOS `~/Library/Caches/Cyberbotics/Webots/assets/`)에 URL의 SHA1 이름으로 저장합니다. 2026-09-30 기준 619개, 약 140 MB입니다.

- GitHub 폴더 목록 조회에는 `GITHUB_TOKEN` 환경변수 또는 `gh auth token`의 토큰을 사용합니다. 토큰이 없으면 시간당 60회 제한에 걸릴 수 있습니다.
- 이미 캐시에 있는 파일과 폴더 목록은 다시 내려받지 않습니다.
- 대회 당일 새 월드를 받으면 해당 `.wbt` 파일을 인자로 실행하십시오.

```bash
.venv/bin/python scripts/prefetch_webots_assets.py path/to/contest.wbt
```

그래도 경고가 남으면 Webots 콘솔 내용을 파일로 저장해 인자로 전달하십시오. 로그의 URL을 추출해 캐시에 저장합니다.

```bash
.venv/bin/python scripts/prefetch_webots_assets.py webots_console.txt
```

### 6. 시뮬레이션 실행

1. Webots에서 File > Open World로 `worlds/sar_apartment.wbt`를 여십시오. 이미 열려 있으면 File > Reload World를 선택하십시오.
2. 시뮬레이션을 시작하십시오. 로봇 컨트롤러는 `sar_main`입니다.

`sar_apartment.wbt`는 대회 연습 맵 `apartment.wbt`에서 로봇 컨트롤러만 `sar_main`으로 바꾼 검증 월드입니다. 동작 검증은 이 월드로만 수행하십시오. `sar_dev.wbt`는 시작 위치가 `config.py`의 `START_*`와 다르고 빨간 사과가 1개뿐이라 과제 검증에 맞지 않습니다.

GUI 없이 검증하려면 저장소 루트에서 다음 명령을 실행하십시오. 직접 실행한 프로세스만 PID로 종료하십시오.

```bash
/Applications/Webots.app/Contents/MacOS/webots --batch --mode=fast --minimize --stdout --stderr --port=1296 worlds/sar_apartment.wbt > webots.log 2>&1 &
PID=$!
# webots.log에서 "-> DONE" 확인 후
kill $PID
```

Intro 실습 월드(`breakroom_teleop.wbt` 등)도 같은 방법으로 실행합니다.

### 7. 설계 근거 측정

설계 선택을 수치로 비교할 때는 측정 월드 `worlds/sar_apartment_eval.wbt`를 사용하십시오. 로봇 컨트롤러 `sar_eval`은 `sar_main`과 같은 `Mission`을 실행합니다. 매 step 센서 값과 Supervisor 실제 pose를 `controllers/sar_eval/output/<모드>/run.npz`에 기록합니다. 인식 step 카메라 화면은 `frames/`에 저장합니다. 실행은 `DONE` 1초 후 자동 종료됩니다.

| `SAR_EVAL_MODE` | 조건 |
|---|---|
| `mission` (기본) | 현재 코드. 실제 pose는 기록에만 사용 |
| `truth` | 오도메트리 대신 실제 pose를 미션에 입력. 위치 추정 오차 제거 조건 |
| `explore`, `truth_explore` | 위 조건 + `TARGET_COUNT = 99`. 프론티어 소진까지 탐색 |

`SAR_EVAL_SET`으로 설정값을 바꿔 실행할 수 있습니다. 예는 `SAR_EVAL_SET=CONFIRM_FRAMES=3`입니다. 결과 폴더 이름에 설정값이 추가됩니다.

```bash
SAR_EVAL_MODE=truth /Applications/Webots.app/Contents/MacOS/webots --batch --mode=fast --minimize --stdout --stderr --port=1297 worlds/sar_apartment_eval.wbt > eval.log 2>&1
.venv/bin/python scripts/design_eval.py            # 전체 실험. --only L,P2 처럼 일부만 실행 가능
```

`scripts/design_eval.py`는 기록을 다시 계산해 조건별 지표를 `controllers/sar_eval/output/design/results.json`에 저장합니다. 그래프는 같은 폴더에 `design_*.png`로 저장합니다. 실험 목록과 결과는 docs [SAR 로봇 시스템 설계서](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/explanation/sar-시스템-설계.md)에 있습니다.

## 결과 확인

- `pip check`: `No broken requirements found.`
- 테스트: `17 passed` 이상 출력. Intro 저장소가 형제 폴더에 있으면 Intro 실습 파일 동기화 검사 포함
- 에셋 스크립트: `619 URLs, ... 0 failed` 출력
- 모듈 단독 테스트: `controllers/sar_main`에서 `../../.venv/bin/python -m sar.mission` 실행 시 `mission self-test ok` 출력
- Webots 콘솔: 시작 회전 후 `나침반 보정 sign=... offset=...` 로그와 `[t=8.8s] INIT_SPIN -> EXPLORE (시작 회전 완료)` 형식의 상태 전환 로그 출력. 1초마다 `[t=...] <상태> pose=(x, y, theta) 구조 n/2 후보 k` 상태 로그 출력
- 지도 그림: `controllers/sar_main/output/`에 `map_latest.png`(5초 주기), `map_rescue_<n>.png`(구조 시점), `map_final.png`(종료 시점) 저장

| 증상 | 확인할 내용 |
|---|---|
| `Cannot download ... error code: 399` | `scripts/prefetch_webots_assets.py` 실행 후 Reload World |
| `Error downloading EXTERNPROTO ... Connection closed` | `scripts/prefetch_webots_assets.py` 실행 후 Reload World |
| `list failed ... rate limit exceeded` | `gh auth login` 또는 `GITHUB_TOKEN` 설정 후 재실행 |
| `git status`에 `worlds/.*.wbproj` 변경 표시 | Webots 창 배치 자동 저장. `git checkout -- worlds/.*.wbproj`로 되돌린 뒤 커밋 |
| `ModuleNotFoundError: No module named 'sar'` | 월드 로봇의 `controller` 필드가 `sar_main`인지, `controllers/sar_main/sar/` 폴더 존재 여부 |
| `ModuleNotFoundError: No module named 'numpy'` | Webots Python command 설정 여부 |
| `Unable to find the 'python' executable` | Python command 경로 오타 여부 |
| `lidar_points=0` | 월드의 로봇 `extensionSlot`에 `RobotisLds01` 존재 여부 |

## 관련 자료

- 버전 기준: [Intro 과정 환경 기준](../reference/intro-environment.md)
- 의존성 파일: `requirements.txt`, `requirements-dev.txt`, `requirements-yolo.txt`
- 코드 규칙: [코드 작성 규칙](../reference/code-conventions.md)
- 구조 설명: [코드 구조](../explanation/architecture.md)
