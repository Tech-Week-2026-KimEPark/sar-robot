# 코드 구조

sar-robot의 폴더 구성, 모듈 사이 데이터 흐름, 구조를 선택한 이유를 설명하는 문서입니다. 함수 형식은 [모듈 인터페이스](../reference/interfaces.md)에 있습니다.

## 폴더 구성

```text
sar-robot/
├── controllers/sar_mission/   Webots 진입점(.venv 재실행, 루프)
├── scripts/                   Webots 에셋 캐시 스크립트
├── src/sar/                   팀 코드 패키지
│   ├── config.py              상수
│   ├── geometry.py            좌표 규칙, Pose, 좌표 변환
│   ├── robot_io.py            Webots 장치 접근
│   └── localization/          Wheel Odometry
├── tests/                     pytest
├── worlds/sar_dev.wbt         개발용 월드 (Intro breakroom_teleop_yolo 복사본)
├── worlds/webots_assets.txt   개발용 월드의 원격 에셋 URL 목록
├── protos/                    목표 물체 사과 PROTO 4종
└── models/YOLO/               YOLO 가중치 (Git 제외)
```

`mapping/`, `planning/`, `perception/`, `control/`, `mission/` 폴더는 기능을 구현할 때 추가합니다. 담당 역할은 [역할과 담당 범위](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/team/roles.md)에 있습니다.

## 데이터 흐름

미션 루프는 인지·계획·행동 순서로 1 step을 처리합니다.

```mermaid
flowchart LR
  IO[RobotIO] -->|바퀴 회전각| LOC[localization]
  IO -->|LiDAR 거리| MAP[mapping]
  IO -->|BGR 이미지| PER[perception]
  LOC -->|Pose| MAP
  LOC -->|Pose| MIS[mission]
  MAP -->|격자 지도| PLN[planning]
  PER -->|대상 검출 결과| MIS
  PLN -->|경로| MIS
  MIS -->|목표점| CTL[control]
  CTL -->|v, w| IO
```

현재 구현된 흐름은 `RobotIO → localization`과 로그 출력입니다. 컨트롤러는 바퀴 속도 0을 유지합니다.

## 구조 선택 이유

### 컨트롤러와 패키지 분리

Webots는 `controllers/<이름>/<이름>.py`를 실행합니다. 로직을 이 파일에 작성하면 Webots 없이 테스트할 수 없습니다. 팀 코드는 `src/sar/` 패키지에 작성하고 컨트롤러는 루프만 담당합니다. `from controller import ...`는 `robot_io.py`에만 두므로 나머지 모듈은 pytest와 CI에서 실행됩니다.

### 컨트롤러의 가상환경 재실행

Webots가 `sar` 패키지와 가상환경의 numpy·OpenCV를 사용하려면 import 경로와 Python 실행 파일을 지정해야 합니다. Webots 환경설정에서 지정하면 PC마다 절대 경로가 달라집니다. `runtime.ini`의 `[python] COMMAND`는 상대 경로를 인식하지 않습니다(Webots R2025a에서 `"../../.venv/bin/python" was not found` 확인).

그래서 `sar_mission.py`가 실행 직후 저장소 `.venv`의 Python으로 자신을 다시 실행합니다. macOS와 Linux는 `os.execv`로 프로세스를 교체하고, Windows는 자식 프로세스의 종료 코드를 전달합니다. Webots가 설정한 환경변수는 재실행한 프로세스에 그대로 전달되므로 `controller` 모듈 import와 시뮬레이터 연결이 유지됩니다. 재실행 전 코드는 macOS 기본 Python 3.9에서도 실행되도록 3.10 이상 문법을 사용하지 않습니다.

### 원격 에셋 사전 캐시

실습 월드는 Webots 기본 PROTO와 텍스처를 GitHub에서 내려받습니다. Webots R2025a의 Qt 6.5.3은 이 다운로드에서 HTTP/2 헤더 압축 오류(`error code: 399`)가 발생합니다. 같은 파일을 curl의 HTTP/2 요청으로 받으면 정상 수신되므로 서버가 아니라 Qt 구현의 문제입니다. Webots는 캐시 폴더에 URL의 SHA1 이름으로 파일이 있으면 네트워크 요청을 하지 않습니다. `scripts/prefetch_webots_assets.py`는 이 규칙에 맞춰 에셋을 HTTP/1.1로 미리 저장합니다.

### 모듈별 폴더

역할 4개가 서로 다른 폴더를 수정하므로 머지 충돌이 줄어듭니다. 모듈 사이 호출은 인터페이스 문서의 함수로 제한합니다. 따라서 담당자가 내부 구현을 바꿔도 다른 모듈을 수정하지 않습니다.

### 상수 파일 1개

속도, 임계값, 장치 이름을 `config.py` 1개에 모읍니다. 대회 월드의 장치 이름이나 로봇 크기가 다르면 이 파일만 수정합니다.

## 개발용 월드

`worlds/sar_dev.wbt`는 Intro의 `breakroom_teleop_yolo.wbt`에서 컨트롤러 이름만 `sar_mission`으로 바꾼 월드입니다. TurtleBot3 Burger에 카메라(640×480, 시야각 1.0472 rad)와 LDS-01 LiDAR가 장착되어 있고 사과 4종이 배치되어 있습니다. 대회 월드는 당일 제공되므로 이 월드는 개발·회귀 테스트용입니다.
