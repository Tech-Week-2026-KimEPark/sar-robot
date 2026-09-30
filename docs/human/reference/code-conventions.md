# 코드 작성 규칙

sar-robot 저장소의 Python 코드 작성 규칙입니다. 규칙 중 자동 검사가 가능한 항목은 ruff와 CI로 확인합니다. Git 브랜치와 커밋 규칙은 [Git 작업 규칙](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/team/git-workflow.md)에 있습니다.

## 자동 검사

| 항목 | 설정 | 확인 명령 |
|---|---|---|
| Python 버전 | 3.10 (`pyproject.toml`의 `requires-python`, [Intro 과정 환경 기준](intro-environment.md)) | `.venv/bin/python --version` |
| 코드 형식 | ruff format, 줄 길이 100 | `.venv/bin/ruff format --check .` |
| 린트 | ruff 규칙 `E`, `F`, `I`, `B`, `UP`. Intro 원본 `controllers/tb3_*` 제외 | `.venv/bin/ruff check .` |
| 테스트 | pytest, `controllers/sar_main`을 import 경로에 추가 | `.venv/bin/python -m pytest -q` |

CI(`.github/workflows/ci.yml`)는 PR과 `main` push마다 위 3개 검사를 실행합니다. 검사에 실패한 PR은 머지하지 않습니다.

## 폴더와 import 규칙

| 위치 | 작성 내용 |
|---|---|
| `controllers/sar_main/sar_main.py` | Webots 진입점. `RobotIO` 생성과 미션 루프만 작성 |
| `controllers/sar_main/sar/<모듈>.py` | 알고리즘과 로직. 기능 1개당 파일 1개. Webots 없이 실행 가능해야 함 |
| `controllers/sar_main/sar/robot_io.py` | Webots 장치 접근. `controller` 모듈 import는 이 파일에만 허용 |
| `controllers/sar_main/sar/config.py` | 모든 설정값 |
| `controllers/tb3_*/` | Intro 원본 실습 컨트롤러. 수정하지 않음 |
| `scripts/` | 개발 보조 스크립트. 테스트는 `tests/`에 작성 |
| `tests/test_<모듈>.py` | 모듈별 pytest |

- 파일 구조와 모듈 목록은 CONTEXT.md 기준입니다. 원본은 docs [과제와 구현 기준](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/sar-과제-구현-기준.md) 7장입니다.
- `sar/` 모듈에서 `from controller import ...`를 사용하지 않음. `robot_io.py`도 `RobotIO` 생성 시점에 import함. 테스트와 CI에 Webots가 없기 때문임
- 모듈 사이 호출은 [모듈 인터페이스](interfaces.md)에 정의한 함수만 사용함
- 새 패키지 의존성은 Intro 고정 버전(numpy 1.23.5 등)과 함께 설치해 `pip check` 통과를 확인한 뒤 버전을 고정함
- Intro 실습 컨트롤러를 변경해야 하면 `tb3_*`를 수정하지 말고 새 컨트롤러 폴더로 복사해 작업함

## 이름 규칙

| 대상 | 형식 | 예시 |
|---|---|---|
| 모듈·함수·변수 | `snake_case` | `to_cell`, `wheel_speeds` |
| 클래스 | `PascalCase` | `Odometry`, `RobotIO` |
| 상수 | `UPPER_SNAKE_CASE` | `MAP_RES` |
| 모듈 내부 전용 | 앞에 `_` | `_STRAIGHT_EPS` |

단위가 필요한 값은 주석이나 이름에 단위를 적으십시오. 기본 단위는 m, rad, s, m/s, rad/s입니다. 도(°) 단위는 로그 출력에만 사용합니다.

```python
MAP_RES = 0.05  # m
timeout_s = 3.0
```

## 상수 규칙

- 속도, 거리 임계값, 색 임계값, 주기는 `config.py`에 정의함
- 함수 안에 숫자를 직접 쓰지 않음. 0, 1, 2와 수식의 계수는 예외
- 튜닝한 값은 커밋 메시지 본문에 측정 조건과 결과를 적음

## 함수 작성 규칙

- 공개 함수에는 타입 힌트와 1줄 docstring을 작성함
- 결과가 없을 수 있는 함수는 `None`을 반환하고 반환 타입에 `| None`을 명시함
- 호출하는 쪽은 `None`을 받았을 때 예외 없이 다음 동작을 정함
- `while` 루프에는 종료 조건이나 타임아웃을 둠
- 각도 차이는 `atan2(sin, cos)`로 [-π, π] 범위로 정리한 뒤 비교함 (`odometry.wrap()`)

## 출력과 로그

- `print`는 `sar_main.py`에서만 사용함. `sar/` 모듈은 값을 반환하고 출력하지 않음. 단독 테스트의 결과 출력은 예외
- 주기적인 로그는 1초 간격 이하로 제한함. 매 step 출력은 시뮬레이션 속도를 떨어뜨림

## 테스트 규칙

- 새 공개 함수에는 테스트를 1개 이상 작성함
- 모든 `sar/` 모듈에 Webots 없이 실행되는 `if __name__ == "__main__":` 단독 테스트를 포함함. `controllers/sar_main`에서 `../../.venv/bin/python -m sar.<모듈>`로 실행
- 수식은 손으로 계산 가능한 값(직진, 90° 회전 등)으로 확인함
- 경계 조건(빈 지도, 경로 없음, 대상 미검출)을 테스트함
- 테스트에서 Webots와 카메라 장치를 사용하지 않음. 필요한 입력은 numpy 배열로 생성함

## 주석과 문서

주석과 docstring은 한국어 공학 문서체로 작성합니다. 코드 동작을 바꾸면 관련 문서를 같은 PR에서 갱신하십시오. 문서 위치 규칙은 저장소 루트의 `AGENTS.md`에 있습니다.
