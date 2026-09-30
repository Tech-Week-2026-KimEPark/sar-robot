# sar-robot 작업 규칙

## 코드 작업

2026 부산대 TECH WEEK Autonomous Search and Rescue 해커톤 팀 코드입니다. Webots R2025a와 Python 3.10을 사용합니다. 패키지 버전은 Intro 과정 기준입니다([Intro 과정 환경 기준](docs/human/reference/intro-environment.md)).

| 작업 전 확인 | 문서 |
|---|---|
| 코드 규칙, 폴더·import 규칙 | [코드 작성 규칙](docs/human/reference/code-conventions.md) |
| 좌표·단위, 모듈 함수 형식 | [모듈 인터페이스](docs/human/reference/interfaces.md) |
| 폴더 구성과 데이터 흐름 | [코드 구조](docs/human/explanation/architecture.md) |
| 브랜치·커밋·PR 규칙 | [Git 작업 규칙](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/team/git-workflow.md) |
| 대회 형식·채점 기준 | 저장소 루트 `CONTEXT.md` (대회 당일 작성) |

- 파일 구조와 모듈 인터페이스는 `CONTEXT.md` 기준입니다. 원본은 docs [과제와 구현 기준](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/sar-과제-구현-기준.md) 7장입니다.
- 팀 코드는 `controllers/sar_main/sar/`에 작성하십시오. Webots `controller` 모듈 import는 `sar/robot_io.py`에만 작성합니다.
- 상수는 `controllers/sar_main/sar/config.py`에 정의하십시오.
- `sar/` 모듈마다 `if __name__ == "__main__":` 단독 테스트를 작성하십시오.
- Webots 동작 검증은 `worlds/sar_apartment.wbt`로만 수행하십시오. 이 월드는 대회 연습 맵 `apartment.wbt`에 `sar_main` 컨트롤러를 지정한 복사본입니다. `sar_dev.wbt` 등 다른 월드의 결과는 검증 결과로 보고하지 마십시오.
- Webots를 헤드리스로 실행했다면 직접 실행한 프로세스만 PID로 종료하십시오. `pkill -f webots`처럼 이름으로 종료하면 다른 팀원·세션의 Webots도 종료됩니다.
- `controllers/tb3_*`와 `requirements*.txt`의 Intro 기준 버전은 변경하지 마십시오.
- 브랜치 이름은 `<type>/<내용>` 형식입니다. AI 도구 이름을 넣지 마십시오.
- 작업을 마치기 전에 저장소 루트에서 다음 명령을 실행하십시오.

```bash
.venv/bin/ruff format .
.venv/bin/ruff check .
.venv/bin/python -m pytest -q
```

Webots 헤드리스 검증은 저장소 루트에서 다음 순서로 실행하십시오. macOS 경로 기준입니다. 절차와 결과 확인은 [개발 환경 설정](docs/human/how-to/dev-setup.md) 6장에 있습니다.

```bash
/Applications/Webots.app/Contents/MacOS/webots --batch --mode=fast --minimize --stdout --stderr --port=1296 worlds/sar_apartment.wbt > webots.log 2>&1 &
PID=$!
# 로그에서 "-> DONE" 확인 후 직접 실행한 프로세스만 종료
kill $PID
```

<!-- devdog-docs:begin docs-layout -->
## 문서 위치

| 내용 | 위치 |
|---|---|
| 사람용 설명 | `docs/human/` |
| AI 작업 안내 | `docs/ai/` |
| 문서 작성 기준 | `../docs/human/how-to/write-docs.md` |

사람용 설명과 AI 안내에 같은 제품 규칙을 복사하지 마십시오. AI 안내는 사람용 원본 문서로 연결합니다.
<!-- devdog-docs:end docs-layout -->

<!-- devdog-docs:begin docs-update -->
## 기능 변경 시 문서 갱신

기능 추가·수정 작업의 완료 조건에는 관련 문서 갱신이 포함입니다. 별도 요청을 기다리지 말고 같은 작업에서 수행하십시오.

1. 작업 전에 `../docs/human/how-to/write-docs.md`의 작성 기준과 원본 위치를 확인하십시오. 형제 문서 저장소가 없을 때의 기준은 [문서 작성 안내](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/how-to/write-docs.md)입니다.
2. 변경한 API·화면·도메인·설정·운영 절차를 관련 사람용 원본 문서에 반영하십시오. 같은 설명을 AI 문서에 복사하지 마십시오.
3. 경로·명령·작업 규칙이 바뀌면 `AGENTS.md`와 관련 AI 안내·참고 파일 목록도 고치십시오. `CLAUDE.md`는 `AGENTS.md`를 참조하는 파일입니다.
4. 문서 저장소(`../docs`)에서 `python3 scripts/check_docs.py --workspace`를 실행하십시오.
5. 완료 보고에 갱신한 문서와 검사 결과를 포함하십시오. 문서 변경이 불필요한 경우에는 이유를 적으십시오. 접근 제한으로 갱신하지 못했다면 미완료 항목으로 보고하십시오.
<!-- devdog-docs:end docs-update -->

## 기능 구현 문서

기능 1개를 구현하면 같은 작업에서 docs 저장소에 기능 문서를 작성하십시오. 사용자의 별도 요청을 기다리지 마십시오. 기능은 `controllers/sar_main/sar/` 모듈 추가, 공개 함수·클래스 추가, 공개 함수의 동작 변경, 컨트롤러 추가입니다.

1. 작성 대상과 구성은 docs [기능 문서](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/explanation/features/README.md)(로컬 `../docs/human/explanation/features/README.md`)를 따르십시오.
2. 문서 위치는 `../docs/human/explanation/features/<모듈 이름>.md`입니다. 같은 모듈 문서가 있으면 기존 문서를 갱신하십시오.
3. 검증 결과에는 실행한 명령과 결과만 적으십시오. 실행하지 않은 항목은 "미확인"으로 구분하십시오.
4. docs 저장소에서 `python3 scripts/check_docs.py --write-catalog`와 `python3 scripts/check_docs.py --workspace`를 실행하십시오.
5. docs 변경은 docs 저장소의 `docs/<모듈 이름>-feature` 브랜치와 PR로 올리고, sar-robot PR 본문에 docs PR 링크를 적으십시오.

<!-- devdog-docs:begin incident -->
## 장애 기록

배포된 서버·앱에서 확인된 오류를 수정하는 작업의 완료 조건에는 장애 기록이 포함입니다. 원인이 자명한 오타·문구 수정은 제외입니다. 수정과 같은 작업에서 수행하십시오.

1. `../docs/human/explanation/troubleshooting/`에 `YYYY-MM-DD(제목).md` 파일을 추가하십시오. 같은 폴더 `README.md`의 목록에도 한 줄을 추가하십시오.
2. 같은 원인이 다시 발생하면 새 파일 대신 기존 기록에 재발 날짜를 추가하십시오.
3. 구성은 해당 폴더 `README.md`의 "작성 방법"을 따르십시오.
<!-- devdog-docs:end incident -->
