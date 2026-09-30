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

- `src/sar/`에서 Webots `controller` 모듈을 import하지 마십시오. Webots 접근은 `src/sar/robot_io.py`에만 작성합니다.
- 상수는 `src/sar/config.py`에 정의하십시오.
- `controllers/tb3_*`와 `requirements*.txt`의 Intro 기준 버전은 변경하지 마십시오.
- 브랜치 이름은 `<type>/<내용>` 형식입니다. AI 도구 이름을 넣지 마십시오.
- 작업을 마치기 전에 저장소 루트에서 다음 명령을 실행하십시오.

```bash
.venv/bin/ruff format .
.venv/bin/ruff check .
.venv/bin/python -m pytest -q
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

<!-- devdog-docs:begin incident -->
## 장애 기록

배포된 서버·앱에서 확인된 오류를 수정하는 작업의 완료 조건에는 장애 기록이 포함입니다. 원인이 자명한 오타·문구 수정은 제외입니다. 수정과 같은 작업에서 수행하십시오.

1. `../docs/human/explanation/troubleshooting/`에 `YYYY-MM-DD(제목).md` 파일을 추가하십시오. 같은 폴더 `README.md`의 목록에도 한 줄을 추가하십시오.
2. 같은 원인이 다시 발생하면 새 파일 대신 기존 기록에 재발 날짜를 추가하십시오.
3. 구성은 해당 폴더 `README.md`의 "작성 방법"을 따르십시오.
<!-- devdog-docs:end incident -->
