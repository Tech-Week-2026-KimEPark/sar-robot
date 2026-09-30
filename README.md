# sar-robot

2026 부산대학교 TECH WEEK Autonomous Search and Rescue 해커톤 팀 코드입니다. Webots R2025a의 TurtleBot3 Burger가 미지 환경을 탐색하고 목표 물체를 찾아 목적지로 이동한 뒤 시작 지점으로 복귀합니다.

## 빠른 시작

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install -r requirements-dev.txt -r requirements-yolo.txt
.venv/bin/python -m pytest -q
.venv/bin/python scripts/prefetch_webots_assets.py
```

Webots Preferences의 Python command를 `.venv/bin/python` 절대 경로로 설정한 뒤 `worlds/sar_apartment.wbt`를 열어 실행하십시오. 팀 컨트롤러는 `controllers/sar_main/`이며 팀 코드는 같은 폴더의 `sar/` 패키지입니다. 마지막 명령은 Webots 텍스처 다운로드 오류를 방지하기 위한 에셋 사전 캐시입니다. 패키지 버전은 Intro 과정 기준입니다. 전체 절차는 [개발 환경 설정](docs/human/how-to/dev-setup.md)에 있습니다.

## 문서

| 문서 | 내용 |
|---|---|
| [개발 환경 설정](docs/human/how-to/dev-setup.md) | 가상환경, Webots 설치·연결, 테스트 |
| [Intro 과정 환경 기준](docs/human/reference/intro-environment.md) | Python·패키지 버전, 로봇 상수, 실습 파일 |
| [코드 구조](docs/human/explanation/architecture.md) | 폴더 구성, 데이터 흐름 |
| [코드 작성 규칙](docs/human/reference/code-conventions.md) | Python 스타일, 이름·상수·테스트 규칙 |
| [모듈 인터페이스](docs/human/reference/interfaces.md) | 좌표·단위 규칙, 모듈 함수 형식 |
| [팀 규칙](https://github.com/Tech-Week-2026-KimEPark/docs/blob/main/human/reference/team/README.md) | 역할, Git 작업, 협업 규칙 |

문서 위키는 https://tech-week-2026-kimepark.github.io/docs/sar-robot/ 입니다.
