## 변경 내용

-

## 담당 모듈

- [ ] A 통합 (`mission/`, `robot_io.py`, `config.py`)
- [ ] B 탐색·경로 (`mapping/`, `planning/`)
- [ ] C 인식 (`perception/`)
- [ ] D 제어 (`control/`, `localization/`)

## 인터페이스 변경

- [ ] 없음
- [ ] 있음. 변경 내용과 영향받는 모듈:

## 테스트 결과

- [ ] `pytest -q` 통과
- [ ] `ruff check .`, `ruff format --check .` 통과
- [ ] Webots 확인 (월드, 확인한 동작):

## AI 작성 코드 확인

- [ ] x/y·row/col 혼동 없음
- [ ] 도·라디안 혼용 없음, 회전 부호 확인
- [ ] 무한 루프 타임아웃, `None` 반환 처리 확인
- [ ] 매직 넘버를 `config.py`로 분리
