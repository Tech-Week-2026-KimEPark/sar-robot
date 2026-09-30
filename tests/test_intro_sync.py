"""Intro 실습 파일이 sar-robot에 빠짐없이 같은 내용으로 등록되었는지 확인.

Intro와 sar-robot의 Git 인덱스 blob 해시를 비교한다. Webots가 월드를 열 때 수정하는
.wbproj(창 배치 상태)의 작업 트리 변경은 비교 대상이 아니다.
Intro 저장소가 형제 폴더(../PNU-TECHWEEK-260930)에 없으면 건너뛴다.
"""

import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
INTRO = ROOT.parent / "PNU-TECHWEEK-260930"
FOLDERS = ["controllers", "models", "protos", "worlds"]


def git_blobs(repo: Path) -> dict[str, str]:
    """경로별 Git 인덱스 blob 해시."""
    output = subprocess.run(
        ["git", "-C", str(repo), "ls-files", "-s", "--", *FOLDERS],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    blobs = {}
    for line in output.splitlines():
        meta, path = line.split("\t", 1)
        blobs[path] = meta.split()[1]
    return blobs


@pytest.mark.skipif(not (INTRO / ".git").exists(), reason="Intro 저장소 없음")
def test_intro_files_are_identical_copies():
    intro, ours = git_blobs(INTRO), git_blobs(ROOT)
    missing = sorted(path for path in intro if path not in ours)
    different = sorted(path for path in intro if path in ours and ours[path] != intro[path])
    assert intro, "Intro 실습 파일 없음"
    assert missing == [], f"누락: {missing}"
    assert different == [], f"내용 다름: {different}"
