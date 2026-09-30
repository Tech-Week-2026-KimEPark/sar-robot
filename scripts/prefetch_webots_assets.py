"""Webots 원격 PROTO·에셋을 로컬 캐시에 미리 저장.

Webots R2025a(Qt 6.5.3)는 raw.githubusercontent.com 파일을 받을 때
"error code: 399"(HTTP/2 헤더 압축 오류) 또는 "error code: 2: Connection closed"로 실패한다.
이 스크립트는 같은 파일을 HTTP/1.1로 받아 Webots 캐시(파일 이름 = URL의 SHA1)에 저장한다.

입력 파일 종류:
- .wbt, .proto: EXTERNPROTO를 재귀로 따라가며 PROTO와 에셋 경로를 수집.
  템플릿으로 이름을 만드는 텍스처는 GitHub 폴더 목록에서 이름 패턴이 맞는 파일을 모두 수집
- 그 외 텍스트: Webots 콘솔 로그나 URL 목록에서 URL 추출

사용법:
    python scripts/prefetch_webots_assets.py          # worlds/*.wbt 전체
    python scripts/prefetch_webots_assets.py log.txt  # 콘솔 로그의 실패 URL
"""

import argparse
import functools
import hashlib
import json
import os
import re
import subprocess
import sys
import urllib.request
from collections.abc import Callable, Iterable
from pathlib import Path
from urllib.parse import urljoin

ALLOWED_PREFIX = "https://raw.githubusercontent.com/cyberbotics/webots/"
WEBOTS_BASE = ALLOWED_PREFIX + "R2025a/"
ROOT = Path(__file__).resolve().parents[1]
URL_PATTERN = re.compile(re.escape(ALLOWED_PREFIX) + r"[^\s'\"<>]+")
EXTERNPROTO = re.compile(r'EXTERNPROTO\s+"([^"]+)"')
EXTENSIONS = r"\.(?:jpg|jpeg|png|hdr|obj|dae|stl|mtl|wav|mp3)"
ASSET = re.compile(r"[\"']([^\"'\s%+]+" + EXTENSIONS + r")[\"']", re.IGNORECASE)
TEMPLATE_BLOCK = re.compile(r"%<.*?>%", re.DOTALL)
TEMPLATE_EXPR = re.compile(r"%<=(.*?)>%")
CONST = re.compile(r"(?:const|let|var)\s+(\w+)\s*=\s*'([^']*)'")
STRING = re.compile(r"'([^']*)'|\"([^\"]*)\"")
WILDCARD = "\0"


def extract_urls(text: str) -> list[str]:
    """텍스트에서 Webots 저장소 URL을 순서대로 중복 없이 추출."""
    return list(dict.fromkeys(URL_PATTERN.findall(text)))


def cache_name(url: str) -> str:
    return hashlib.sha1(url.encode("utf-8")).hexdigest()


def resolve(ref: str, base: str) -> str:
    """PROTO·월드 안 참조 경로를 절대 URL 또는 로컬 경로로 변환."""
    if ref.startswith("webots://"):
        return WEBOTS_BASE + ref[len("webots://") :]
    if ref.startswith(("https://", "http://")):
        return ref
    if base.startswith("https://"):
        return urljoin(base, ref)
    return str((Path(base).parent / ref).resolve())


def dynamic_patterns(text: str) -> list[tuple[str, re.Pattern]]:
    """템플릿 표현식에서 (폴더 경로, 파일 이름 정규식) 목록을 추출.

    예: '"textures/pavement/' + textureName + '_pavement_base_color.jpg"'
    → ("textures/pavement/", r".*_pavement_base_color\\.jpg")
    문자열 상수(const path = '...')는 값으로 치환하고, 나머지 변수는 임의 문자열로 취급한다.
    """
    consts = dict(CONST.findall(text))
    patterns = []
    for expr in TEMPLATE_EXPR.findall(text):
        parts = []
        for token in expr.split("+"):
            token = token.strip()
            literal = STRING.fullmatch(token)
            if literal:
                parts.append((literal.group(1) or literal.group(2) or "").replace('"', ""))
            else:
                parts.append(consts.get(token, WILDCARD))
        path = "".join(parts)
        if not re.search(EXTENSIONS + "$", path, re.IGNORECASE) or WILDCARD not in path:
            continue
        folder, _, name = path.rpartition("/")
        if WILDCARD in folder:
            continue  # 폴더 이름까지 변수이면 목록 조회 대상을 정할 수 없음
        regex = ".*".join(re.escape(piece) for piece in name.split(WILDCARD))
        patterns.append((folder + "/", re.compile(regex)))
    return patterns


def crawl(
    sources: Iterable[str],
    read_text: Callable[[str], str],
    list_dir: Callable[[str], list[str]],
) -> tuple[list[str], list[str]]:
    """월드·PROTO에서 EXTERNPROTO를 재귀로 따라가며 (원격 URL 목록, 목록 조회 실패 폴더)를 반환."""
    found: dict[str, None] = {}
    list_failures: list[str] = []
    queue, visited = list(sources), set()
    while queue:
        source = queue.pop()
        if source in visited:
            continue
        visited.add(source)
        try:
            text = read_text(source)
        except OSError:
            continue  # 실패한 PROTO URL은 main()에서 다시 시도하고 실패로 보고
        for ref in EXTERNPROTO.findall(text):
            target = resolve(ref, source)
            if target.startswith(ALLOWED_PREFIX):
                found[target] = None
            if target.startswith(ALLOWED_PREFIX) or Path(target).is_file():
                queue.append(target)
        for ref in ASSET.findall(TEMPLATE_BLOCK.sub("", text)):
            target = resolve(ref, source)
            if target.startswith(ALLOWED_PREFIX):
                found[target] = None
        for folder, name in dynamic_patterns(text):
            folder_url = resolve(folder, source)
            if not folder_url.startswith(ALLOWED_PREFIX):
                continue
            try:
                names = list_dir(folder_url)
            except OSError as error:
                list_failures.append(f"{folder_url} ({error})")
                continue
            for file_name in names:
                if name.fullmatch(file_name):
                    found[folder_url + file_name] = None
    return list(found), list(dict.fromkeys(list_failures))


@functools.cache
def github_token() -> str | None:
    """GITHUB_TOKEN 또는 gh CLI 로그인 토큰. 없으면 비인증(시간당 60회) 조회."""
    token = os.environ.get("GITHUB_TOKEN")
    if token:
        return token
    try:
        result = subprocess.run(["gh", "auth", "token"], capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


def default_cache_dir() -> Path:
    """Qt CacheLocation 기준 Webots 에셋 캐시 폴더."""
    if sys.platform == "darwin":
        return Path.home() / "Library/Caches/Cyberbotics/Webots/assets"
    if os.name == "nt":
        return Path(os.environ["LOCALAPPDATA"]) / "Cyberbotics/Webots/cache/assets"
    base = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return base / "Cyberbotics/Webots/assets"


class Cache:
    def __init__(self, directory: Path):
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.saved = 0

    def path(self, url: str) -> Path:
        return self.directory / cache_name(url)

    def fetch(self, url: str) -> bytes:
        """캐시에 있으면 읽고, 없으면 내려받아 저장."""
        target = self.path(url)
        if target.exists():
            return target.read_bytes()
        with urllib.request.urlopen(url, timeout=30) as response:
            data = response.read()
        tmp = target.with_suffix(".part")
        tmp.write_bytes(data)
        tmp.replace(target)
        self.saved += 1
        return data

    def list_dir(self, folder_url: str) -> list[str]:
        """폴더의 파일 이름 목록. GitHub API 응답을 캐시에 저장해 재사용."""
        path = folder_url[len(WEBOTS_BASE) :].rstrip("/")
        api = f"https://api.github.com/repos/cyberbotics/webots/contents/{path}?ref=R2025a"
        target = self.path(api)
        if not target.exists():
            request = urllib.request.Request(api, headers={"Accept": "application/vnd.github+json"})
            token = github_token()
            if token:
                request.add_header("Authorization", f"Bearer {token}")
            with urllib.request.urlopen(request, timeout=30) as response:
                entries = json.load(response)
            names = [entry["name"] for entry in entries if entry.get("type") == "file"]
            target.write_text(json.dumps(names), encoding="utf-8")
        return json.loads(target.read_text(encoding="utf-8"))

    def read_text(self, source: str) -> str:
        if source.startswith(ALLOWED_PREFIX):
            return self.fetch(source).decode("utf-8", errors="replace")
        return Path(source).read_text(encoding="utf-8", errors="replace")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sources", nargs="*", type=Path)
    parser.add_argument("--cache-dir", type=Path, default=None)
    args = parser.parse_args(argv)

    sources = args.sources or sorted((ROOT / "worlds").glob("*.wbt"))
    cache = Cache(args.cache_dir or default_cache_dir())
    scenes = [str(p.resolve()) for p in sources if p.suffix in (".wbt", ".proto")]
    texts = [p for p in sources if p.suffix not in (".wbt", ".proto")]

    urls, list_failures = crawl(scenes, cache.read_text, cache.list_dir)
    for path in texts:
        urls += extract_urls(path.read_text(encoding="utf-8"))
    urls = list(dict.fromkeys(urls))

    for failure in list_failures:
        print(f"list failed {failure}")
    failed = len(list_failures)
    for url in urls:
        try:
            cache.fetch(url)
        except OSError as error:
            failed += 1
            print(f"failed   {url} ({error})")
    print(f"{len(urls)} URLs, {cache.saved} saved, {failed} failed. Cache: {cache.directory}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
