"""Webots 원격 에셋을 로컬 캐시에 미리 저장.

Webots R2025a(Qt 6.5.3)는 raw.githubusercontent.com 텍스처·메시를 HTTP/2로 받을 때
"error code: 399" 오류로 실패한다. 이 스크립트는 같은 파일을 HTTP/1.1로 받아
Webots 캐시(파일 이름 = URL의 SHA1)에 저장한다.

사용법:
    python scripts/prefetch_webots_assets.py                    # worlds/webots_assets.txt
    python scripts/prefetch_webots_assets.py webots_console.txt # Webots 콘솔 로그의 실패 URL
"""

import argparse
import hashlib
import os
import re
import sys
import urllib.request
from pathlib import Path

ALLOWED_PREFIX = "https://raw.githubusercontent.com/cyberbotics/webots/"
URL_PATTERN = re.compile(re.escape(ALLOWED_PREFIX) + r"[^\s'\"<>]+")
DEFAULT_LIST = Path(__file__).resolve().parents[1] / "worlds" / "webots_assets.txt"


def extract_urls(text: str) -> list[str]:
    """텍스트에서 Webots 저장소 URL을 순서대로 중복 없이 추출."""
    return list(dict.fromkeys(URL_PATTERN.findall(text)))


def cache_name(url: str) -> str:
    return hashlib.sha1(url.encode("utf-8")).hexdigest()


def default_cache_dir() -> Path:
    """Qt CacheLocation 기준 Webots 에셋 캐시 폴더."""
    if sys.platform == "darwin":
        return Path.home() / "Library/Caches/Cyberbotics/Webots/assets"
    if os.name == "nt":
        return Path(os.environ["LOCALAPPDATA"]) / "Cyberbotics/Webots/cache/assets"
    base = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    return base / "Cyberbotics/Webots/assets"


def download(url: str, target: Path) -> None:
    with urllib.request.urlopen(url, timeout=30) as response:
        data = response.read()
    tmp = target.with_suffix(".part")
    tmp.write_bytes(data)
    tmp.replace(target)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sources", nargs="*", type=Path, default=[DEFAULT_LIST])
    parser.add_argument("--cache-dir", type=Path, default=None)
    args = parser.parse_args(argv)

    cache_dir = args.cache_dir or default_cache_dir()
    cache_dir.mkdir(parents=True, exist_ok=True)
    urls = extract_urls("\n".join(p.read_text(encoding="utf-8") for p in args.sources))

    failed = 0
    for url in urls:
        target = cache_dir / cache_name(url)
        if target.exists():
            print(f"cached   {url}")
            continue
        try:
            download(url, target)
            print(f"saved    {url}")
        except OSError as error:
            failed += 1
            print(f"failed   {url} ({error})")
    print(f"{len(urls)} URLs, {failed} failed. Cache: {cache_dir}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
