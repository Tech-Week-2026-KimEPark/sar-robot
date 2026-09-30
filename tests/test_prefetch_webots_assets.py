import hashlib

from prefetch_webots_assets import cache_name, extract_urls

BASE = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/"


def test_extract_urls_from_console_log():
    log = (
        f"WARNING: Sofa > ImageTexture : Cannot download '{BASE}a/sofa.jpg', error code: 399\n"
        f"WARNING: Mesh : Cannot download '{BASE}b/tire.obj', error code: 399\n"
        f"WARNING: again '{BASE}a/sofa.jpg'\n"
        "WARNING: other 'https://example.com/x.jpg'\n"
    )
    assert extract_urls(log) == [BASE + "a/sofa.jpg", BASE + "b/tire.obj"]


def test_cache_name_is_sha1_of_url():
    url = BASE + "robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"
    assert cache_name(url) == hashlib.sha1(url.encode()).hexdigest()
