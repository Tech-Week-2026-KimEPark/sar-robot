import hashlib
from pathlib import Path

from prefetch_webots_assets import cache_name, crawl, dynamic_patterns, extract_urls, resolve

BASE = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/"


def test_extract_urls_from_console_log():
    log = (
        f"WARNING: Sofa > ImageTexture : Cannot download '{BASE}a/sofa.jpg', error code: 399\n"
        f"ERROR: Error downloading EXTERNPROTO 'Desk': Cannot download '{BASE}b/Desk.proto'\n"
        f"WARNING: again '{BASE}a/sofa.jpg'\n"
        "WARNING: other 'https://example.com/x.jpg'\n"
    )
    assert extract_urls(log) == [BASE + "a/sofa.jpg", BASE + "b/Desk.proto"]


def test_cache_name_is_sha1_of_url():
    url = BASE + "robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"
    assert cache_name(url) == hashlib.sha1(url.encode()).hexdigest()


def test_resolve_webots_and_relative_urls():
    proto = BASE + "humans/pedestrian/protos/Pedestrian.proto"
    assert (
        resolve("PedestrianTorso.proto", proto)
        == BASE + "humans/pedestrian/protos/PedestrianTorso.proto"
    )
    assert resolve("webots://projects/a/b.jpg", proto) == BASE + "a/b.jpg"


def test_dynamic_patterns_from_template():
    text = (
        "%< const path = 'webots://projects/default/worlds/textures/cubic'; >%\n"
        "url [ %<= '\"textures/pavement/' + textureName + '_pavement_base_color.jpg\"' >% ]\n"
        "url [ %<= '\"' + path + '/' + texture + '_back.hdr\"' >% ]\n"
    )
    patterns = dynamic_patterns(text)
    assert [folder for folder, _ in patterns] == [
        "textures/pavement/",
        "webots://projects/default/worlds/textures/cubic/",
    ]
    assert patterns[0][1].fullmatch("tiles_pavement_base_color.jpg")
    assert not patterns[0][1].fullmatch("tiles_pavement_normal.jpg")


def test_crawl_follows_nested_protos_and_folders(tmp_path):
    world = tmp_path / "w.wbt"
    world.write_text(f'EXTERNPROTO "{BASE}objects/A.proto"\n')
    files = {
        BASE + "objects/A.proto": 'EXTERNPROTO "B.proto"\nurl [ "textures/a.jpg" ]\n',
        BASE + "objects/B.proto": "url [ %<= '\"textures/' + name + '_b.jpg\"' >% ]\n",
    }
    listing = {BASE + "objects/textures/": ["x_b.jpg", "y_b.jpg", "x_c.jpg"]}

    def read_text(source):
        return files[source] if source in files else Path(source).read_text()

    urls, failures = crawl([str(world)], read_text, lambda folder: listing.get(folder, []))
    assert failures == []
    assert set(urls) == {
        BASE + "objects/A.proto",
        BASE + "objects/B.proto",
        BASE + "objects/textures/a.jpg",
        BASE + "objects/textures/x_b.jpg",
        BASE + "objects/textures/y_b.jpg",
    }
