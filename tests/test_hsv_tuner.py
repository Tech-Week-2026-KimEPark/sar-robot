import csv
import math
import os
import sys

import pytest

from sar import config

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "controllers", "hsv_tuner"))

import hsv_tuner  # noqa: E402

NAMES = {47: "apple", 49: "orange", 32: "sports ball"}
TARGET = {
    "cx": 400.0,
    "cy": 300.0,
    "w": 30.0,
    "h": 30.0,
    "conf": 0.8,
    "cls": 47,
    "color": "red",
    "dist": 2.0,
    "bearing": 0.0,
    "source": "yolo",
}
RAW = [
    {"xyxy": (385.0, 285.0, 415.0, 315.0), "conf": 0.8, "cls": 47, "ratio": 0.6},
    {"xyxy": (100.0, 285.0, 130.0, 315.0), "conf": 0.5, "cls": 49, "ratio": 0.01},
]


@pytest.mark.parametrize("color", sorted(config.HSV_RANGES))
def test_trackbar_values_roundtrip(color):
    ranges = config.HSV_RANGES[color]
    back = hsv_tuner.ranges_from_values(hsv_tuner.initial_values(ranges))
    assert back == [tuple(tuple(v) for v in r) for r in ranges]


def test_measurement_rows_target_and_raw():
    rows = hsv_tuner.measurement_rows(
        10.0,
        (1.0, 2.0, math.pi / 2),
        [TARGET],
        RAW,
        NAMES,
        [((0, 0, 0), (1, 1, 1))],
        70.0,
        "auto 2.0m",
    )
    assert [r["kind"] for r in rows] == ["target", "yolo_raw", "yolo_raw"]
    assert {r["note"] for r in rows} == {"auto 2.0m"}
    target = rows[0]
    assert target["cls_name"] == "apple"
    assert (target["world_x"], target["world_y"]) == pytest.approx((1.0, 4.0))
    assert target["robot_theta_deg"] == pytest.approx(90.0)
    assert target["yolo_ms"] == pytest.approx(70.0)
    assert rows[2]["cls_name"] == "orange"
    assert rows[2]["color_ratio"] == pytest.approx(0.01)
    assert rows[2]["w"] == pytest.approx(30.0)


def test_measurement_rows_none_when_nothing_seen():
    rows = hsv_tuner.measurement_rows(1.0, (0.0, 0.0, 0.0), [], [], {}, [])
    assert len(rows) == 1
    assert rows[0]["kind"] == "none"
    assert rows[0]["yolo_ms"] == ""


def test_color_fallback_row_has_empty_class():
    det = {**TARGET, "cls": None, "source": "color"}
    row = hsv_tuner.measurement_rows(0.0, (0.0, 0.0, 0.0), [det], [], NAMES, [])[0]
    assert row["cls"] == ""
    assert row["cls_name"] == ""


def test_append_csv_writes_header_once(tmp_path):
    path = str(tmp_path / "out" / "m.csv")
    rows = hsv_tuner.measurement_rows(1.0, (0.0, 0.0, 0.0), [TARGET], [], NAMES, [])
    assert hsv_tuner.append_csv(path, rows)
    assert hsv_tuner.append_csv(path, rows)
    with open(path, encoding="utf-8-sig", newline="") as f:
        read = list(csv.DictReader(f))
    assert len(read) == 2
    assert list(read[0]) == hsv_tuner.CSV_FIELDS


def test_append_csv_failure(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("x")
    assert hsv_tuner.append_csv(str(blocker / "m.csv"), []) is False


def test_summary_line():
    line = hsv_tuner.summary_line(12.3, (0.0, 0.0, 0.0), [TARGET], RAW, NAMES, 71.6, 0.85)
    assert line.startswith("[t=12.3s]")
    assert "yolo=72ms" in line
    assert "rtf=0.85" in line
    assert "target=apple conf=0.80 d=2.00m" in line
    assert "orange 0.50 r=0.01" in line
    empty = hsv_tuner.summary_line(0.0, (0.0, 0.0, 0.0), [], [], {}, None, None)
    assert "target=none" in empty
    assert "yolo=-" in empty


def test_cv2_key_name():
    assert hsv_tuner.cv2_key_name(-1) == (None, False)
    assert hsv_tuner.cv2_key_name(ord("w")) == ("w", False)
    assert hsv_tuner.cv2_key_name(ord("W")) == ("w", True)
    assert hsv_tuner.cv2_key_name(2490368) == ("up", False)
    assert hsv_tuner.cv2_key_name(ord(" ")) == (" ", False)


def test_drive_from_keys():
    assert hsv_tuner.drive_from_keys(set(), False) == (0.0, 0.0)
    assert hsv_tuner.drive_from_keys({"w"}, False) == pytest.approx((config.TUNER_V, 0.0))
    assert hsv_tuner.drive_from_keys({"up", "left"}, False) == pytest.approx(
        (config.TUNER_V, config.TUNER_W)
    )
    assert hsv_tuner.drive_from_keys({"w", "up"}, False)[0] == pytest.approx(config.TUNER_V)
    assert hsv_tuner.drive_from_keys({"w", "s"}, False) == (0.0, 0.0)
    fine = hsv_tuner.drive_from_keys({"d"}, True)
    assert fine == pytest.approx((0.0, -config.TUNER_W * config.TUNER_FINE_SCALE))


def test_key_hold_expires_and_cancels_opposite():
    hold = hsv_tuner.KeyHold(hold=0.5)
    hold.press("w", 0.0)
    assert hold.held(0.4) == ({"w"}, False)
    assert hold.held(0.6) == (set(), False)
    hold.press("w", 1.0)
    hold.press("a", 1.1, fine=True)
    assert hold.held(1.2) == ({"w", "a"}, True)
    hold.press("s", 1.3)  # 반대 방향 w 해제, a 유지
    assert hold.held(1.35)[0] == {"s", "a"}
    hold.clear()
    assert hold.held(1.35)[0] == set()
