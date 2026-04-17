from rover_logging.run_logger import build_tag_log_row, build_tag_png_name


def test_build_tag_log_row_contains_expected_fields():
    row = build_tag_log_row(
        unix_timestamp=1713390000,
        tag_id=2,
        decision="TURN_RIGHT",
        image_name="1713390000_001_tag2.png",
    )

    assert row == ["1713390000", "2", "TURN_RIGHT", "1713390000_001_tag2.png"]


def test_build_tag_png_name_includes_seq_and_label():
    name = build_tag_png_name(1713390000, seq=3, label=4)
    assert name == "1713390000_003_tag4.png"


def test_build_tag_png_name_defaults():
    name = build_tag_png_name(1713390000)
    assert name == "1713390000_000_tag0.png"


def test_build_tag_log_row_all_strings():
    row = build_tag_log_row(
        unix_timestamp=9999999999,
        tag_id=5,
        decision="TRACK_ORANGE",
        image_name="test.png",
    )
    assert all(isinstance(field, str) for field in row)
    assert len(row) == 4
