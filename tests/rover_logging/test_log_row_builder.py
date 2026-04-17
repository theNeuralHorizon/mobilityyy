from rover_logging.run_logger import build_tag_log_row, build_tag_png_name


def test_build_tag_log_row_contains_expected_fields():
    row = build_tag_log_row(
        unix_timestamp=1713390000,
        tag_id=2,
        decision="TURN_RIGHT",
        image_name="1713390000.png",
    )

    assert row == ["1713390000", "2", "TURN_RIGHT", "1713390000.png"]


def test_build_tag_png_name_uses_unix_timestamp():
    assert build_tag_png_name(1713390000) == "1713390000.png"
