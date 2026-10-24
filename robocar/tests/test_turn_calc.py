import argparse

from turn_calc import parse_end


def test_end_readings_name_the_side_instead_of_a_sign():
    assert parse_end("20L") == 20.0
    assert parse_end("15r") == -15.0
    assert parse_end(" 7.5R ") == -7.5
    assert parse_end("0") == 0.0


def test_an_end_reading_without_a_side_is_refused_unless_it_is_zero():
    for bad in ("20", "-20", "L20", "20 left", ""):
        try:
            parse_end(bad)
        except argparse.ArgumentTypeError:
            continue
        raise AssertionError(f"accepted {bad!r}")
