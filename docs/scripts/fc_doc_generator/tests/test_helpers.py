"""
Tests for pure helper functions in fc_doc_generator.py.
"""
import fc_doc_generator as fcdg


# ---------------------------------------------------------------------------
# make_outputs_range
# ---------------------------------------------------------------------------

class TestMakeOutputsRange:
    def test_consecutive(self):
        assert fcdg.make_outputs_range([1, 2, 3]) == "1-3"

    def test_gap(self):
        assert fcdg.make_outputs_range([1, 2, 3, 5]) == "1-3, 5"

    def test_all_gaps(self):
        assert fcdg.make_outputs_range([1, 3, 5]) == "1, 3, 5"

    def test_single(self):
        assert fcdg.make_outputs_range([4]) == "4"

    def test_empty(self):
        assert fcdg.make_outputs_range([]) == ""

    def test_unsorted_input(self):
        assert fcdg.make_outputs_range([3, 1, 2]) == "1-3"

    def test_full_range(self):
        assert fcdg.make_outputs_range(list(range(1, 9))) == "1-8"


# ---------------------------------------------------------------------------
# _make_slug
# ---------------------------------------------------------------------------

class TestMakeSlug:
    def test_lowercase(self):
        assert fcdg._make_slug("Holybro") == "holybro"

    def test_spaces_to_underscores(self):
        assert fcdg._make_slug("KakuteH7 V2") == "kakuteh7_v2"

    def test_hyphens_to_underscores(self):
        assert fcdg._make_slug("fmu-v6x") == "fmu_v6x"

    def test_mixed(self):
        assert fcdg._make_slug("Pixhawk 6X") == "pixhawk_6x"

    def test_leading_trailing_whitespace(self):
        assert fcdg._make_slug("  test  ") == "test"

    def test_multiple_hyphens(self):
        assert fcdg._make_slug("a--b") == "a_b"


# ---------------------------------------------------------------------------
# _doc_filename
# ---------------------------------------------------------------------------

class TestDocFilename:
    def test_with_fmu_version(self):
        assert fcdg._doc_filename("holybro", "Pixhawk 6X", "fmu-v6x") == "holybro_pixhawk_6x_fmu_v6x.md"

    def test_without_fmu_version(self):
        assert fcdg._doc_filename("holybro", "KakuteH7") == "holybro_kakuteh7.md"

    def test_manufacturer_slug(self):
        name = fcdg._doc_filename("Cube Pilot", "Orange", None)
        assert name.startswith("cube_pilot_")


# ---------------------------------------------------------------------------
# _serial_label
# ---------------------------------------------------------------------------

class TestSerialLabel:
    def test_tel1_to_telem1(self):
        assert fcdg._serial_label("TEL1") == "TELEM1"

    def test_tel2_to_telem2(self):
        assert fcdg._serial_label("TEL2") == "TELEM2"

    def test_gps1_unchanged(self):
        assert fcdg._serial_label("GPS1") == "GPS1"

    def test_rc_unchanged(self):
        assert fcdg._serial_label("RC") == "RC"

    def test_telem_already_normalised(self):
        # If input already says TELEM3, it should stay
        assert fcdg._serial_label("TELEM3") == "TELEM3"


# ---------------------------------------------------------------------------
# _and_list
# ---------------------------------------------------------------------------

class TestAndList:
    def test_one_item(self):
        assert fcdg._and_list(["A"]) == "A"

    def test_two_items(self):
        assert fcdg._and_list(["A", "B"]) == "A and B"

    def test_three_items(self):
        assert fcdg._and_list(["A", "B", "C"]) == "A, B, and C"

    def test_empty(self):
        assert fcdg._and_list([]) == ""


# ---------------------------------------------------------------------------
# _rc_serial_protocols
# ---------------------------------------------------------------------------

class TestRcSerialProtocols:
    def test_rc_input_includes_sbus_crsf_st24(self):
        entry = {"has_rc_input": True}
        protos = fcdg._rc_serial_protocols(entry)
        assert "SBUS" in protos
        assert "CRSF" in protos
        assert "ST24" in protos
        assert "SUMD" in protos

    def test_common_rc_excludes_st24_sumd(self):
        entry = {"has_common_rc": True}
        protos = fcdg._rc_serial_protocols(entry)
        assert "SBUS" in protos
        assert "CRSF" in protos
        assert "ST24" not in protos
        assert "SUMD" not in protos

    def test_ppm_shared_prepended(self):
        entry = {"has_rc_input": True, "has_ppm_pin": True, "ppm_shared_with_rc_serial": True}
        protos = fcdg._rc_serial_protocols(entry)
        assert protos[0] == "PPM"

    def test_ppm_not_shared_not_prepended(self):
        entry = {"has_rc_input": True, "has_ppm_pin": True, "ppm_shared_with_rc_serial": False}
        protos = fcdg._rc_serial_protocols(entry)
        assert protos[0] != "PPM"

    def test_io_board_only(self):
        entry = {"has_io_board": True}
        protos = fcdg._rc_serial_protocols(entry)
        assert "SBUS" in protos

    def test_no_rc_empty(self):
        entry = {}
        protos = fcdg._rc_serial_protocols(entry)
        assert protos == []
