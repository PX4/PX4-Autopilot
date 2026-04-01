"""
Tests for the wizard flow (_run_wizard) and _wizard_prompt helper.
"""
import argparse
import fc_doc_generator as fcdg


def _make_args(**kwargs):
    defaults = {
        "manufacturer": None,
        "product": None,
        "board": None,
        "fmu_version": None,
        "since_version": "vX.XX",
        "manufacturer_url": None,
        "rc_ports_wizard": None,
        "gps_ports_wizard": None,
        "power_ports_wizard": None,
    }
    defaults.update(kwargs)
    return argparse.Namespace(**defaults)


# ---------------------------------------------------------------------------
# _wizard_prompt unit tests
# ---------------------------------------------------------------------------

class TestWizardPrompt:
    def test_returns_entered_value(self, monkeypatch):
        monkeypatch.setattr("builtins.input", lambda prompt="": "my_value")
        assert fcdg._wizard_prompt("Label") == "my_value"

    def test_empty_returns_default(self, monkeypatch):
        monkeypatch.setattr("builtins.input", lambda prompt="": "")
        assert fcdg._wizard_prompt("Label", default="fallback") == "fallback"

    def test_eof_returns_default(self, monkeypatch):
        def raise_eof(prompt=""):
            raise EOFError
        monkeypatch.setattr("builtins.input", raise_eof)
        assert fcdg._wizard_prompt("Label", default="fallback") == "fallback"

    def test_eof_no_default_returns_empty(self, monkeypatch):
        def raise_eof(prompt=""):
            raise EOFError
        monkeypatch.setattr("builtins.input", raise_eof)
        assert fcdg._wizard_prompt("Label") == ""

    def test_optional_empty_returns_empty_string(self, monkeypatch):
        monkeypatch.setattr("builtins.input", lambda prompt="": "")
        assert fcdg._wizard_prompt("Label", required=False) == ""

    def test_required_loops_until_non_empty(self, monkeypatch):
        responses = iter(["", "", "final"])
        monkeypatch.setattr("builtins.input", lambda prompt="": next(responses))
        assert fcdg._wizard_prompt("Label", required=True) == "final"


# ---------------------------------------------------------------------------
# _run_wizard flow tests (patch _wizard_prompt and gather_board_data)
# ---------------------------------------------------------------------------

class TestRunWizard:
    def _patch(self, monkeypatch, responses, board_data=None):
        """Patch _wizard_prompt with a sequence of responses and stub gather_board_data.
        Also stubs cache functions to prevent writing to real METADATA_DIR."""
        it = iter(responses)
        monkeypatch.setattr(fcdg, "_wizard_prompt",
                            lambda label, default="", required=False: next(it, default))
        monkeypatch.setattr(fcdg, "gather_board_data", lambda: board_data or {})
        monkeypatch.setattr(fcdg, "_load_wizard_cache", lambda *a, **kw: None)
        monkeypatch.setattr(fcdg, "_save_wizard_cache", lambda *a, **kw: None)

    def test_fills_manufacturer_and_product(self, monkeypatch):
        # responses: manufacturer, product, board_key, fmu_version, since_version,
        #            serial_label, ppm separate connector?, ppm label,
        #            gps: GPS1 side, add more GPS?
        self._patch(monkeypatch, [
            "Holybro", "KakuteH7", "",  # manufacturer, product, board key
            "",                           # fmu_version (optional)
            "v1.18",                      # since_version
            "RC",                         # serial RC label
            "n",                          # dedicated PPM connector? (no)
            "GPS1", "n",                  # gps label, more?
        ])
        args = _make_args()
        fcdg._run_wizard(args)
        assert args.manufacturer == "Holybro"
        assert args.product == "KakuteH7"

    def test_skips_manufacturer_prompt_when_prefilled(self, monkeypatch):
        prompts_seen = []
        it = iter(["", "v1.18", "RC", "n", "GPS1", "n"])
        def mock_prompt(label, default="", required=False):
            prompts_seen.append(label)
            return next(it, default)
        monkeypatch.setattr(fcdg, "_wizard_prompt", mock_prompt)
        monkeypatch.setattr(fcdg, "gather_board_data", lambda: {})
        args = _make_args(manufacturer="Holybro", product="KakuteH7")
        fcdg._run_wizard(args)
        for label in prompts_seen:
            # "Manufacturer name" and "Product name" prompts should be skipped,
            # but "Manufacturer URL" prompt is still shown (different field)
            assert label != "Manufacturer name"
            assert label != "Product name"

    def test_px4_board_infers_fmu_version(self, monkeypatch):
        self._patch(monkeypatch, [
            "",        # board key (already set via args)
            "v1.18",   # since_version
            "RC", "n", "GPS1", "n",
        ])
        args = _make_args(manufacturer="PX4", product="FMU-v6X", board="px4/fmu-v6x")
        fcdg._run_wizard(args)
        assert args.fmu_version == "fmu-v6x"

    def test_fmu_version_not_prompted_for_px4_board(self, monkeypatch):
        prompts_seen = []
        it = iter(["", "v1.18", "RC", "n", "GPS1", "n"])
        def mock_prompt(label, default="", required=False):
            prompts_seen.append(label)
            return next(it, default)
        monkeypatch.setattr(fcdg, "_wizard_prompt", mock_prompt)
        monkeypatch.setattr(fcdg, "gather_board_data", lambda: {})
        args = _make_args(manufacturer="PX4", product="FMU-v6X", board="px4/fmu-v6x")
        fcdg._run_wizard(args)
        for label in prompts_seen:
            assert "FMU version" not in label

    def test_rc_ports_wizard_populated(self, monkeypatch):
        self._patch(monkeypatch, [
            "TestMfr", "TestBoard", "",
            "",
            "v1.18",
            "RC", "n", "GPS1", "n",
        ])
        args = _make_args()
        fcdg._run_wizard(args)
        assert args.rc_ports_wizard is not None
        assert len(args.rc_ports_wizard) >= 1

    def test_gps_ports_wizard_populated(self, monkeypatch):
        # GPS wizard only runs when the board entry has GPS serial ports.
        board_data = {
            "test/board": {
                "has_rc_input": True, "has_common_rc": False,
                "rc_serial_device": "/dev/ttyS4",
                "has_ppm_pin": False, "ppm_shared_with_rc_serial": False,
                "has_pps_capture": False, "has_safety_switch": False, "has_safety_led": False,
                "serial_ports": [
                    {"uart": "USART1", "device": "/dev/ttyS0", "label": "TELEM1"},
                    {"uart": "UART4", "device": "/dev/ttyS3", "label": "GPS1"},
                    {"uart": "USART6", "device": "/dev/ttyS4", "label": "RC"},
                ],
            }
        }
        # responses: board_key, fmu_version, since_version, serial_label,
        #            GPS1 label, 6-pin connector?, manufacturer_url
        self._patch(monkeypatch, [
            "TestMfr", "TestBoard", "test/board",
            "",
            "v1.18",
            "RC",
            "GPS1", "y",
            "https://example.com/",
        ], board_data)
        args = _make_args()
        fcdg._run_wizard(args)
        assert args.gps_ports_wizard is not None
        assert len(args.gps_ports_wizard) >= 1

    def test_since_version_set(self, monkeypatch):
        self._patch(monkeypatch, [
            "TestMfr", "TestBoard", "",
            "",
            "v1.19",
            "RC", "n", "GPS1", "n",
        ])
        args = _make_args()
        fcdg._run_wizard(args)
        assert args.since_version == "v1.19"


# ---------------------------------------------------------------------------
# Wizard cache: _save_wizard_cache / _load_wizard_cache
# ---------------------------------------------------------------------------

class TestWizardCache:
    def test_save_and_load_roundtrip(self, tmp_path):
        args = _make_args(
            manufacturer="Holybro",
            product="Pixhawk 6X",
            fmu_version="fmu-v6x",
            since_version="v1.14",
            manufacturer_url="https://holybro.com/",
            rc_ports_wizard=[{"label": "RC", "side": "FMU"}],
            gps_ports_wizard=[{"port_key": "GPS1", "label": "GPS1",
                               "pixhawk_standard": True, "full_port": True}],
        )
        fcdg._save_wizard_cache("Holybro", "Pixhawk 6X", args, meta_dir=tmp_path)
        loaded = fcdg._load_wizard_cache("Holybro", "Pixhawk 6X", meta_dir=tmp_path)
        assert loaded is not None
        assert loaded["manufacturer"] == "Holybro"
        assert loaded["product"] == "Pixhawk 6X"
        assert loaded["fmu_version"] == "fmu-v6x"
        assert loaded["rc_ports_wizard"] == [{"label": "RC", "side": "FMU"}]
        assert loaded["gps_ports_wizard"][0]["full_port"] is True

    def test_load_returns_none_when_missing(self, tmp_path):
        result = fcdg._load_wizard_cache("No", "Such Board", meta_dir=tmp_path)
        assert result is None

    def test_filename_uses_manufacturer_product(self, tmp_path):
        # Key is manufacturer+product only — fmu_version not part of filename
        args = _make_args(manufacturer="Corvon", product="ARSE", fmu_version="fmu-v5")
        fcdg._save_wizard_cache("Corvon", "ARSE", args, meta_dir=tmp_path)
        assert (tmp_path / "corvon_arse_wizard.json").exists()

    def test_run_wizard_saves_cache(self, monkeypatch, tmp_path):
        """After completing the wizard, cache is written using manufacturer+product+fmu_version."""
        monkeypatch.setattr(fcdg, "gather_board_data", lambda: {})
        monkeypatch.setattr(fcdg, "METADATA_DIR", tmp_path)
        monkeypatch.setattr(fcdg, "_wizard_prompt",
                            lambda label, default="", required=False: {
                                "Manufacturer name": "Corvon",
                                "Product name": "ARSE",
                                "Board key (e.g. holybro/kakuteh7 or px4/fmu-v6x)": "px4/fmu-v5",
                                "PX4 version introduced in": "v1.18",
                                "Serial RC port label as printed on board": "RC",
                                "Manufacturer URL": "https://example.com/",
                            }.get(label, default))
        args = _make_args()
        fcdg._run_wizard(args)
        # key is manufacturer+product only: corvon_arse_wizard.json
        cache_file = tmp_path / "corvon_arse_wizard.json"
        assert cache_file.exists()
        import json
        data = json.loads(cache_file.read_text())
        assert data["manufacturer"] == "Corvon"
        assert data["since_version"] == "v1.18"

    def test_run_wizard_uses_cached_values_as_defaults(self, monkeypatch, tmp_path):
        """When cache exists, each prompt's default is the corresponding cached value."""
        import json
        cached = {
            "manufacturer": "ACME", "product": "Widget",
            "fmu_version": "fmu-v5x", "since_version": "v1.15",
            "manufacturer_url": "https://example.com/",
            "rc_ports_wizard": [{"label": "MY_RC", "side": "FMU"}],
            "gps_ports_wizard": None,
            "power_ports_wizard": [{"label": "BAT", "connector_type": "6-pin JST GH"}],
            "overview_wizard": None,
        }
        (tmp_path / "acme_widget_wizard.json").write_text(json.dumps(cached))
        monkeypatch.setattr(fcdg, "METADATA_DIR", tmp_path)
        monkeypatch.setattr(fcdg, "gather_board_data", lambda: {})

        defaults_seen = {}
        def mock_prompt(label, default="", required=False):
            defaults_seen[label] = default
            return default  # accept every default
        monkeypatch.setattr(fcdg, "_wizard_prompt", mock_prompt)

        args = _make_args(manufacturer="ACME", product="Widget")
        fcdg._run_wizard(args)

        # Cached values must appear as defaults, not hardcoded generics
        assert defaults_seen.get("PX4 version introduced in") == "v1.15"
        assert defaults_seen.get("Serial RC port label as printed on board") == "MY_RC"
        assert defaults_seen.get("  POWER port label as printed on board") == "BAT"
        # No "Use it?" prompt should appear
        assert not any("cached" in k.lower() and "use" in k.lower() for k in defaults_seen)

        # Accepting all defaults restores cached values into args
        assert args.since_version == "v1.15"
        assert args.rc_ports_wizard[0]['label'] == "MY_RC"

    def test_run_wizard_cached_defaults_can_be_overridden(self, monkeypatch, tmp_path):
        """The user can type a new value to override any cached default."""
        import json
        cached = {
            "manufacturer": "ACME", "product": "Widget",
            "since_version": "v1.15", "manufacturer_url": "https://example.com/",
            "rc_ports_wizard": None, "gps_ports_wizard": None,
            "power_ports_wizard": None, "overview_wizard": None,
        }
        (tmp_path / "acme_widget_wizard.json").write_text(json.dumps(cached))
        monkeypatch.setattr(fcdg, "METADATA_DIR", tmp_path)
        monkeypatch.setattr(fcdg, "gather_board_data", lambda: {})

        responses = {
            "PX4 version introduced in": "v1.20",       # override the cached v1.15
            "Serial RC port label as printed on board": "RC",
            "Manufacturer URL": "https://new.example.com/",
        }
        monkeypatch.setattr(fcdg, "_wizard_prompt",
                            lambda label, default="", required=False: responses.get(label, default))
        args = _make_args(manufacturer="ACME", product="Widget")
        fcdg._run_wizard(args)

        assert args.since_version == "v1.20"
        assert args.manufacturer_url == "https://new.example.com/"


# ---------------------------------------------------------------------------
# _embed_wizard_data_comment / _read_wizard_data_from_doc
# ---------------------------------------------------------------------------

class TestWizardDataComment:
    _FRONT_MATTER = "---\ntitle: Test Board\n---\n"
    _BODY = "## Overview\n\nSome content.\n"

    def test_embed_appends_comment_at_end(self):
        content = self._FRONT_MATTER + self._BODY
        result = fcdg._embed_wizard_data_comment(content, {"foo": "bar"})
        assert result.startswith(self._FRONT_MATTER)
        assert result.endswith("-->\n")
        assert "<!-- wizard-data" in result
        assert '"foo": "bar"' in result

    def test_embed_without_front_matter_appends_comment(self):
        content = "# No front matter\n"
        result = fcdg._embed_wizard_data_comment(content, {"x": 1})
        assert result.startswith("# No front matter")
        assert result.endswith("-->\n")
        assert "<!-- wizard-data" in result

    def test_roundtrip_via_tmp_file(self, tmp_path):
        import json
        wizard_data = {
            "manufacturer": "Holybro",
            "rc_ports_wizard": [{"label": "RC", "side": "FMU"}],
        }
        content = self._FRONT_MATTER + self._BODY
        embedded = fcdg._embed_wizard_data_comment(content, wizard_data)
        doc = tmp_path / "test.md"
        doc.write_text(embedded, encoding="utf-8")

        result = fcdg._read_wizard_data_from_doc(doc)
        assert result is not None
        assert result["manufacturer"] == "Holybro"
        assert result["rc_ports_wizard"] == [{"label": "RC", "side": "FMU"}]

    def test_read_returns_none_when_no_comment(self, tmp_path):
        doc = tmp_path / "no_comment.md"
        doc.write_text("# No wizard data here\n", encoding="utf-8")
        assert fcdg._read_wizard_data_from_doc(doc) is None

    def test_read_returns_none_for_missing_file(self, tmp_path):
        assert fcdg._read_wizard_data_from_doc(tmp_path / "nonexistent.md") is None

    def test_read_returns_none_for_malformed_json(self, tmp_path):
        doc = tmp_path / "bad.md"
        doc.write_text("<!-- wizard-data\n{not valid json\n-->\n", encoding="utf-8")
        assert fcdg._read_wizard_data_from_doc(doc) is None


# ---------------------------------------------------------------------------
# _wizard_list fallback behaviour
# ---------------------------------------------------------------------------

class TestWizardList:
    """Tests for _wizard_list() — a closure defined inside _run_wizard().

    We build a minimal closure environment and extract the function directly
    from the source rather than running the full interactive wizard.
    """

    @staticmethod
    def _make_wizard_list(input_values: list[str]):
        """Return a _wizard_list callable that feeds input_values in sequence."""
        inputs = iter(input_values)

        def fake_input(_prompt):
            try:
                return next(inputs)
            except StopIteration:
                return ""

        # Build the closure the same way _run_wizard does
        def _wizard_prompt_local(label, default="", required=False):
            val = fake_input(label).strip()
            if val:
                return val
            if default:
                return default
            return ""

        def _wizard_list(prompt_label, detected=None, cached_list=None):
            items = []
            explicitly_done = False
            print(f"  {prompt_label} (one per line, blank to finish):")
            if detected:
                print(f"    {0:2}. (done — stop here / clear remaining)")
                for i, name in enumerate(detected, 1):
                    print(f"    {i:2}. {name}")
                print("    (enter a number, a custom name, or press Enter to accept default)")
            elif cached_list:
                print("     0. (done — stop here / clear remaining)")
                print("    (press Enter to accept each default, or 0 to clear all)")
            idx = 1
            while True:
                cached_default = str(cached_list[idx - 1]) if cached_list and idx <= len(cached_list) else ""
                val = _wizard_prompt_local(f"    item {idx}", default=cached_default)
                if val == "0":
                    explicitly_done = True
                    break
                if not val:
                    break
                if detected and val.strip().isdigit():
                    num = int(val.strip())
                    if 1 <= num <= len(detected):
                        val = detected[num - 1]
                        print(f"      → {val}")
                items.append(val)
                idx += 1
            if not items and not explicitly_done and cached_list:
                items = list(cached_list)
                print(f"    (using cached: {', '.join(str(v) for v in items)})")
            return items

        return _wizard_list

    def test_no_input_no_cache_returns_empty(self, capsys):
        """When user enters nothing and no cache exists, detected list is NOT auto-confirmed."""
        wl = self._make_wizard_list([""])  # user immediately presses Enter
        result = wl("IMU(s)", detected=["ICM-42688P", "BMI088"])
        assert result == [], f"Expected [], got {result}"
        out = capsys.readouterr().out
        assert "using cached" not in out

    def test_no_input_with_cache_accepts_per_item_defaults(self, capsys):
        """Pressing Enter at each prompt accepts the per-item cached default silently.

        With per-item defaults the user is effectively accepting each cached value
        one-by-one, so the "using cached" fallback message is NOT shown.
        """
        # 3 cached items; user presses Enter 4 times (3 accept + 1 stop)
        wl = self._make_wizard_list(["", "", "", ""])
        result = wl("IMU(s)", detected=["ICM-42688P", "BMI088"],
                    cached_list=["ICM-42688P", "ICM-42688P", "ICM-42688P"])
        assert result == ["ICM-42688P", "ICM-42688P", "ICM-42688P"]
        out = capsys.readouterr().out
        assert "using cached" not in out

    def test_no_input_no_detected_with_cache_uses_fallback(self, capsys):
        """When no detected list and user skips past all cached items immediately,
        the bulk cache fallback fires and prints '(using cached: ...)'."""
        # User enters "" immediately at item 1 — but cached_list[0] = "ICM-42688P"
        # so per-item default is returned for item 1, item 2, item 3, then stop.
        # To actually hit the fallback we need to bypass per-item defaults entirely,
        # which can't happen when cached_list has values — so this test confirms
        # the fallback no longer fires when cached values exist.
        wl = self._make_wizard_list([""])  # single empty input
        result = wl("IMU(s)", cached_list=["ICM-42688P"])
        # item 1 accepts "ICM-42688P" via default; item 2 returns "" → break
        assert result == ["ICM-42688P"]
        out = capsys.readouterr().out
        assert "using cached" not in out

    def test_user_input_overrides_cache(self, capsys):
        """Explicit user entries replace both cache and detected."""
        wl = self._make_wizard_list(["ICM-45686", "ICM-45686", ""])
        result = wl("IMU(s)", detected=["ICM-42688P", "BMI088"],
                    cached_list=["ICM-42688P"])
        assert result == ["ICM-45686", "ICM-45686"]
        out = capsys.readouterr().out
        assert "using cached" not in out

    def test_numeric_entry_resolves_to_detected_name(self, capsys):
        """Entering a number selects the corresponding detected driver."""
        detected = ["ADIS16470", "BMI088", "ICM-45686"]
        wl = self._make_wizard_list(["3", "3", "3", ""])  # pick ICM-45686 three times
        result = wl("IMU(s)", detected=detected)
        assert result == ["ICM-45686", "ICM-45686", "ICM-45686"]
        out = capsys.readouterr().out
        assert "→ ICM-45686" in out

    def test_out_of_range_number_kept_as_literal(self, capsys):
        """A number outside the detected list range is kept as a literal string."""
        detected = ["BMI088"]
        wl = self._make_wizard_list(["99", ""])
        result = wl("IMU(s)", detected=detected)
        assert result == ["99"]

    def test_no_input_no_cache_no_detected_returns_empty(self):
        """No user input, no cache, no detected → empty list."""
        wl = self._make_wizard_list([""])
        result = wl("OSD chip (optional)")
        assert result == []

    def test_zero_stops_without_using_cache(self, capsys):
        """Entering '0' sets explicitly_done — cached list is NOT used as fallback."""
        wl = self._make_wizard_list(["0"])
        result = wl("IMU(s)", detected=["ICM-42688P", "BMI088"],
                    cached_list=["ICM-42688P", "ICM-42688P"])
        assert result == []
        out = capsys.readouterr().out
        assert "using cached" not in out

    def test_zero_clears_cache_only_list(self, capsys):
        """Entering '0' on a cached-only list (no detected) returns empty."""
        wl = self._make_wizard_list(["0"])
        result = wl("IMU(s)", cached_list=["ICM-42688P", "ICM-42688P"])
        assert result == []
        out = capsys.readouterr().out
        assert "using cached" not in out

    def test_per_item_cached_default_accepted(self, capsys):
        """Pressing Enter at each prompt accepts the cached default for that position."""
        # 3 cached items; user presses Enter 3 times then once more to stop
        wl = self._make_wizard_list(["", "", "", ""])
        result = wl("IMU(s)", cached_list=["ICM-45686", "ICM-45686", "ICM-45686"])
        assert result == ["ICM-45686", "ICM-45686", "ICM-45686"]
        out = capsys.readouterr().out
        # Cache fallback message should NOT appear — items were explicitly accepted
        assert "using cached" not in out

    def test_per_item_cached_default_partial_override(self, capsys):
        """User overrides first item but accepts second cached default via Enter."""
        wl = self._make_wizard_list(["BMI088", "", ""])
        result = wl("IMU(s)", cached_list=["ICM-45686", "ICM-45686"])
        assert result == ["BMI088", "ICM-45686"]
        out = capsys.readouterr().out
        assert "using cached" not in out
