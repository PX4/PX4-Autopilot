#!/usr/bin/env python3
"""Offline regressions: python3 -m unittest discover -s Tools/ci -p test_check_board_ids.py"""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from urllib.error import URLError

import check_board_ids as checker


REGISTRY = """\
# Board identities, including an intentional alias and a quoted reservation.
AP_HW_MATEKH743 1013
AP_HW_DURANDAL 139
TARGET_HW_PX4_FMU_V2 9
TARGET_HW_PX4_FMU_V3 9 # same as FMU_V2
Reserved "NXP MR-TROPIC" 37
"""


class BoardIdsTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.registry = checker.parse_registry(REGISTRY)
        self.policy = {
            "registry_revision": "a" * 40,
            "boards": {"matek/h743": "AP_HW_MATEKH743"},
            "legacy_boards": {},
            "bootloader_exceptions": {},
        }
        self.board = self.add_board("matek/h743", 1013, 1013)

    def add_board(self, name, board_id, board_type):
        board = self.root / "boards" / name
        board.mkdir(parents=True, exist_ok=True)
        (board / "firmware.prototype").write_text(json.dumps({"board_id": board_id}), encoding="utf-8")
        if board_type is not None:
            (board / "src").mkdir(exist_ok=True)
            (board / "src/hw_config.h").write_text(f"#define BOARD_TYPE {board_type}\n", encoding="utf-8")
            (board / "bootloader.px4board").touch()
        return board

    def check(self):
        return checker.check_boards(self.root, self.policy, self.registry)

    def run_main(self, *arguments, offline=True):
        policy_file = self.root / "board_ids.json"
        policy_file.write_text(json.dumps(self.policy), encoding="utf-8")
        registry_file = self.root / "board_types.txt"
        registry_file.write_text(REGISTRY, encoding="utf-8")
        argv = ["check_board_ids.py", "--root", str(self.root), *arguments]
        if offline:
            argv.extend(["--registry", str(registry_file)])
        output = io.StringIO()
        with patch.object(checker, "POLICY", policy_file), patch("sys.argv", argv):
            with redirect_stdout(output), redirect_stderr(output):
                status = checker.main()
        return status, output.getvalue()

    def test_matching_ids(self):
        self.assertEqual(self.check(), ([], []))

    def test_firmware_mismatch(self):
        self.add_board("matek/h743", 139, 1013)
        errors, _ = self.check()
        self.assertTrue(any("board_id=139, expected 1013" in error for error in errors))

    def test_bootloader_mismatch(self):
        self.add_board("matek/h743", 1013, 139)
        errors, _ = self.check()
        self.assertTrue(any("BOARD_TYPE=139, expected 1013" in error for error in errors))

    def test_matching_but_wrong_ids(self):
        self.add_board("matek/h743", 139, 139)
        errors, _ = self.check()
        self.assertEqual(len(errors), 2)

    def test_pr_28654_is_rejected_despite_legacy_exception(self):
        policy = checker.load_json(checker.POLICY)
        self.policy["boards"]["matek/h743"] = policy["boards"]["matek/h743"]
        self.policy["bootloader_exceptions"]["matek/h743"] = {
            "board_type": 139, "reason": "Historical bootloader mismatch from PX4-Autopilot#27731"
        }
        self.add_board("matek/h743", 1013, 139)
        errors, warnings = self.check()
        self.assertEqual(errors, [])
        self.assertEqual(len(warnings), 1)
        self.add_board("matek/h743", 139, 139)
        status, output = self.run_main()
        self.assertEqual(status, 1)
        self.assertIn("board_id=139, expected 1013", output)

    def test_fixed_bootloader_requires_exception_removal(self):
        self.policy["bootloader_exceptions"]["matek/h743"] = {"board_type": 139, "reason": "Existing mismatch"}
        self.assertTrue(self.check()[0])
        self.policy["bootloader_exceptions"].clear()
        self.assertEqual(self.check(), ([], []))

    def test_exception_does_not_allow_arbitrary_bootloader_ids(self):
        self.policy["bootloader_exceptions"]["matek/h743"] = {"board_type": 139, "reason": "Existing mismatch"}
        self.add_board("matek/h743", 1013, 9)
        self.assertTrue(self.check()[0])

    def test_external_bootloader(self):
        (self.board / "src/hw_config.h").unlink()
        (self.board / "bootloader.px4board").unlink()
        self.assertEqual(self.check(), ([], []))
        self.add_board("matek/h743", 139, None)
        self.assertTrue(self.check()[0])

    def test_in_tree_bootloader_requires_header(self):
        (self.board / "src/hw_config.h").unlink()
        self.assertTrue(any("missing bootloader configuration" in error for error in self.check()[0]))

    def test_bootloader_target_requires_prototype(self):
        self.add_board("test/new", 1013, 1013)
        (self.root / "boards/test/new/firmware.prototype").unlink()
        self.assertTrue(any("missing firmware.prototype" in error for error in self.check()[0]))

    def test_explicit_shared_identity(self):
        self.add_board("matek/h743-slim", 1013, 1013)
        self.assertTrue(self.check()[0])
        self.policy["boards"]["matek/h743-slim"] = "AP_HW_MATEKH743"
        self.assertEqual(self.check(), ([], []))

    def test_unknown_registry_name(self):
        self.policy["boards"]["matek/h743"] = "UNREGISTERED"
        self.assertTrue(any("missing from the pinned registry" in error for error in self.check()[0]))

    def test_legacy_values_are_exact(self):
        self.policy["boards"].clear()
        self.policy["legacy_boards"]["matek/h743"] = {
            "board_id": 1013, "board_type": 1013, "reason": "Existing unconfirmed identity"
        }
        self.assertEqual(self.check()[0], [])
        self.add_board("matek/h743", 139, 139)
        self.assertTrue(any("legacy IDs changed" in error for error in self.check()[0]))

    def test_legacy_header_removal_is_rejected(self):
        self.policy["boards"].clear()
        self.policy["legacy_boards"]["matek/h743"] = {
            "board_id": 1013, "board_type": 1013, "reason": "Existing unconfirmed identity"
        }
        (self.board / "src/hw_config.h").unlink()
        (self.board / "bootloader.px4board").unlink()
        self.assertTrue(self.check()[0])

    def test_stale_or_overlapping_policy(self):
        self.policy["boards"]["missing/board"] = "AP_HW_DURANDAL"
        self.assertTrue(any("stale" in error for error in self.check()[0]))
        self.policy["legacy_boards"]["matek/h743"] = {
            "board_id": 1013, "board_type": 1013, "reason": "Existing"
        }
        self.assertTrue(any("both boards and legacy_boards" in error for error in self.check()[0]))

    def test_invalid_firmware_ids(self):
        for value in (True, "1013", 1013.0, 0, -1, 2**32):
            with self.subTest(value=value):
                self.add_board("matek/h743", value, 1013)
                self.assertTrue(self.check()[0])

    def test_missing_or_duplicate_json_keys(self):
        for text in ("{}", '{"board_id":1013,"board_id":139}', "not json"):
            with self.subTest(text=text):
                (self.board / "firmware.prototype").write_text(text, encoding="utf-8")
                self.assertTrue(self.check()[0])

    def test_unsupported_macro_is_not_silently_skipped(self):
        for definition in (
            "", "#define BOARD_TYPE OTHER_ID", "#define BOARD_TYPE (1000 + 13)",
            "#define BOARD_TYPE 01013", "#define BOARD_TYPE 1013\n#define BOARD_TYPE 139",
        ):
            with self.subTest(definition=definition):
                (self.board / "src/hw_config.h").write_text(definition, encoding="utf-8")
                self.assertTrue(self.check()[0])

    def test_hex_and_comments(self):
        (self.board / "src/hw_config.h").write_text(
            "/* #define BOARD_TYPE 139 */\n#define BOARD_TYPE 0x3f5 // Matek\n", encoding="utf-8"
        )
        self.assertEqual(self.check(), ([], []))

    def test_registry_aliases_and_reservations(self):
        self.assertEqual(self.registry["TARGET_HW_PX4_FMU_V2"], self.registry["TARGET_HW_PX4_FMU_V3"])
        self.assertEqual(self.registry['Reserved "NXP MR-TROPIC"'], 37)

    def test_invalid_registry(self):
        for text in ("", "<html>Not found</html>", "BOARD 1\nBOARD 2"):
            with self.subTest(text=text), self.assertRaises(ValueError):
                checker.parse_registry(text)

    def test_empty_checkout_fails(self):
        (self.board / "firmware.prototype").unlink()
        self.assertTrue(self.check()[0])

    def test_offline_cli_does_not_access_network(self):
        with patch.object(checker, "urlopen") as fetch:
            self.assertEqual(self.run_main()[0], 0)
            fetch.assert_not_called()

    def test_strict_cli_fails_on_known_inconsistency(self):
        self.policy["bootloader_exceptions"]["matek/h743"] = {"board_type": 139, "reason": "Existing mismatch"}
        self.add_board("matek/h743", 1013, 139)
        self.assertEqual(self.run_main()[0], 0)
        self.assertEqual(self.run_main("--strict")[0], 1)

    def test_network_failure_is_not_success(self):
        with patch.object(checker, "urlopen", side_effect=URLError("Registry unavailable")):
            status, output = self.run_main(offline=False)
        self.assertEqual(status, 1)
        self.assertIn("Registry unavailable", output)

    def test_download_uses_pinned_revision(self):
        with patch.object(checker, "urlopen", return_value=io.BytesIO(REGISTRY.encode())) as fetch:
            self.assertEqual(self.run_main(offline=False)[0], 0)
        fetch.assert_called_once_with(
            f"https://raw.githubusercontent.com/PX4/PX4-Bootloader/{'a' * 40}/board_types.txt",
            timeout=30,
        )

    def test_unpinned_registry_is_rejected(self):
        self.policy["registry_revision"] = "main"
        with patch.object(checker, "urlopen") as fetch:
            self.assertEqual(self.run_main(offline=False)[0], 1)
            fetch.assert_not_called()


if __name__ == "__main__":
    unittest.main()
