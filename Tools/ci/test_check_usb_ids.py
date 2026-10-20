"""Check check_usb_ids.py against a fixture registry and fake board trees.

The checker gates every PR that touches a board defconfig, so a false
pass lets a board ship a PID the registry never gave it, and a false
failure or traceback blocks unrelated board work.
"""

import contextlib
import io
from pathlib import Path
import tempfile
from typing import List, Tuple
import unittest

from check_usb_ids import cmd_check, cmd_lookup, load_registry

REGISTRY = """
vid: "0x3643"
vendor_string: "Dronecode Project, Inc."
manufacturers:
  - name: PX4/Dronecode
    px4_vendor: px4
    pids:
      - pid: "0x001D"
        board: FMU-v6XRT
        px4_board: px4/fmu-v6xrt
      - pid: "0x001E"
        board: Unmapped
  - name: SIYI
    px4_vendor: siyi
    pids:
      - pid: "0x04B0"
        board: SIYI N7
        px4_board: siyi/n7
"""


class CheckUsbIdsTest(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        registry = self.root / "usb-ids.yaml"
        registry.write_text(REGISTRY)
        self.registry = load_registry(str(registry))

    def defconfig(self, board: str, *lines: str) -> str:
        path = self.root.joinpath(
            "boards", board, "nuttx-config", "nsh", "defconfig")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("".join(f"{line}\n" for line in lines))
        return str(path)

    def check(self, *paths: str) -> Tuple[int, List[str]]:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = cmd_check(self.registry, list(paths))
        return rc, err.getvalue().splitlines()

    def assert_one_error(self, path: str, expected: str) -> None:
        rc, errors = self.check(path)
        self.assertEqual(rc, 1)
        self.assertEqual(len(errors), 1, errors)
        self.assertIn(expected, errors[0])

    def test_registered_and_mapped_passes(self) -> None:
        path = self.defconfig("px4/fmu-v6xrt",
                              "CONFIG_CDCACM_VENDORID=0x3643",
                              "CONFIG_CDCACM_PRODUCTID=0x001D")
        self.assertEqual(self.check(path), (0, []))

    def test_pid_compare_is_case_insensitive(self) -> None:
        # the registry spells PIDs uppercase, defconfigs often do not
        path = self.defconfig("px4/fmu-v6xrt",
                              "CONFIG_CDCACM_VENDORID=0x3643",
                              "CONFIG_CDCACM_PRODUCTID=0x001d")
        self.assertEqual(self.check(path), (0, []))

    def test_custom_vendor_string_is_ignored(self) -> None:
        # the registry does not govern the vendor string
        path = self.defconfig("siyi/n7",
                              "CONFIG_CDCACM_VENDORID=0x3643",
                              "CONFIG_CDCACM_PRODUCTID=0x04B0",
                              'CONFIG_CDCACM_VENDORSTR='
                              '"Totally Weird Widgets GmbH"')
        self.assertEqual(self.check(path), (0, []))

    def test_unregistered_pid_fails(self) -> None:
        path = self.defconfig("px4/fmu-v6xrt",
                              "CONFIG_CDCACM_VENDORID=0x3643",
                              "CONFIG_CDCACM_PRODUCTID=0x0999")
        self.assert_one_error(path, "PID 0x0999 is not registered")

    def test_pid_mapped_to_another_board_fails(self) -> None:
        # a vendor reusing its PID for a second board
        path = self.defconfig("siyi/n8",
                              "CONFIG_CDCACM_VENDORID=0x3643",
                              "CONFIG_CDCACM_PRODUCTID=0x04B0")
        self.assert_one_error(
            path,
            "mapped to boards/siyi/n7/ in the registry, not boards/siyi/n8/")

    def test_missing_px4_board_fails(self) -> None:
        path = self.defconfig("px4/other",
                              "CONFIG_CDCACM_VENDORID=0x3643",
                              "CONFIG_CDCACM_PRODUCTID=0x001E")
        self.assert_one_error(
            path,
            'has no px4_board in the registry; set it to "px4/other"')

    def test_malformed_hex_is_a_one_line_error(self) -> None:
        cases = {
            "vid": ("CONFIG_CDCACM_VENDORID=0x36G3",
                    "CONFIG_CDCACM_PRODUCTID=0x001D"),
            "pid": ("CONFIG_CDCACM_VENDORID=0x3643",
                    "CONFIG_CDCACM_PRODUCTID=0xZZ01"),
        }
        for name, lines in cases.items():
            with self.subTest(name):
                path = self.defconfig(f"bad/{name}", *lines)
                self.assert_one_error(path, "is not a hex value")

    def test_other_vendor_id_is_ignored(self) -> None:
        path = self.defconfig("acme/fc",
                              "CONFIG_CDCACM_VENDORID=0x1209",
                              "CONFIG_CDCACM_PRODUCTID=0x0999")
        self.assertEqual(self.check(path), (0, []))

    def lookup(self, query: str) -> Tuple[int, str]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = cmd_lookup(self.registry, query)
        return rc, out.getvalue().strip()

    def test_lookup_by_pid(self) -> None:
        self.assertEqual(self.lookup("0x04b0"),
                         (0, "0x04b0: SIYI, board: SIYI N7"))
        self.assertEqual(self.lookup("0x0999"), (1, "0x0999: not registered"))

    def test_lookup_by_board_path(self) -> None:
        self.assertEqual(
            self.lookup("boards/px4/fmu-v6xrt"),
            (0, "boards/px4/: PX4/Dronecode, PIDs: 0x001D, 0x001E"))
        self.assertEqual(self.lookup("boards/acme/fc")[0], 1)


if __name__ == "__main__":
    unittest.main()
