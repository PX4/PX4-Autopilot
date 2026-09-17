#!/usr/bin/env python3
"""Exercise generated UART startup policy with fake parameter and ioctl commands."""
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

GENERATOR = Path(__file__).resolve().parents[1] / "generate_config.py"


class SerialConfigTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def generate(self, enabled):
        args = [sys.executable, str(GENERATOR), "--rc-dir", str(self.root),
                "--params-file", str(self.root / "params.c"),
                "--serial-ports", "GPS1:/dev/ttyS3", "RC:/dev/ttyS4"]
        if enabled:
            args.append("--serial-config")
        subprocess.run(args, check=True)
        return (self.root / "rc.serial_port").read_text()

    def run_port(self, script, swap, fail=False, claimed=False,
                 port_config="GPS_1_CONFIG", port_id=201):
        # NSH 'set NAME value' assigns an environment variable; translate only
        # that syntax so the generated control flow runs in a host POSIX shell.
        script = re.sub(r"(?m)^(\s*)set (\w+) (.+)$", r'\1\2=\3', script)
        test_script = """
param() {
    case "$2" in
        GPS_1_CONFIG|MAV_0_CONFIG) [ "$3" = "$PORT_ID" ] ;;
        SER_GPS1_SWAP|SER_RC_SWAP) [ "$SWAP" = "$3" ] ;;
        *) return 1 ;;
    esac
}
serial_config() {
    echo "ioctl:$*"
    [ "$FAIL" = 0 ]
}
""" + f"PRT={port_config}\n" + ("PRT_GPS1_=1\n" if claimed else "") + script + """
echo "selected:$SERIAL_DEV"
if [ "$SERIAL_DEV" != none ]; then echo service-start; fi
"""
        env = dict(os.environ, SWAP=str(swap), FAIL=str(int(fail)), PORT_ID=str(port_id))
        return subprocess.check_output(["sh", "-c", test_script], env=env, text=True)

    def test_feature_omitted_when_not_built(self):
        script = self.generate(False)
        self.assertNotIn("serial_config -d", script)
        self.assertNotIn("_SWAP", (self.root / "params.c").read_text())
        self.assertIn("service-start", self.run_port(script, 1, fail=True))

    def test_default_does_not_touch_uart(self):
        output = self.run_port(self.generate(True), 0)
        self.assertNotIn("ioctl:", output)
        self.assertIn("selected:/dev/ttyS3", output)
        self.assertIn("service-start", output)

    def test_swap_precedes_service_start(self):
        output = self.run_port(self.generate(True), 1)
        self.assertIn("ioctl:-d /dev/ttyS3 -s", output)
        self.assertLess(output.index("ioctl:"), output.index("service-start"))

    def test_failed_swap_prevents_service_start(self):
        output = self.run_port(self.generate(True), 1, fail=True)
        self.assertIn("selected:none", output)
        self.assertNotIn("service-start", output)

    def test_mavlink_uses_assigned_port_setting(self):
        output = self.run_port(self.generate(True), 1,
                               port_config="MAV_0_CONFIG", port_id=300)
        self.assertIn("ioctl:-d /dev/ttyS4 -s", output)
        self.assertNotIn("ioctl:-d /dev/ttyS3", output)
        self.assertIn("service-start", output)

    def test_claimed_port_is_not_reconfigured(self):
        output = self.run_port(self.generate(True), 1, claimed=True)
        self.assertNotIn("ioctl:", output)
        self.assertIn("selected:none", output)

    def test_parameters_follow_board_ports(self):
        self.generate(True)
        params = (self.root / "params.c").read_text()
        self.assertIn("PARAM_DEFINE_INT32(SER_GPS1_SWAP, 0)", params)
        self.assertIn("PARAM_DEFINE_INT32(SER_RC_SWAP, 0)", params)
        self.assertNotIn("SER_TEL1_SWAP", params)
        for name in ("GPS1", "RC"):
            definition = params.split(f"PARAM_DEFINE_INT32(SER_{name}_SWAP")[0]
            self.assertIn("@reboot_required true", definition.rsplit("/**", 1)[1])


if __name__ == "__main__":
    unittest.main()
