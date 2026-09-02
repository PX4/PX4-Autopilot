#!/usr/bin/env python3
"""Tests for Tools/serial/generate_config.py protocol_id rules and C walker output."""

import os
import subprocess
import sys
import tempfile
import unittest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'generate_config.py')


def write_yaml(path, body):
    with open(path, 'w') as handle:
        handle.write(body)


def run_generate(config_files, extra_args=None):
    with tempfile.TemporaryDirectory() as tmp:
        params = os.path.join(tmp, 'serial_params.c')
        rc_dir = os.path.join(tmp, 'rc')
        os.mkdir(rc_dir)
        cmd = [
            sys.executable, SCRIPT,
            '--params-file', params,
            '--rc-dir', rc_dir,
            '--serial-ports', 'TEL1:/dev/ttyS6', 'GPS1:/dev/ttyS0',
            '--config-files', *config_files,
        ]
        if extra_args:
            cmd.extend(extra_args)
        proc = subprocess.run(cmd, capture_output=True, text=True)
        header = os.path.join(tmp, 'serial_autostart_config.h')
        rc_serial = os.path.join(rc_dir, 'rc.serial')
        rc_text = header_text = params_text = None
        if os.path.exists(rc_serial):
            with open(rc_serial) as handle:
                rc_text = handle.read()
        if os.path.exists(header):
            with open(header) as handle:
                header_text = handle.read()
        if os.path.exists(params):
            with open(params) as handle:
                params_text = handle.read()
        return proc, rc_text, header_text, params_text


class ProtocolIdTests(unittest.TestCase):
    def test_missing_protocol_id_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            yaml_path = os.path.join(tmp, 'mod.yaml')
            write_yaml(yaml_path, """
module_name: NoId
serial_config:
    - command: foo start -d ${SERIAL_DEV}
""")
            proc, *_ = run_generate([yaml_path])
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn('missing protocol_id', proc.stderr + proc.stdout)

    def test_protocol_id_zero_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            yaml_path = os.path.join(tmp, 'mod.yaml')
            write_yaml(yaml_path, """
module_name: Zero
serial_config:
    - command: foo start -d ${SERIAL_DEV}
      protocol_id: 0
      protocol_name: Zero
""")
            proc, *_ = run_generate([yaml_path])
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn('protocol_id must be > 0', proc.stderr + proc.stdout)

    def test_protocol_id_negative_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            yaml_path = os.path.join(tmp, 'mod.yaml')
            write_yaml(yaml_path, """
module_name: Neg
serial_config:
    - command: foo start -d ${SERIAL_DEV}
      protocol_id: -3
      protocol_name: Neg
""")
            proc, *_ = run_generate([yaml_path])
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn('protocol_id must be > 0', proc.stderr + proc.stdout)

    def test_duplicate_id_different_name_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = os.path.join(tmp, 'a.yaml')
            b = os.path.join(tmp, 'b.yaml')
            write_yaml(a, """
module_name: Alpha
serial_config:
    - command: alpha start -d ${SERIAL_DEV}
      protocol_id: 13
      protocol_name: GHST
""")
            write_yaml(b, """
module_name: Beta
serial_config:
    - command: beta start -d ${SERIAL_DEV}
      protocol_id: 13
      protocol_name: Ghost
""")
            proc, *_ = run_generate([a, b])
            self.assertNotEqual(proc.returncode, 0)
            combined = proc.stderr + proc.stdout
            self.assertIn('duplicate protocol_id 13', combined)
            self.assertIn('GHST', combined)
            self.assertIn('Ghost', combined)

    def test_duplicate_id_same_name_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = os.path.join(tmp, 'a.yaml')
            b = os.path.join(tmp, 'b.yaml')
            write_yaml(a, """
module_name: GHST RC Input Driver
serial_config:
    - command: "ghst_rc start -d ${SERIAL_DEV}"
      protocol_id: 13
      protocol_name: GHST
""")
            write_yaml(b, """
module_name: GHST RC Input Driver
serial_config:
    - command: "ghst_rc start -d ${SERIAL_DEV}"
      protocol_id: 13
      protocol_name: GHST
""")
            proc, rc_text, header_text, params_text = run_generate([a, b])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertEqual(header_text.count('13,'), 1)
            self.assertEqual(params_text.count('@value 13 GHST'), 2)  # once per port


class AutostartOutputTests(unittest.TestCase):
    def test_rc_serial_is_one_liner(self):
        with tempfile.TemporaryDirectory() as tmp:
            yaml_path = os.path.join(tmp, 'mod.yaml')
            write_yaml(yaml_path, """
module_name: GPS
serial_config:
    - command: gps start -d ${SERIAL_DEV} -b p:${BAUD_PARAM} ${DUAL_GPS_ARGS}
      secondary_command: set DUAL_GPS_ARGS "-e ${SERIAL_DEV} -g p:${BAUD_PARAM}"
      protocol_id: 5
      protocol_name: GPS
      default: GPS1
""")
            proc, rc_text, header_text, params_text = run_generate([yaml_path])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertEqual(rc_text.strip(), 'serial_autostart')
            self.assertNotIn('param compare', rc_text)
            self.assertIn('SER_TEL1_PROTO', params_text)
            self.assertIn('kSerialKindCollect', header_text)
            self.assertIn('gps start -d ${SERIAL_DEV}', header_text)
            self.assertIn('-e ${SERIAL_DEV} -g p:${BAUD_PARAM}', header_text)
            self.assertIn('SER_TEL1_PROTO', header_text)
            self.assertIn('/dev/ttyS6', header_text)
            self.assertIn('collect_rank', header_text)

    def test_gps_collect_rank_prefers_gps_tags(self):
        with tempfile.TemporaryDirectory() as tmp:
            yaml_path = os.path.join(tmp, 'mod.yaml')
            write_yaml(yaml_path, """
module_name: GPS
serial_config:
    - command: gps start -d ${SERIAL_DEV} -b p:${BAUD_PARAM} ${DUAL_GPS_ARGS}
      secondary_command: set DUAL_GPS_ARGS "-e ${SERIAL_DEV} -g p:${BAUD_PARAM}"
      protocol_id: 5
      protocol_name: GPS
      default: GPS1
""")
            proc, _, header_text, _ = run_generate(
                [yaml_path],
                extra_args=['--serial-ports', 'TEL2:/dev/ttyS4', 'GPS1:/dev/ttyS0', 'GPS2:/dev/ttyS1'])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn('"GPS1"', header_text)
            gps1 = header_text.find('"GPS1"')
            gps2 = header_text.find('"GPS2"')
            tel2 = header_text.find('"TEL2"')
            self.assertGreater(gps1, 0)
            self.assertGreater(gps2, gps1)
            # collect_rank 0/1/2 for GPS tags, 255 for TEL2
            self.assertRegex(header_text[gps1:gps1 + 120], r',\s*0\s*\}')
            self.assertRegex(header_text[gps2:gps2 + 120], r',\s*1\s*\}')
            self.assertRegex(header_text[tel2:tel2 + 120], r',\s*255\s*\}')

    def test_board_with_io_skips_rc_sbus_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            yaml_path = os.path.join(tmp, 'mod.yaml')
            write_yaml(yaml_path, """
module_name: SBUS RC Input Driver
serial_config:
    - command: "sbus_rc start -d ${SERIAL_DEV}"
      protocol_id: 10
      protocol_name: SBUS
      default: RC
""")
            proc, _, _, params_text = run_generate(
                [yaml_path],
                extra_args=['--serial-ports', 'RC:/dev/ttyS5', '--board-with-io'])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn('PARAM_DEFINE_INT32(SER_RC_PROTO, 0)', params_text)

    def test_iridium_success_command_in_header(self):
        with tempfile.TemporaryDirectory() as tmp:
            yaml_path = os.path.join(tmp, 'mod.yaml')
            write_yaml(yaml_path, """
module_name: Iridium
serial_config:
    - command: usleep 200000; iridiumsbd start -d ${SERIAL_DEV}
      success_command: mavlink start -d /dev/iridium -m iridium -b 115200
      fail_command: tune_control play error
      protocol_id: 23
      protocol_name: Iridium
""")
            proc, _, header_text, _ = run_generate([yaml_path])
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertIn('iridiumsbd start -d ${SERIAL_DEV}', header_text)
            self.assertIn('mavlink start -d /dev/iridium', header_text)
            self.assertIn('tune_control play error', header_text)
            self.assertNotIn('if iridiumsbd', header_text)


class MetadataTests(unittest.TestCase):
    def test_serial_json_lists_ports_in_instance_order(self):
        import json
        with tempfile.TemporaryDirectory() as tmp:
            yaml_path = os.path.join(tmp, 'mod.yaml')
            write_yaml(yaml_path, """
module_name: mavlink
serial_config:
    - command: mavlink start -d ${SERIAL_DEV}
      protocol_id: 1
      protocol_name: MAVLink
      num_instances: 3
      ethernet_param: MAV_ETH_EN
parameters:
    - group: MAVLink
      definitions:
        MAV_${i}_MODE:
            type: enum
            num_instances: 3
            default: [0, 2, 0]
        MAV_ETH_EN:
            type: boolean
            default: false
""")
            out = os.path.join(tmp, 'serial.json')
            cmd = [sys.executable, SCRIPT, '--metadata-file', out, '--compress', '--ethernet',
                   '--serial-ports', 'GPS1:/dev/ttyS0', 'TEL1:/dev/ttyS6',
                   '--config-files', yaml_path]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertTrue(os.path.exists(out + '.xz'))
            with open(out) as handle:
                serial = json.load(handle)['buses']['serial']
            self.assertEqual([p['id'] for p in serial['ports']], ['TEL1', 'GPS1'])
            self.assertEqual(serial['ports'][0]['protocolParam'], 'SER_TEL1_PROTO')
            self.assertEqual(serial['ports'][0]['baudParam'], 'SER_TEL1_BAUD')
            self.assertTrue(serial['ethernet'])
            mavlink = [p for p in serial['protocols'] if p['id'] == 1][0]
            self.assertEqual(mavlink['maxPorts'], 3)
            self.assertEqual(mavlink['instanceParams'], ['MAV_${i}_MODE'])
            self.assertEqual(mavlink['ethernetParam'], 'MAV_ETH_EN')
            self.assertEqual(serial['protocols'][0], {'id': 0, 'name': 'Disabled'})


if __name__ == '__main__':
    unittest.main()
