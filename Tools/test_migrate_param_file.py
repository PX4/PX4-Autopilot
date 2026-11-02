#!/usr/bin/env python3
"""Tests for migrate_param_file.py."""

import os
import unittest

import migrate_param_file as m

TRANSLATION = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                           'src', 'lib', 'parameters', 'param_translation.cpp')

HEADER = ['# Onboard parameters for Vehicle 1\n', '#\n',
          '# Version: 1.17.0 \n', '#\n',
          '# Vehicle-Id Component-Id Name Value Type\n']


def params_file(*rows):
    return HEADER + [f'1\t{c}\t{n}\t{v}\t{t}\n' for c, n, v, t in rows]


def migrate(*rows):
    log = []
    lines = m.migrate(params_file(*rows), log.append)
    values = {}
    for line in lines:
        f = line.rstrip('\n').split('\t')
        if not line.startswith('#'):
            values[(int(f[1]), f[2])] = (f[3], int(f[4]))
    return lines, values, log


class MigrateParamFileTest(unittest.TestCase):

    def test_conversions(self):
        lines, values, _ = migrate(
            (1, 'EKF2_GPS_DELAY', '150.000000000000000000', 9),
            (1, 'EKF2_GPS_POS_X', '0.119999997317790985', 9),
            (1, 'MNT_RANGE_PITCH', '90.000000000000000000', 9),
            (1, 'MOT_POLE_COUNT', '12', 6),
            (1, 'COM_RC_OVERRIDE', '0', 6),
            (1, 'MPC_LAND_RC_HELP', '1', 6),
            (1, 'SENS_IMU_TEMP', '45.000000000000000000', 9),
            (1, 'SENS_TEMP_ID', '0', 6),
            (1, 'MPC_XY_VEL_MAX', '8.000000000000000000', 9))
        self.assertEqual(lines[:len(HEADER)], HEADER)
        self.assertEqual(values, {
            (1, 'SENS_GPS0_DELAY'): ('150', 6),
            (1, 'SENS_GPS1_DELAY'): ('150', 6),
            (1, 'SENS_GPS0_OFFX'): ('0.119999997317790985', 9),
            (1, 'MNT_MAX_PITCH'): ('45.000000000000000000', 9),
            (1, 'MNT_MIN_PITCH'): ('-45.000000000000000000', 9),
            **{(1, f'DSHOT_MOT_POL{i}'): ('12', 6) for i in range(1, 13)},
            (1, 'MAN_OVERRIDE_SPD'): ('-1.000000000000000000', 9),
            (1, 'MPC_AUTO_NUDGING'): ('2', 6),
            (1, 'HEATER1_TEMP'): ('45.000000000000000000', 9),
            (1, 'HEATER1_SENS_ID'): ('-1', 6),
            (1, 'MPC_XY_VEL_MAX'): ('8.000000000000000000', 9),
        })

    def test_values_firmware_does_not_convert(self):
        _, values, log = migrate(
            (1, 'COM_RC_OVERRIDE', '1', 6),
            (1, 'MNT_RANGE_PITCH', '0.000000000000000000', 9),
            (1, 'MPC_LAND_RC_HELP', '0', 6))
        self.assertEqual(values, {})
        self.assertIn('converted: COM_RC_OVERRIDE 1 -> nothing to set', log)

    def test_heater_sensor_id_kept(self):
        _, values, _ = migrate((1, 'SENS_TEMP_ID', '2621474', 6))
        self.assertEqual(values, {(1, 'HEATER1_SENS_ID'): ('2621474', 6)})

    def test_landing_nudging_merged_into_existing_bits(self):
        _, values, _ = migrate((1, 'MPC_AUTO_NUDGING', '1', 6),
                               (1, 'MPC_LAND_RC_HELP', '1', 6))
        self.assertEqual(values, {(1, 'MPC_AUTO_NUDGING'): ('3', 6)})

    def test_rc_reverse_written_as_integer(self):
        _, values, log = migrate((1, 'RC1_REV', '-1.000000000000000000', 9),
                                 (1, 'RC2_REV', '1.000000000000000000', 9),
                                 (1, 'RC3_REV', '-1', 6))
        self.assertEqual(values, {(1, 'RC1_REV'): ('-1', 6),
                                  (1, 'RC2_REV'): ('1', 6),
                                  (1, 'RC3_REV'): ('-1', 6)})
        self.assertEqual(log, ['reversed: RC1_REV written as the integer -1'])

    def test_removed_dropped_and_listed(self):
        _, values, log = migrate((1, 'COM_RC_STICK_OV', '30.0', 9),
                                 (1, 'FW_AT_START', '0', 6))
        self.assertEqual(values, {})
        self.assertEqual(log, [
            'removed: COM_RC_STICK_OV 30: see MAN_OVERRIDE_SPD'
            ' (stick speed in 1/s)',
            'removed: FW_AT_START 0'])

    def test_old_default_kept_and_listed(self):
        _, values, log = migrate((1, 'FD_ACT_EN', '1', 6),
                                 (1, 'SENS_MAG_AUTOCAL', '0', 6))
        self.assertEqual(values, {(1, 'FD_ACT_EN'): ('1', 6),
                                  (1, 'SENS_MAG_AUTOCAL'): ('0', 6)})
        self.assertEqual(
            log, ['default: FD_ACT_EN 1 is a v1.17 default, v1.18 uses 0'])

    def test_other_components_untouched(self):
        _, values, log = migrate((154, 'EKF2_GPS_DELAY', '150.0', 9))
        self.assertEqual(values, {(154, 'EKF2_GPS_DELAY'): ('150.0', 9)})
        self.assertEqual(log, [])

    def test_other_lines_copied_and_listed(self):
        log = []
        lines = m.migrate(HEADER + ['EKF2_GPS_DELAY,150\n'], log.append)
        self.assertEqual(lines, HEADER + ['EKF2_GPS_DELAY,150\n'])
        self.assertEqual(log, ['copied unchanged, not in the QGroundControl'
                               ' format: EKF2_GPS_DELAY,150'])

    def test_every_written_parameter_has_a_type(self):
        for kind, new, _ in m.CONVERSIONS.values():
            for name in (new,) if isinstance(new, str) else new:
                self.assertIn(name, m.NEW_TYPES)

    def test_firmware_translates_every_converted_parameter(self):
        with open(TRANSLATION, encoding='utf-8') as f:
            source = f.read()
        for old in m.CONVERSIONS:
            self.assertIn(f'"{old}"', source)


if __name__ == '__main__':
    unittest.main()
