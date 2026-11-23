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
            (1, 'MPC_XY_VEL_MAX'): ('8.000000000000000000', 9),
        })

    def test_values_firmware_does_not_convert(self):
        _, values, log = migrate(
            (1, 'COM_RC_OVERRIDE', '1', 6),
            (1, 'MNT_RANGE_PITCH', '0.000000000000000000', 9),
            (1, 'MPC_LAND_RC_HELP', '0', 6),
            (1, 'SENS_TEMP_ID', '0', 6))
        self.assertEqual(values, {})
        self.assertIn('converted: COM_RC_OVERRIDE 1 -> nothing to set', log)
        self.assertIn('converted: SENS_TEMP_ID 0 -> nothing to set', log)

    def test_heater_sensor_id_kept(self):
        _, values, log = migrate((1, 'SENS_TEMP_ID', '2621474', 6))
        self.assertEqual(values, {(1, 'HEATER1_SENS_ID'): ('2621474', 6)})
        self.assertEqual(log, ['converted: SENS_TEMP_ID 2621474 ->'
                               ' HEATER1_SENS_ID 2621474'])

    def test_aux_global_position_moved_to_first_source(self):
        _, values, log = migrate(
            (1, 'EKF2_AGP_CTRL', '1', 6),
            (1, 'EKF2_AGP_MODE', '1', 6),
            (1, 'EKF2_AGP_DELAY', '50.000000000000000000', 9),
            (1, 'EKF2_AGP_NOISE', '0.899999976158142090', 9),
            (1, 'EKF2_AGP_GATE', '3.000000000000000000', 9))
        self.assertEqual(values, {
            (1, 'EKF2_AGP0_CTRL'): ('1', 6),
            (1, 'EKF2_AGP0_ID'): ('1', 6),
            (1, 'EKF2_AGP0_MODE'): ('1', 6),
            (1, 'EKF2_AGP0_DELAY'): ('50.000000000000000000', 9),
            (1, 'EKF2_AGP0_NOISE'): ('0.899999976158142090', 9),
            (1, 'EKF2_AGP0_GATE'): ('3.000000000000000000', 9),
        })
        self.assertIn('converted: EKF2_AGP_CTRL 1 ->'
                      ' EKF2_AGP0_CTRL 1, EKF2_AGP0_ID 1', log)
        self.assertIn('default: EKF2_AGP_NOISE 0.9 is a v1.17 default,'
                      ' v1.18 uses 1', log)

    def test_aux_global_position_id_kept_when_disabled(self):
        _, values, log = migrate((1, 'EKF2_AGP_CTRL', '0', 6))
        self.assertEqual(values, {(1, 'EKF2_AGP0_CTRL'): ('0', 6)})
        self.assertEqual(log, ['converted: EKF2_AGP_CTRL 0 ->'
                               ' EKF2_AGP0_CTRL 0'])

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

    def test_roboclaw_defaults_replaced(self):
        _, values, log = migrate((1, 'RBCLW_DIS1', '128', 6),
                                 (1, 'RBCLW_MAX1', '256', 6),
                                 (1, 'RBCLW_MIN1', '1', 6),
                                 (1, 'RBCLW_FAIL1', '-1', 6))
        self.assertEqual(values, {(1, 'RBCLW_DIS1'): ('127', 6),
                                  (1, 'RBCLW_MAX1'): ('254', 6),
                                  (1, 'RBCLW_MIN1'): ('0', 6),
                                  (1, 'RBCLW_FAIL1'): ('-1', 6)})
        self.assertEqual(log, [
            'replaced: RBCLW_DIS1 128 -> 127, the v1.17 default by the'
            ' v1.18 default',
            'replaced: RBCLW_MAX1 256 -> 254, the v1.17 default by the'
            ' v1.18 default',
            'replaced: RBCLW_MIN1 1 -> 0, the v1.17 default by the'
            ' v1.18 default'])

    def test_roboclaw_set_values_kept_and_listed(self):
        _, values, log = migrate((1, 'RBCLW_MAX2', '200', 6),
                                 (1, 'RBCLW_FAIL2', '128', 6))
        self.assertEqual(values, {(1, 'RBCLW_MAX2'): ('200', 6),
                                  (1, 'RBCLW_FAIL2'): ('128', 6)})
        note = ('RoboClaw outputs use 0 to 254 with stop at 127 in v1.18'
                ' (v1.17: 0 to 256, stop at 128)')
        self.assertEqual(log, [f'check: RBCLW_MAX2 200: {note}',
                               f'check: RBCLW_FAIL2 128: {note}'])

    def test_vehicle_type_default_listed_with_its_scope(self):
        _, values, log = migrate(
            (1, 'EKF2_REQ_PDOP', '2.500000000000000000', 9))
        self.assertEqual(values,
                         {(1, 'EKF2_REQ_PDOP'): ('2.500000000000000000', 9)})
        self.assertEqual(log, ['default: EKF2_REQ_PDOP 2.5 is a v1.17 default,'
                               ' v1.18 uses 4 on fixed-wing'])

    def test_other_components_untouched(self):
        _, values, log = migrate((1, 'MPC_XY_VEL_MAX', '8.0', 9),
                                 (154, 'EKF2_GPS_DELAY', '150.0', 9))
        self.assertEqual(values, {(1, 'MPC_XY_VEL_MAX'): ('8.0', 9),
                                  (154, 'EKF2_GPS_DELAY'): ('150.0', 9)})
        self.assertEqual(log, [])

    def test_no_autopilot_component_listed(self):
        _, values, log = migrate((2, 'EKF2_GPS_DELAY', '150.0', 9))
        self.assertEqual(values, {(2, 'EKF2_GPS_DELAY'): ('150.0', 9)})
        self.assertEqual(log, ['warning: no parameters of component 1,'
                               ' nothing converted: only an autopilot with'
                               ' MAV_COMP_ID 1 is supported'])

    def test_core_heater_removed_with_hint(self):
        _, values, log = migrate((1, 'CORE_TEMP_ID', '3014698', 6))
        self.assertEqual(values, {})
        self.assertEqual(log, ['removed: CORE_TEMP_ID 3014698:'
                               ' see HEATER2_SENS_ID'])

    def test_nav_acceptance_radius_listed_for_multicopter(self):
        _, _, log = migrate((1, 'NAV_ACC_RAD', '2.000000000000000000', 9))
        self.assertEqual(log, ['default: NAV_ACC_RAD 2 is a v1.17 default,'
                               ' v1.18 uses 10 on multicopter, except'
                               ' airframes 4016, 4020 and 24001, which keep'
                               ' 2'])

    def test_multi_ekf_defaults_listed_for_skynode(self):
        _, values, log = migrate((1, 'EKF2_MULTI_IMU', '3', 6),
                                 (1, 'SENS_IMU_MODE', '0', 6))
        self.assertEqual(values, {(1, 'EKF2_MULTI_IMU'): ('3', 6),
                                  (1, 'SENS_IMU_MODE'): ('0', 6)})
        self.assertEqual(log, ['default: EKF2_MULTI_IMU 3 is a v1.17 default,'
                               ' v1.18 uses 0 on Skynode',
                               'default: SENS_IMU_MODE 0 is a v1.17 default,'
                               ' v1.18 uses 1 on Skynode'])

    def test_imxrt_defaults_listed(self):
        boards = 'on i.MX RT boards (fmu-v6xrt, mr-tropic, tropic-community)'
        _, values, log = migrate((1, 'EKF2_MULTI_IMU', '0', 6),
                                 (1, 'SENS_IMU_MODE', '1', 6),
                                 (1, 'IMU_GYRO_FFT_EN', '0', 6),
                                 (1, 'UAVCAN_ENABLE', '0', 6))
        self.assertEqual(values, {(1, 'EKF2_MULTI_IMU'): ('0', 6),
                                  (1, 'SENS_IMU_MODE'): ('1', 6),
                                  (1, 'IMU_GYRO_FFT_EN'): ('0', 6),
                                  (1, 'UAVCAN_ENABLE'): ('0', 6)})
        self.assertEqual(log, [
            f'default: EKF2_MULTI_IMU 0 is a v1.17 default, v1.18 uses 3'
            f' {boards}',
            f'default: SENS_IMU_MODE 1 is a v1.17 default, v1.18 uses 0'
            f' {boards}',
            f'default: IMU_GYRO_FFT_EN 0 is a v1.17 default, v1.18 uses 1'
            f' {boards}',
            f'default: UAVCAN_ENABLE 0 is a v1.17 default, v1.18 uses 2'
            f' {boards}'])

    def test_board_default_set_by_hand_not_listed(self):
        _, _, log = migrate((1, 'EKF2_MULTI_IMU', '2', 6),
                            (1, 'UAVCAN_ENABLE', '3', 6))
        self.assertEqual(log, [])

    def test_other_lines_copied_and_listed(self):
        log = []
        lines = m.migrate(HEADER + ['EKF2_GPS_DELAY,150\n'], log.append)
        self.assertEqual(lines, HEADER + ['EKF2_GPS_DELAY,150\n'])
        self.assertEqual(log, ['copied unchanged, not in the QGroundControl'
                               ' format: EKF2_GPS_DELAY,150'])

    def test_line_without_newline_kept_apart(self):
        lines = m.migrate(['# saved by hand'] + params_file(
            (1, 'MPC_XY_VEL_MAX', '8.0', 9))[len(HEADER):], lambda _: None)
        self.assertEqual(lines, ['# saved by hand\n',
                                 '1\t1\tMPC_XY_VEL_MAX\t8.0\t9\n'])

    def test_every_written_parameter_has_a_type(self):
        for kind, new, _ in m.CONVERSIONS.values():
            for name in (new,) if isinstance(new, str) else new:
                self.assertIn(name, m.NEW_TYPES)

    def test_firmware_translates_every_converted_parameter(self):
        # Catches a translation removed from the firmware, not a changed
        # target: main already writes SENS_GNSS*, the script the v1.18 names
        with open(TRANSLATION, encoding='utf-8') as f:
            source = f.read()
        for old in m.CONVERSIONS:
            self.assertIn(f'"{old}"', source)


if __name__ == '__main__':
    unittest.main()
