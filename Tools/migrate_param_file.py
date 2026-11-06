#!/usr/bin/env python3
############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

"""Convert a parameter file saved on PX4 v1.17 for loading on v1.18.

param_translation.cpp converts renamed parameters only when PX4 upgrades
the parameters stored on the vehicle. Loading a .params file in
QGroundControl sets each parameter by name, so values saved under old
names are ignored. This script applies the same conversions to a
QGroundControl .params file, writes RC*_REV as integers (QGroundControl
loads a saved -1.000000 into the now integer parameter as 0), and drops
the parameters v1.18 no longer has, listing each one on stderr.

A saved file also holds the v1.17 defaults. Values still at a v1.17
default that v1.18 changed are kept and listed, so they can be reviewed.
The RoboClaw output defaults, whose scale changed, get the v1.18 default,
as after an upgrade of the parameters stored on the vehicle.

Check the converted file before loading it.

Examples:
    Tools/migrate_param_file.py saved.params converted.params
"""

import argparse
import struct
import sys

AUTOPILOT_COMPONENT = 1
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9

# Conversions of param_translation.cpp from v1.17 to v1.18.
# old name: (kind, new name(s), argument)
CONVERSIONS = {
    'EKF2_GPS_POS_X': ('rename', 'SENS_GPS0_OFFX', None),
    'EKF2_GPS_POS_Y': ('rename', 'SENS_GPS0_OFFY', None),
    'EKF2_GPS_POS_Z': ('rename', 'SENS_GPS0_OFFZ', None),
    'EKF2_GPS_DELAY': ('copy_int',
                       ('SENS_GPS0_DELAY', 'SENS_GPS1_DELAY'), None),
    'MNT_RANGE_PITCH': ('half_range',
                        ('MNT_MAX_PITCH', 'MNT_MIN_PITCH'), None),
    'MOT_POLE_COUNT': ('copy_int',
                       tuple(f'DSHOT_MOT_POL{i}' for i in range(1, 13)),
                       None),
    # only "disabled" is converted
    'COM_RC_OVERRIDE': ('if_zero_set', 'MAN_OVERRIDE_SPD', -1.0),
    'SENS_IMU_TEMP': ('rename', 'HEATER1_TEMP', None),
    'SENS_IMU_TEMP_FF': ('rename', 'HEATER1_TEMP_FF', None),
    'SENS_IMU_TEMP_I': ('rename', 'HEATER1_TEMP_I', None),
    'SENS_IMU_TEMP_P': ('rename', 'HEATER1_TEMP_P', None),
    # SENS_TEMP_ID 0 is the v1.17 default, which the vehicle does not store,
    # so the upgrade leaves HEATER1_SENS_ID at its v1.18 default (the file
    # cannot show a 0 stored over a board default, which becomes -1)
    'SENS_TEMP_ID': ('if_nonzero_rename_int', 'HEATER1_SENS_ID', None),
    # landing nudging is bit 1 of MPC_AUTO_NUDGING (v1.18 default 0)
    'MPC_LAND_RC_HELP': ('if_nonzero_set_bit', 'MPC_AUTO_NUDGING', 1),
    # the first source slot only fuses messages with id EKF2_AGP0_ID, which
    # is set to 1 (the id v1.17 sources arrive with) when fusion was enabled
    'EKF2_AGP_CTRL': ('rename_if_nonzero_set',
                      ('EKF2_AGP0_CTRL', 'EKF2_AGP0_ID'), 1),
    'EKF2_AGP_MODE': ('rename', 'EKF2_AGP0_MODE', None),
    'EKF2_AGP_DELAY': ('rename', 'EKF2_AGP0_DELAY', None),
    'EKF2_AGP_NOISE': ('rename', 'EKF2_AGP0_NOISE', None),
    'EKF2_AGP_GATE': ('rename', 'EKF2_AGP0_GATE', None),
}

# Types of the parameters the conversions write
NEW_TYPES = {
    'SENS_GPS0_OFFX': MAV_PARAM_TYPE_REAL32,
    'SENS_GPS0_OFFY': MAV_PARAM_TYPE_REAL32,
    'SENS_GPS0_OFFZ': MAV_PARAM_TYPE_REAL32,
    'SENS_GPS0_DELAY': MAV_PARAM_TYPE_INT32,
    'SENS_GPS1_DELAY': MAV_PARAM_TYPE_INT32,
    'MNT_MAX_PITCH': MAV_PARAM_TYPE_REAL32,
    'MNT_MIN_PITCH': MAV_PARAM_TYPE_REAL32,
    **{f'DSHOT_MOT_POL{i}': MAV_PARAM_TYPE_INT32 for i in range(1, 13)},
    'MAN_OVERRIDE_SPD': MAV_PARAM_TYPE_REAL32,
    'HEATER1_TEMP': MAV_PARAM_TYPE_REAL32,
    'HEATER1_TEMP_FF': MAV_PARAM_TYPE_REAL32,
    'HEATER1_TEMP_I': MAV_PARAM_TYPE_REAL32,
    'HEATER1_TEMP_P': MAV_PARAM_TYPE_REAL32,
    'HEATER1_SENS_ID': MAV_PARAM_TYPE_INT32,
    'MPC_AUTO_NUDGING': MAV_PARAM_TYPE_INT32,
    'EKF2_AGP0_CTRL': MAV_PARAM_TYPE_INT32,
    'EKF2_AGP0_ID': MAV_PARAM_TYPE_INT32,
    'EKF2_AGP0_MODE': MAV_PARAM_TYPE_INT32,
    'EKF2_AGP0_DELAY': MAV_PARAM_TYPE_REAL32,
    'EKF2_AGP0_NOISE': MAV_PARAM_TYPE_REAL32,
    'EKF2_AGP0_GATE': MAV_PARAM_TYPE_REAL32,
}

# RC*_REV changed from float to int32
REVERSE_PARAMS = {f'RC{i}_REV' for i in range(1, 19)}

_SEE_LIGHTS = 'see UAVCAN_LGT_MODE, UAVCAN_LGT_NUM, UAVCAN_LGT_IDn/FNn'

# Parameters removed in v1.18 without a conversion: name: hint
REMOVED = {
    'ASPD_BETA_GATE': '', 'ASPD_BETA_NOISE': '', 'ASPD_SCALE_NSD': '',
    'ASPD_TAS_GATE': '',
    'BAT1_I_CHANNEL': '', 'BAT2_I_CHANNEL': '', 'BAT_ADC_CHANNEL': '',
    'COM_ARM_SDCARD': '', 'COM_DISARM_MAN': '', 'COM_FLT_PROFILE': '',
    'COM_HLDL_REG_T': '', 'COM_IMB_PROP_ACT': 'see FD_IMB_PROP_THR',
    'COM_KILL_DISARM': '', 'COM_LKDOWN_TKO': '', 'COM_MOT_TEST_EN': '',
    'COM_OBC_LOSS_T': '', 'COM_RC_ARM_HYST': '',
    'COM_RC_STICK_OV': 'see MAN_OVERRIDE_SPD (stick speed in 1/s)',
    'COM_TAKEOFF_ACT': '',
    # CUAV X25-EVO core heater, now the second heater instance
    'CORE_IMU_TEMP': 'see HEATER2_TEMP',
    'CORE_IMU_TEMP_FF': 'see HEATER2_TEMP_FF',
    'CORE_IMU_TEMP_I': 'see HEATER2_TEMP_I',
    'CORE_IMU_TEMP_P': 'see HEATER2_TEMP_P',
    'CORE_TEMP_ID': 'see HEATER2_SENS_ID',
    'DSHOT_BIDIR_EN': 'choose BDShot in PWM_MAIN_TIMn / PWM_AUX_TIMn',
    'FD_ACT_MOT_C2T': 'see MOTFAIL_C2T and MOTFAIL_OFF',
    'FD_ACT_MOT_THR': '',
    'FD_ACT_MOT_TOUT': 'see MOTFAIL_TIME (in s)',
    'FD_ESCS_EN': 'see FD_ACT_EN',
    'FW_AT_START': '', 'FW_T_SPD_DEV_STD': '', 'FW_T_SPD_PRC_STD': '',
    'FW_T_SPD_STD': '', 'GF_PREDICT': '', 'INA226_CONFIG': '',
    'LNDMC_TRIG_TIME': '', 'MC_AT_START': '',
    'MNT_OFF_PITCH': 'use the output trim',
    'MNT_OFF_ROLL': 'use the output trim',
    'MNT_OFF_YAW': 'use the output trim',
    'MPC_USE_HTE': '', 'SIM_ARSPD_FAIL': '',
    'SIM_BAT_ENABLE': 'see SIM_BAT_DRAIN',
    'UAVCAN_LGT_ANTCL': _SEE_LIGHTS, 'UAVCAN_LGT_LAND': _SEE_LIGHTS,
    'UAVCAN_LGT_NAV': _SEE_LIGHTS, 'UAVCAN_LGT_STROB': _SEE_LIGHTS,
}


# Defaults changed in v1.18: name: (v1.17 default, v1.18 default)
# A converted parameter is listed under its v1.17 name.
DEFAULT_CHANGES = {
    'EKF2_AGP_NOISE': (0.9, 1.0),
    'FD_ACT_EN': (1, 0),
    'FW_T_HRATE_FF': (0.3, 0.5),
    'MPC_LAND_RADIUS': (1000.0, -1.0),
    'SENS_MAG_AUTOCAL': (1, 0),
    'VT_LND_PITCH_MIN': (-5.0, 0.0),
    'VT_PITCH_MIN': (-5.0, 0.0),
    # set by the v1.18 fixed-wing defaults, still 2.5 for other vehicles
    'EKF2_REQ_PDOP': (2.5, 4.0),
    # set by the v1.17 multicopter defaults, no longer in v1.18 (boats and
    # airframes 4016, 4020 and 24001 keep 2)
    'NAV_ACC_RAD': (2.0, 10.0),
}

# Defaults that v1.18 changed for some vehicles only
DEFAULT_CHANGE_SCOPES = {
    'EKF2_REQ_PDOP': 'on fixed-wing',
    'NAV_ACC_RAD': ('on multicopter, except airframes 4016, 4020 and 24001,'
                    ' which keep 2'),
}

_SKYNODE = 'on Skynode'
_IMXRT = 'on i.MX RT boards (fmu-v6xrt, mr-tropic, tropic-community)'

# Defaults that the v1.18 board or chip defaults changed on some boards:
# name: [(v1.17 default, v1.18 default, scope)]
BOARD_DEFAULT_CHANGES = {
    # Skynode (fmu-v5x) runs a single EKF in v1.18 instead of the three of
    # the STM32F7 defaults. The i.MX RT defaults now match fmu-v6x.
    'EKF2_MULTI_IMU': [(3, 0, _SKYNODE), (0, 3, _IMXRT)],
    'SENS_IMU_MODE': [(0, 1, _SKYNODE), (1, 0, _IMXRT)],
    'IMU_GYRO_FFT_EN': [(0, 1, _IMXRT)],
    'UAVCAN_ENABLE': [(0, 2, _IMXRT)],
}

# RoboClaw outputs: 0 to 256 with stop at 128 in v1.17, 0 to 254 with stop
# at 127 in v1.18. In v1.18 the v1.17 defaults are out of range or one
# step off (DIS 128 drives forward while disarmed), so they are replaced:
# name: (v1.17 default, v1.18 default)
REPLACED_DEFAULTS = {
    'RBCLW_DIS1': (128, 127), 'RBCLW_DIS2': (128, 127),
    'RBCLW_MAX1': (256, 254), 'RBCLW_MAX2': (256, 254),
    'RBCLW_MIN1': (1, 0), 'RBCLW_MIN2': (1, 0),
}

_RBCLW_SCALE = ('RoboClaw outputs use 0 to 254 with stop at 127 in v1.18'
                ' (v1.17: 0 to 256, stop at 128)')

# Parameters whose other values need a check: name: (value to skip, note)
SCALE_CHECKS = {
    **{name: (None, _RBCLW_SCALE) for name in REPLACED_DEFAULTS},
    'RBCLW_FAIL1': (-1, _RBCLW_SCALE), 'RBCLW_FAIL2': (-1, _RBCLW_SCALE),
}


def float32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def show(value):
    """Format a value for the log, integers without an exponent."""
    if float(value).is_integer():
        return str(int(value))
    return f'{value:g}'


def format_value(value, mav_type):
    """Format a value the way QGroundControl saves it."""
    if mav_type == MAV_PARAM_TYPE_REAL32:
        return f'{float32(value):.18f}'
    return str(int(value))


def convert(name, value, current):
    """Return [(name, value)] for an old parameter, as PX4 converts it.

    current: values already in the file, for MPC_AUTO_NUDGING.
    """
    kind, new, arg = CONVERSIONS[name]
    if kind == 'rename':
        return [(new, value)]
    if kind == 'copy_int':
        return [(n, int(value)) for n in new]
    if kind == 'half_range':
        if value <= 0:
            return []
        half = float32(value) * 0.5
        return [(new[0], half), (new[1], -half)]
    if kind == 'if_zero_set':
        return [(new, arg)] if value == 0 else []
    if kind == 'if_nonzero_rename_int':
        return [(new, int(value))] if value != 0 else []
    if kind == 'rename_if_nonzero_set':
        return [(new[0], value)] + ([(new[1], arg)] if value != 0 else [])
    if kind == 'if_nonzero_set_bit':
        if value == 0:
            return []
        return [(new, int(current.get(new, 0)) | (1 << arg))]
    raise ValueError(kind)


def log_old_default(name, value, log):
    """List a value still at a v1.17 default that v1.18 changed."""
    if name in DEFAULT_CHANGES:
        old, new = DEFAULT_CHANGES[name]
        if value == float32(old):
            scope = DEFAULT_CHANGE_SCOPES.get(name)
            log(f'default: {name} {show(value)} is a v1.17 default,'
                f' v1.18 uses {show(new)}'
                + (f' {scope}' if scope else ''))
    for old, new, scope in BOARD_DEFAULT_CHANGES.get(name, []):
        if value == float32(old):
            log(f'default: {name} {show(value)} is a v1.17 default,'
                f' v1.18 uses {show(new)} {scope}')


def migrate(lines, log):
    """Convert the lines of a .params file and return the new lines."""
    rows = []
    for line in lines:
        fields = line.rstrip('\r\n').split('\t')
        try:
            int(fields[1])
            float(fields[3])
            assert len(fields) == 5
        except (AssertionError, IndexError, ValueError):
            if line.strip() and not line.startswith('#'):
                log(f'copied unchanged, not in the QGroundControl format:'
                    f' {line.strip()}')
            # parameters follow these lines in the output
            rows.append(line if line.endswith('\n') else line + '\n')
            continue
        rows.append(fields)

    params = [r for r in rows if isinstance(r, list)
              and r[1] == str(AUTOPILOT_COMPONENT)]
    if not params and any(isinstance(r, list) for r in rows):
        log(f'warning: no parameters of component {AUTOPILOT_COMPONENT},'
            f' nothing converted: only an autopilot with MAV_COMP_ID'
            f' {AUTOPILOT_COMPONENT} is supported')
    current = {r[2]: float(r[3]) for r in params}
    vehicle = params[0][0] if params else '1'
    new_values = {}

    out = []
    for row in rows:
        if not isinstance(row, list) or row[1] != str(AUTOPILOT_COMPONENT):
            out.append(row)
            continue
        name = row[2]
        value = float32(float(row[3]))
        if name in CONVERSIONS:
            writes = convert(name, value, {**current, **new_values})
            for new_name, new_value in writes:
                new_values[new_name] = new_value
            done = ', '.join(f'{n} {show(v)}' for n, v in writes)
            log(f'converted: {name} {show(value)} ->'
                f' {done or "nothing to set"}')
            log_old_default(name, value, log)
        elif name in REMOVED:
            hint = f': {REMOVED[name]}' if REMOVED[name] else ''
            log(f'removed: {name} {show(value)}{hint}')
        elif (name in REPLACED_DEFAULTS
              and value == REPLACED_DEFAULTS[name][0]):
            old, new = REPLACED_DEFAULTS[name]
            log(f'replaced: {name} {show(old)} -> {show(new)},'
                f' the v1.17 default by the v1.18 default')
            out.append([row[0], row[1], name,
                        format_value(new, int(row[4])), row[4]])
        elif name in REVERSE_PARAMS and row[4] != str(MAV_PARAM_TYPE_INT32):
            reverse = -1 if value < 0 else 1
            if reverse < 0:
                log(f'reversed: {name} written as the integer -1')
            out.append([row[0], row[1], name, str(reverse),
                        str(MAV_PARAM_TYPE_INT32)])
        else:
            log_old_default(name, value, log)
            if name in SCALE_CHECKS and value != SCALE_CHECKS[name][0]:
                log(f'check: {name} {show(value)}: {SCALE_CHECKS[name][1]}')
            out.append(row)

    # A converted value replaces the file's own value of that parameter
    out = [r for r in out if not (isinstance(r, list)
                                  and r[1] == str(AUTOPILOT_COMPONENT)
                                  and r[2] in new_values)]
    for name, value in new_values.items():
        out.append([vehicle, str(AUTOPILOT_COMPONENT), name,
                    format_value(value, NEW_TYPES[name]),
                    str(NEW_TYPES[name])])

    header = [r for r in out if not isinstance(r, list)]
    body = sorted((r for r in out if isinstance(r, list)),
                  key=lambda r: (int(r[1]), r[2]))
    return header + ['\t'.join(r) + '\n' for r in body]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('input', help='.params file saved on PX4 v1.17')
    parser.add_argument('output', help='converted .params file to write')
    args = parser.parse_args()

    with open(args.input, encoding='utf-8') as f:
        lines = f.readlines()

    version = next((line.split(':', 1)[1].strip() for line in lines
                    if line.startswith('# Version:')), None)
    if version is not None and not version.startswith('1.17'):
        print(f'warning: the file was saved on PX4 {version}, not v1.17',
              file=sys.stderr)

    converted = migrate(lines, lambda message: print(message, file=sys.stderr))

    with open(args.output, 'w', encoding='utf-8') as f:
        f.writelines(converted)


if __name__ == '__main__':
    main()
