#! /usr/bin/env python
""" Script to generate Serial (UART) parameters and the ROMFS startup script """

from __future__ import print_function

import argparse
import os
import sys

from jinja2 import Environment, FileSystemLoader


## Configuration

# All possible Serial ports and their default param values.
# Omitted default values will be set to 0.
serial_ports = {
    "GPS1": {
        "label": "GPS 1",
        "defaults": {
            "CONFIG": 'Main GPS',
            "BAUD": 0,
            },
        },
    "GPS2": {
        "label": "GPS 2",
        "defaults": {
            "CONFIG": 'Disabled',
            "BAUD": 0,
            },
        },
    "TEL1": {
        "label": "TELEM 1",
        "defaults": {
            "CONFIG": 'MAVLink',
            "BAUD": 57600,
            "MAV_MDE": 'Normal',
            "MAV_R": 1200,
            "MAV_FWD": 1,
            },
        },
    "TEL2": { # companion port
        "label": "TELEM 2",
        "defaults": {
            "CONFIG": 'Disabled', # disabled by default to reduce RAM usage
            "BAUD": 57600,
            "MAV_MDE": 'OSD',
            },
        },
    "TEL3": {
        "label": "TELEM 3",
        "defaults": {
            "CONFIG": 'Disabled',
            "BAUD": 57600,
            "MAV_MDE": 'Normal',
            },
        },
    "TEL4": {
        "label": "TELEM 4",
        "defaults": {
            "CONFIG": 'MAVLink',
            "BAUD": 57600,
            "MAV_MDE": 'Normal',
            "MAV_R": 1200,
            },
        },
    "URT3": { # for Omnibus
        "label": "UART 3",
        "defaults": {
            "CONFIG": 'MAVLink',
            "BAUD": 57600,
            "MAV_MDE": 'Normal',
            "MAV_R": 1200,
            "MAV_FWD": 1,
            },
        },
    "URT6": { # for Omnibus
        "label": "UART 6",
        "defaults": {
            "CONFIG": 'Main GPS',
            "BAUD": 0,
            },
        },
    }

# Serial commands: each dict entry is a list with 2 items:
# - user-facing label (must be short, will be shown in a drop-down list)
# - ROMFS command (can be more than one or any scripting logic).
#   These variables can be used:
#       ${SER_DEV}             Serial device (e.g. /dev/ttyS1)
#       ${SER_TAG}             Serial tag (e.g. GPS1)
#       SER_${SER_TAG}_CONFIG  Param name for the current configuration
#       SER_${SER_TAG}_BAUD    Param name for the current baudrate
#       SER_${SER_TAG}_MAV_MDE Param name for the current mavlink mode
#       SER_${SER_TAG}_MAV_R   Param name for the current mavlink rate
#       SER_${SER_TAG}_MAV_FWD Param name for current mavlink forwarding
#       ${DUAL_GPS_ARGS}       Arguments passed to the gps to start the secondary GPS
#       ${MAV_ARGS}            Mavlink arguments (baudrate, mode and rate)
#
# Note: do NOT re-use or change indexes. When adding a command, always use an
# index that has never been used before. This is important for compatibility
# with QGC
serial_commands = {
    0: ["Disabled", ""],

    # MAVLink & RTPS
    1: ["MAVLink", "mavlink start -d ${SER_DEV} ${MAV_ARGS} -x"],

    2: ["MAVLink over Syslink", '''
syslink start
mavlink start -d /dev/bridge0 ${MAV_ARGS}
'''],

    3: ["MAVLink over Iridium", '''
iridiumsbd start -d ${SER_DEV}
mavlink start -d /dev/iridium -b 19200 -m iridium -r 10
'''],

    4: ["MAVLink + FastRTPS", '''
protocol_splitter start ${SER_DEV}
mavlink start -d /dev/mavlink ${MAV_ARGS} -x
micrortps_client start -d /dev/rtps -b p:SER_${SER_TAG}_BAUD -l -1
'''],
    5: ["FastRTPS", "micrortps_client start -d ${SER_DEV} -b p:SER_${SER_TAG}_BAUD -l -1"],

    # GPS
    50: ["Main GPS", "gps start -d ${SER_DEV} ${DUAL_GPS_ARGS}"],
    51: ["Secondary GPS", ""], # special handling for the command

    # Telemetry
    100: ["FrSky Telemetry", "frsky_telemetry start -d ${SER_DEV}"],
    101: ["HoTT Telemetry", "hott_telemetry start -d ${SER_DEV}"],

    # Sensor drivers
    200: ["LeddarOne Rangefinder", "leddar_one start -d ${SER_DEV}"],
    201: ["Benewake TFmini Rangefinder", "tfmini start -d ${SER_DEV}"],
    202: ["Lightware Laser Rangefinder", "sf0x start -d ${SER_DEV}"],
    }

mavlink_modes = {
    0: "Normal",
    1: "Custom",
    2: "Onboard",
    3: "OSD",
    4: "Magic",
    5: "Config",
    6: "Iridium",
    7: "Minimal",
    }

parser = argparse.ArgumentParser(description='Generate Serial params & startup script')

parser.add_argument('--serial-ports', type=str, nargs='*', metavar="TAG:DEVICE",
                    default=[],
                    help='Serial ports: mappings from the tag name to the device (e.g. GPS1:/dev/ttyS1)')
parser.add_argument('--all-ports', action='store_true',
                    help='Generate output for all known ports (params file only)')
parser.add_argument('--rc-file', type=str, action='store',
                    help='ROMFS output script', default=None)
parser.add_argument('--params-file', type=str, action='store',
                    help='Parameter output file', default=None)
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')

args = parser.parse_args()

arg_board_serial_ports = args.serial_ports
verbose = args.verbose
rc_serial_output_file = args.rc_file
rc_serial_template = 'rc.serial.jinja'
serial_params_output_file = args.params_file
serial_params_template = 'serial_params.c.jinja'
generate_for_all_ports = args.all_ports

if generate_for_all_ports:
    board_ports = [(key, "") for key in serial_ports]
else:
    # convert arg_board_serial_ports list [ "TAG:DEVICE" ] into [ ("TAG", "DEVICE") ]
    board_ports = [tuple(port.split(":")) for port in arg_board_serial_ports]


if rc_serial_output_file is None and serial_params_output_file is None:
    raise Exception("At least one of --rc-file (e.g. rc.serial) or --params-file "
        "(e.g. serial_params.c) needs to be specified")


# replace strings in defaults with their numerical value
for key in serial_ports:
    defaults = serial_ports[key]["defaults"]

    if "CONFIG" in defaults and isinstance(defaults["CONFIG"], str):
        config_index_list = [ key for key in serial_commands \
                if serial_commands[key][0] == defaults["CONFIG"]]
        if len(config_index_list) == 0:
            raise Exception("Config mode {:} not found".format(defaults["CONFIG"]))
        defaults["CONFIG"] = config_index_list[0]

    if "MAV_MDE" in defaults and isinstance(defaults["MAV_MDE"], str):
        mode_index_list = [ key for key in mavlink_modes \
                if mavlink_modes[key] == defaults["MAV_MDE"]]
        if len(mode_index_list) == 0:
            raise Exception("Mavlink mode {:} not found".format(defaults["MAV_MDE"]))
        defaults["MAV_MDE"] = mode_index_list[0]


# sanity check (makes sure the param names don't exceed the max length of 16 chars)
for key in serial_ports:
    if len(key) > 4:
        raise Exception("Serial tag {:} is too long (max length=4)".format(key))

serial_devices = []
for tag, device in board_ports:
    if tag not in serial_ports:
        raise Exception("Unknown serial port {:}. "
            "You might have to add it to serial_ports in\n {:}".format(tag,
                os.path.realpath(__file__)))
    serial_devices.append({
        'tag': tag,
        'device': device,
        'label': serial_ports[tag]["label"]
        })


# construct commands based on selected board
commands = []
disabled_commands = [] # e.g. ["HoTT Telemetry"] TODO: do we need support for that and configure it from the board config?
for key in serial_commands:
    if serial_commands[key][0] in disabled_commands:
        continue
    label = serial_commands[key][0]
    command = serial_commands[key][1]
    commands.append({
        'value': key,
        'command': command,
        'label': label
        })

if verbose:
    print("Serial Devices: {:}".format(serial_devices))
    #print("Commands: {:}".format(commands))


jinja_env = Environment(loader=FileSystemLoader(
    os.path.dirname(os.path.realpath(__file__))))

# generate the ROMFS script using a jinja template
if rc_serial_output_file is not None:
    if generate_for_all_ports:
        raise Exception("Cannot create rc file for --all-ports")

    if verbose: print("Generating {:}".format(rc_serial_output_file))
    if len(serial_devices) == 0:
        # if the board has no UARTs, create an empty rc file
        open(rc_serial_output_file, 'w').close()
    else:
        template = jinja_env.get_template(rc_serial_template)
        with open(rc_serial_output_file, 'w') as fid:
            secondary_gps_value = 51
            # sanity-check
            if serial_commands[secondary_gps_value][0] != "Secondary GPS":
                raise Exception("Unexpected value for Secondary GPS")
            fid.write(template.render(serial_devices=serial_devices,
                secondary_gps_value=secondary_gps_value,
                commands=commands))

# parameter definitions
if serial_params_output_file is not None:
    if verbose: print("Generating {:}".format(serial_params_output_file))
    template = jinja_env.get_template(serial_params_template)
    with open(serial_params_output_file, 'w') as fid:
        fid.write(template.render(serial_devices=serial_devices,
            commands=commands, mavlink_modes=mavlink_modes,
            serial_ports=serial_ports))

