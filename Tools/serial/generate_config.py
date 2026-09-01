#!/usr/bin/env python3
"""Generate serial (UART) parameters, autostart tables, and the ROMFS startup script.

Port -> protocol: each board UART tag gets SER_<tag>_PROT / SER_<tag>_BAUD.
Drivers declare a stable protocol_id in module.yaml serial_config.
rc.serial is a one-liner that runs serial_autostart; the C module walks ports.
"""

import argparse
import os
import re
import sys

try:
    from jinja2 import Environment, FileSystemLoader
except ImportError as e:
    print("Failed to import jinja2: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user jinja2")
    print("")
    sys.exit(1)

try:
    import yaml
except ImportError as e:
    print("Failed to import yaml: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyyaml")
    print("")
    sys.exit(1)


# All possible Serial ports
# Note: do not re-use or change indexes. When adding a port, always use an
# index that has never been used before. This is important for compatibility
# with QGC (parameter metadata) and param_modify_on_import.
serial_ports = {
    # index 0 means disabled
    # index 1000 means ethernet

    "URT6": {
        "label": "UART 6",
        "index": 6,
        "default_baudrate": 57600,
        },

    "TEL1": {
        "label": "TELEM 1",
        "index": 101,
        "default_baudrate": 57600,
        },
    "TEL2": {
        "label": "TELEM 2",
        "index": 102,
        "default_baudrate": 921600,
        },
    "TEL3": {
        "label": "TELEM 3",
        "index": 103,
        "default_baudrate": 57600,
        },
    "TEL4": {
        "label": "TELEM/SERIAL 4",
        "index": 104,
        "default_baudrate": 57600,
        },

    "GPS1": {
        "label": "GPS 1",
        "index": 201,
        "default_baudrate": 0,
        },
    "GPS2": {
        "label": "GPS 2",
        "index": 202,
        "default_baudrate": 0,
        },
    "GPS3": {
        "label": "GPS 3",
        "index": 203,
        "default_baudrate": 0,
        },

    "RC": {
        "label": "Radio Controller",
        "index": 300,
        "default_baudrate": 0,
        },

    "WIFI": {
        "label": "Wifi Port",
        "index": 301,
        "default_baudrate": 1,
        },

    "EXT2": {
        "label": "EXT2",
        "index": 401,
        "default_baudrate": 57600,
        },
    }


parser = argparse.ArgumentParser(description='Generate Serial params & startup script')

parser.add_argument('--serial-ports', type=str, nargs='*', metavar="TAG:DEVICE",
                    default=[],
                    help='Serial ports: mappings from the tag name to the device (e.g. GPS1:/dev/ttyS1)')
parser.add_argument('--config-files', type=str, nargs='*', default=[],
                    help='YAML module config file(s)')
parser.add_argument('--all-ports', action='store_true',
                    help='Generate output for all known ports (params file only)')
parser.add_argument('--constrained-flash', action='store_true',
                    help='Reduce verbosity in ROMFS scripts to reduce flash size')
parser.add_argument('--rc-dir', type=str, action='store',
                    help='ROMFS output directory', default=None)
parser.add_argument('--params-file', type=str, action='store',
                    help='Parameter output file', default=None)
parser.add_argument('--ethernet', action='store_true',
                    help='Ethernet support')
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true',
                    help='Verbose Output')

args = parser.parse_args()

arg_board_serial_ports = args.serial_ports
verbose = args.verbose
rc_serial_output_dir = args.rc_dir
serial_params_output_file = args.params_file
generate_for_all_ports = args.all_ports
constrained_flash = args.constrained_flash
ethernet_supported = args.ethernet

if generate_for_all_ports:
    board_ports = [(key, "") for key in serial_ports]
else:
    board_ports = [tuple(port.split(":")) for port in arg_board_serial_ports]


if rc_serial_output_dir is None and serial_params_output_file is None:
    raise Exception("At least one of --rc-dir or --params-file "
        "(e.g. serial_params.c) needs to be specified")


def parse_yaml_serial_config(yaml_config):
    if 'serial_config' not in yaml_config:
        return []
    ret = []
    module_name = yaml_config['module_name']
    for serial_config in yaml_config['serial_config']:
        if 'protocol_id' not in serial_config:
            raise Exception("{:}: serial_config entry missing protocol_id".format(module_name))
        if 'protocol_name' not in serial_config:
            serial_config['protocol_name'] = module_name
        if 'label' not in serial_config:
            serial_config['label'] = serial_config['protocol_name']
        ret.append(serial_config)
    return ret

def compact_command(command, kind):
    """Reduce yaml nsh snippets to an argv template the C walker can exec."""
    if kind == 'instance':
        # MAVLink flags are assembled in C from MAV_${i}_*.
        return 'mavlink start'
    if command is None:
        return ''
    if 'iridiumsbd' in command:
        return 'usleep 200000; iridiumsbd start -d ${SERIAL_DEV}'
    if 'uxrce_dds_client' in command:
        return 'uxrce_dds_client start -t serial -d ${SERIAL_DEV} -b p:${BAUD_PARAM}'
    lines = []
    for line in command.splitlines():
        stripped = line.strip()
        if stripped and not stripped.startswith('#'):
            lines.append(stripped)
    return ' '.join(lines)


def compact_secondary(secondary_command):
    if not secondary_command:
        return None
    match = re.search(r'"([^"]*)"', secondary_command)
    if match:
        return match.group(1)
    return secondary_command.strip()


serial_commands = []
for yaml_file in args.config_files:
    with open(yaml_file, 'r') as stream:
        try:
            yaml_config = yaml.load(stream, Loader=yaml.Loader)
            if yaml_config is None:
                continue
            serial_commands.extend(parse_yaml_serial_config(yaml_config))

        except yaml.YAMLError as exc:
            print(exc)
            raise


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
        'label': serial_ports[tag]["label"],
        'index': serial_ports[tag]["index"],
        'default_baudrate': serial_ports[tag]["default_baudrate"]
        })

# Sort ports by index so MAVLink/GPS instance order is stable
serial_devices.sort(key=lambda d: d['index'])


protocols = []
seen_ids = {}
tag_defaults = {}  # tag -> protocol_id

for serial_command in serial_commands:
    protocol_id = int(serial_command['protocol_id'])
    if protocol_id <= 0:
        raise Exception("protocol_id must be > 0 (0 is Disabled)")
    if protocol_id in seen_ids:
        if seen_ids[protocol_id] != serial_command['protocol_name']:
            raise Exception("duplicate protocol_id {:}: {:} and {:}".format(
                protocol_id, seen_ids[protocol_id], serial_command['protocol_name']))
        # Same-name duplicate (voxl2 ghst_rc vs src/drivers/rc/ghst_rc).
        continue
    seen_ids[protocol_id] = serial_command['protocol_name']

    secondary_command = serial_command.get('secondary_command')
    num_instances = serial_command.get('num_instances', 2 if secondary_command else 1)
    if secondary_command:
        kind = 'collect'
    elif num_instances > 1:
        kind = 'instance'
    else:
        kind = 'single'

    default_tag = serial_command.get('default', '')
    if default_tag:
        if default_tag not in serial_ports:
            raise Exception("Default port {:} not found for {:}".format(
                default_tag, serial_command['protocol_name']))
        if default_tag in tag_defaults:
            raise Exception("Multiple protocols default to {:}".format(default_tag))
        # Only apply default if the board has that tag (or --all-ports)
        if default_tag in dict(board_ports).keys():
            tag_defaults[default_tag] = protocol_id

    ethernet_param = serial_command.get('ethernet_param')
    ethernet_command = None
    if ethernet_param and 'uxrce_dds_client' in serial_command['command']:
        ethernet_command = 'uxrce_dds_client start -t udp'

    protocols.append({
        'id': protocol_id,
        'name': serial_command['protocol_name'],
        'label': serial_command['label'],
        'command': compact_command(serial_command['command'], kind),
        'secondary_command': compact_secondary(secondary_command),
        'kind': kind,
        'kind_id': {'single': 0, 'instance': 1, 'collect': 2}[kind],
        'num_instances': num_instances,
        'supports_networking': serial_command.get('supports_networking', False),
        'ethernet_param': ethernet_param,
        'ethernet_command': ethernet_command,
        })

protocols.sort(key=lambda p: p['id'])

for dev in serial_devices:
    dev['default_protocol'] = tag_defaults.get(dev['tag'], 0)

ethernet_configuration = []
if ethernet_supported:
    ethernet_configuration.append({
        'tag': "ETH",
        'label': "Ethernet",
        'index': 1000
        })

if verbose:
    print("Serial Devices: {:}".format(serial_devices))
    print("Protocols: {:}".format([(p['id'], p['name'], p['kind']) for p in protocols]))


def c_escape(value):
    if value is None:
        return ''
    return value.replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n')


jinja_env = Environment(loader=FileSystemLoader(
    os.path.dirname(os.path.realpath(__file__))),
    keep_trailing_newline=True)
jinja_env.filters['c_escape'] = c_escape

if rc_serial_output_dir is not None:
    if generate_for_all_ports:
        raise Exception("Cannot create rc file for --all-ports")
    rc_serial_output_file = os.path.join(rc_serial_output_dir, "rc.serial")

    if verbose:
        print("Generating {:}".format(rc_serial_output_file))
    if len(serial_devices) == 0 and not ethernet_supported:
        open(rc_serial_output_file, 'w').close()
    else:
        template = jinja_env.get_template('rc.serial.jinja')
        with open(rc_serial_output_file, 'w') as fid:
            fid.write(template.render())

if serial_params_output_file is not None:
    if verbose:
        print("Generating {:}".format(serial_params_output_file))
    template = jinja_env.get_template('serial_params.c.jinja')
    with open(serial_params_output_file, 'w') as fid:
        fid.write(template.render(
            serial_devices=serial_devices,
            protocols=protocols,
            ethernet_configuration=ethernet_configuration))

    autostart_header = os.path.join(os.path.dirname(os.path.abspath(
        serial_params_output_file)), 'serial_autostart_config.h')
    if verbose:
        print("Generating {:}".format(autostart_header))
    template = jinja_env.get_template('serial_autostart_config.h.jinja')
    with open(autostart_header, 'w') as fid:
        fid.write(template.render(
            serial_devices=serial_devices,
            protocols=protocols,
            ethernet_configuration=ethernet_configuration))
