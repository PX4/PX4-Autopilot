#!/usr/bin/env python3
"""Generate serial (UART) parameters, autostart tables, and the ROMFS startup script.

Port -> protocol: each board UART tag gets SER_<tag>_PROTO / SER_<tag>_BAUD.
Drivers declare a stable protocol_id in module.yaml serial_config.
rc.serial is a one-liner that runs serial_autostart; the C module walks ports.
"""

import argparse
import json
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
parser.add_argument('--board-with-io', action='store_true',
                    help='Board has PX4IO; do not default the RC UART to SBUS')
parser.add_argument('--metadata-file', type=str, action='store',
                    help='Serial component metadata JSON output file', default=None)
parser.add_argument('--compress', action='store_true',
                    help='Also write an xz-compressed copy of the metadata file')
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
board_with_io = args.board_with_io
metadata_output_file = args.metadata_file

if generate_for_all_ports:
    board_ports = [(key, "") for key in serial_ports]
else:
    board_ports = [tuple(port.split(":")) for port in arg_board_serial_ports]


if rc_serial_output_dir is None and serial_params_output_file is None and metadata_output_file is None:
    raise Exception("At least one of --rc-dir, --params-file "
        "(e.g. serial_params.c) or --metadata-file needs to be specified")


def parse_yaml_serial_config(yaml_config):
    if 'serial_config' not in yaml_config:
        return []
    ret = []
    module_name = yaml_config['module_name']
    # Parameters with an instance placeholder belong to whichever port runs
    # instance i; the GCS shows them under that port.
    instance_params = []
    for group in yaml_config.get('parameters', []):
        for name in group.get('definitions', {}):
            if '${i}' in name:
                instance_params.append(name)
    for serial_config in yaml_config['serial_config']:
        if 'protocol_id' not in serial_config:
            raise Exception("{:}: serial_config entry missing protocol_id".format(module_name))
        if 'protocol_name' not in serial_config:
            serial_config['protocol_name'] = module_name
        serial_config['instance_params'] = instance_params
        ret.append(serial_config)
    return ret

def compact_command(command, kind):
    """Reduce a yaml command to a single line the C walker can exec."""
    if kind == 'instance':
        # MAVLink flags are assembled in C from MAV_${i}_*.
        return 'mavlink start'
    if command is None:
        return ''
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
    collect_rank = {'GPS1': 0, 'GPS2': 1, 'GPS3': 2}.get(tag, 255)
    serial_devices.append({
        'tag': tag,
        'device': device,
        'label': serial_ports[tag]["label"],
        'index': serial_ports[tag]["index"],
        'default_baudrate': serial_ports[tag]["default_baudrate"],
        'collect_rank': collect_rank,
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
    identity = (serial_command['protocol_name'], compact_command(serial_command['command'], 'single'))
    if protocol_id in seen_ids:
        if seen_ids[protocol_id] != identity:
            raise Exception("duplicate protocol_id {:}: {:} and {:}".format(
                protocol_id, seen_ids[protocol_id][0], serial_command['protocol_name']))
        # Same module built from two trees (voxl2 ghst_rc vs src/drivers/rc/ghst_rc).
        continue
    seen_ids[protocol_id] = identity

    secondary_command = serial_command.get('secondary_command')
    num_instances = serial_command.get('num_instances', 2 if secondary_command else 1)
    if secondary_command:
        kind = 'collect'
    elif num_instances > 1:
        # The C walker assembles per-instance flags from MAV_${i}_*, so only
        # mavlink can run one process per port.
        if not compact_command(serial_command['command'], 'single').startswith('mavlink start'):
            raise Exception("{:}: num_instances > 1 requires secondary_command".format(
                serial_command['protocol_name']))
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
        # IO firmware owns SBUS/PPM/DSM on the RC connector. Keep the RC UART
        # configurable (CRSF/GHST) but do not start sbus_rc on it by default.
        skip_rc_default = board_with_io and default_tag == 'RC'
        if default_tag in dict(board_ports).keys() and not skip_rc_default:
            tag_defaults[default_tag] = protocol_id

    ethernet_param = serial_command.get('ethernet_param')
    ethernet_command = serial_command.get('ethernet_command')
    if ethernet_command:
        ethernet_command = compact_command(ethernet_command, 'single')

    protocols.append({
        'id': protocol_id,
        'name': serial_command['protocol_name'],
        'command': compact_command(serial_command['command'], kind),
        'secondary_command': compact_secondary(secondary_command),
        'success_command': compact_command(serial_command.get('success_command'), 'single') or None,
        'fail_command': compact_command(serial_command.get('fail_command'), 'single') or None,
        'kind': kind,
        'kind_id': {'single': 0, 'instance': 1, 'collect': 2}[kind],
        'num_instances': num_instances,
        'ethernet_param': ethernet_param,
        'ethernet_command': ethernet_command,
        'instance_params': serial_command['instance_params'] if kind == 'instance' else [],
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

if metadata_output_file is not None:
    # Static COMPONENT_METADATA describing the serial buses, the port-selects-
    # protocol counterpart of actuators.json. Ports are listed in instance
    # order: the n-th port running an "instance" protocol is instance n, and
    # ethernet takes the next free instance.
    ports = []
    for dev in serial_devices:
        ports.append({
            'id': dev['tag'],
            'label': dev['label'],
            'device': dev['device'],
            'protocolParam': 'SER_{:}_PROTO'.format(dev['tag']),
            'baudParam': 'SER_{:}_BAUD'.format(dev['tag']),
            'defaultBaud': dev['default_baudrate'],
            })
    protocol_list = [{'id': 0, 'name': 'Disabled'}]
    for protocol in protocols:
        entry = {
            'id': protocol['id'],
            'name': protocol['name'],
            'maxPorts': protocol['num_instances'],
            }
        if protocol['instance_params']:
            entry['instanceParams'] = protocol['instance_params']
        if protocol['ethernet_param']:
            entry['ethernetParam'] = protocol['ethernet_param']
        protocol_list.append(entry)
    metadata = {
        'version': 1,
        'buses': {
            'serial': {
                'ports': ports,
                'protocols': protocol_list,
                'ethernet': ethernet_supported,
                },
            },
        }
    if verbose:
        print("Generating {:}".format(metadata_output_file))
    with open(metadata_output_file, 'w') as fid:
        json.dump(metadata, fid, indent=2, sort_keys=True)
        fid.write('\n')
    if args.compress:
        import lzma
        with open(metadata_output_file, 'r') as fid:
            content = fid.read()
        with lzma.open(metadata_output_file + '.xz', 'wt', preset=9) as fid:
            fid.write(content)
