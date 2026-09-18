#!/usr/bin/env python3
"""
Decode the ESC EEPROM dumps contained in a PX4 ULog file.

Two ways to get the schema:
  - no --schema: downloads the current one from https://am32.ca/eeprom
  - --schema <file>: uses that local JSON file instead

Install: pip install pyulog
Run:     ./esc_eeprom_decode.py <log.ulg>
         ./esc_eeprom_decode.py <log.ulg> --schema my_eeprom.json
"""

import argparse
import json
import sys
import urllib.request

INFO_KEY = 'esc_eeprom_read'
SCHEMA_URL = 'https://am32.ca/eeprom'

# MAVLink ESC_FIRMWARE enum, as logged in the 'firmware' field
ESC_FIRMWARE = {0: 'UNKNOWN', 1: 'AM32'}


def load_dumps(log_path):
    """Return every EEPROM dump found in the log, as {'esc', 'firmware', 'data'} dicts."""
    from pyulog import ULog

    ulog = ULog(log_path, None, False)
    chains = ulog.msg_info_multiple_dict.get(INFO_KEY, [])

    dumps = []

    for chain in chains:
        for line in chain:
            # Each line is: "esc=1 firmware=1 data=1234"
            fields = {}

            for token in line.split():
                key, value = token.split('=', 1)
                fields[key] = value

            dumps.append({
                'esc': int(fields['esc']),
                'firmware': int(fields['firmware']),
                'data': bytes.fromhex(fields['data']),
            })

    return dumps


def load_schema(schema_path):
    """Load the schema from a local file, or download the current one if no file is given."""
    if schema_path:
        with open(schema_path, encoding='utf-8') as schema_file:
            text = schema_file.read()

    else:
        print(f'no --schema given, downloading the current one from {SCHEMA_URL}', file=sys.stderr)

        with urllib.request.urlopen(SCHEMA_URL, timeout=10) as response:
            text = response.read().decode('utf-8')

    return json.loads(text)


def firmware_key(text):
    """Turn '2.18' into the tuple (2, 18), so firmware versions can be compared with <, >=, etc."""
    major, _, minor = text.partition('.')
    return (int(major), int(minor or 0))


def apply_overlay(resolved, overlay):
    """Apply a version overlay onto a field definition: objects merge one level deep, everything
    else replaces the previous value."""
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(resolved.get(key), dict):
            resolved[key].update(value)

        else:
            resolved[key] = value


def resolve_field(field, eeprom_version, firmware_version):
    """Collapse a field definition and its version overlays into a single definition.

    Overlays are applied in the order the schema defines: 'default' first, then 'eeprom:N+' in
    ascending N, then 'firmware:X.Y+' in ascending X.Y, keeping only those the reported versions
    satisfy.
    """
    resolved = dict(field)
    versions = resolved.pop('versions', {})

    if 'default' in versions:
        apply_overlay(resolved, versions['default'])

    eeprom_overlays = []
    firmware_overlays = []

    for key, overlay in versions.items():
        if key.startswith('eeprom:') and key.endswith('+'):
            eeprom_overlays.append((int(key[len('eeprom:'):-1]), overlay))

        elif key.startswith('firmware:') and key.endswith('+'):
            firmware_overlays.append((firmware_key(key[len('firmware:'):-1]), overlay))

    # The 'N+' / 'X.Y+' thresholds in one field are always unique, so sorting the (threshold,
    # overlay) pairs directly sorts by threshold.
    eeprom_overlays.sort()
    firmware_overlays.sort()

    for minimum, overlay in eeprom_overlays:
        if eeprom_version >= minimum:
            apply_overlay(resolved, overlay)

    for minimum, overlay in firmware_overlays:
        if firmware_version >= minimum:
            apply_overlay(resolved, overlay)

    return resolved


def read_raw(data, offset, size):
    """Read a little-endian unsigned integer, or None if it falls outside the dump."""
    if offset + size > len(data):
        return None

    return int.from_bytes(data[offset:offset + size], 'little')


def format_value(field, raw):
    """Render a raw value using the field's enum table or display scaling."""
    field_type = field.get('type')
    unit = field.get('unit', '')

    if field_type == 'enum':
        for entry in field.get('values', []):
            if entry.get('raw') == raw:
                return f'{entry.get("name", raw)} ({raw})'

        return f'{raw} (not in schema)'

    if field_type == 'bool':
        return f'{"On" if raw else "Off"} ({raw})'

    display = field.get('display')

    if display:
        scaled = raw * display.get('factor', 1) + display.get('offset', 0)
        text = f'{scaled:.{display.get("decimals", 0)}f}'
        return f'{text} {unit}'.strip() + f'  (raw {raw})'

    return f'{raw} {unit}'.strip()


def eeprom_version_of(data, fields):
    """Read the byte the schema calls 'eepromVersion', or 0 if the schema has no such field."""
    field = fields.get('eepromVersion')

    if not field:
        return 0

    return read_raw(data, field['offset'], field.get('size', 1)) or 0


def firmware_version_of(data, fields):
    """Read the (major, minor) firmware version the schema reports, or (0, 0) if it can't."""
    major_field = fields.get('firmwareMajor')
    minor_field = fields.get('firmwareMinor')

    if not major_field or not minor_field:
        return (0, 0)

    major = read_raw(data, major_field['offset'], major_field.get('size', 1)) or 0
    minor = read_raw(data, minor_field['offset'], minor_field.get('size', 1)) or 0
    return (major, minor)


def decode(dump, schema):
    """Decode one raw dump into {name: (label, text)} for every setting present in it."""
    data = dump['data']
    fields = schema['fields']

    eeprom_version = eeprom_version_of(data, fields)
    firmware_version = firmware_version_of(data, fields)

    settings = {}

    for name, field in fields.items():
        if field.get('type') == 'reserved':
            continue

        if eeprom_version < field.get('minEepromVersion', 0):
            continue

        resolved = resolve_field(field, eeprom_version, firmware_version)
        raw = read_raw(data, resolved['offset'], resolved.get('size', 1))

        if raw is not None:
            settings[name] = (resolved.get('name', name), format_value(resolved, raw))

    return eeprom_version, firmware_version, settings


def print_dump(dump, schema, index, total):
    eeprom_version, firmware_version, settings = decode(dump, schema)
    layout = schema.get('eepromVersions', {}).get(str(eeprom_version), {})

    title = f' ESC {dump["esc"]} '

    if total > 1:
        title += f'({index}/{total}) '

    print()
    print(title.center(64, '='))
    print(f'  firmware type   {ESC_FIRMWARE.get(dump["firmware"], dump["firmware"])}')
    print(f'  firmware        {firmware_version[0]}.{firmware_version[1]}')
    print(f'  EEPROM layout   {eeprom_version}  ({layout.get("description", "unknown")})')

    groups = schema.get('groups', {})
    order = list(schema.get('groupOrder', []))

    for group_key in groups:
        if group_key not in order:
            order.append(group_key)

    shown = set()

    for group_key in order:
        names = [name for name in groups.get(group_key, {}).get('fields', []) if name in settings]

        if not names:
            continue

        print(f'\n  {groups[group_key].get("name", group_key)}')

        for name in names:
            label, text = settings[name]
            print(f'    {label:<28} {text}')
            shown.add(name)

    leftover = [name for name in settings if name not in shown]

    if leftover:
        print('\n  Other')

        for name in leftover:
            label, text = settings[name]
            print(f'    {label:<28} {text}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log', help='ULog file (.ulg) containing esc_eeprom_read entries')
    parser.add_argument('-s', '--schema', help='local schema JSON file (default: download from am32.ca)')
    parser.add_argument('-e', '--esc', type=int, help='only show this ESC (1-based, as printed by PX4)')
    args = parser.parse_args()

    dumps = load_dumps(args.log)

    if args.esc is not None:
        dumps = [dump for dump in dumps if dump['esc'] == args.esc]

    if not dumps:
        sys.exit(f'no esc_eeprom_read entries in {args.log}')

    schema = load_schema(args.schema)

    print(f'log:    {args.log}')
    print(f'schema: {schema.get("title", "untitled")} v{schema.get("version", "?")}')

    for index, dump in enumerate(dumps, start=1):
        print_dump(dump, schema, index, len(dumps))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        sys.exit(f'error: {error}')
