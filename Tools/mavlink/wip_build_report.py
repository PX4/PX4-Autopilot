#!/usr/bin/env python3
"""
Turn the compiler output of a beta or release stage build into a list of
unguarded MAVLink WIP uses, each with the guard to add.

GCC reports such a use as, for example:

    foo.cpp:87:3: error: 'void mavlink_msg_esc_eeprom_send_struct(...)' is
    deprecated: MAVLink WIP item used outside a guard. ...

The deprecation text cannot name the MAVLink definition, so this maps the
symbol back to it (mavlink_msg_esc_eeprom_* is ESC_EEPROM). It also lists the
legacy WIP items the build let through.

Usage:
    wip_build_report.py build.log [--summary $GITHUB_STEP_SUMMARY]

Exits 1 if any unguarded use was found.
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import mavlink_flags  # noqa: E402

DEPRECATED = re.compile(r'^(?P<path>[^\s:]+):(?P<line>\d+):(?:\d+:)? (?:error|warning): '
                        r'[\'‘`](?P<symbol>.+?)[\'’] is deprecated: MAVLink WIP item used outside a guard')
# GCC: "note: '#pragma message: Legacy ...'", clang: "warning: Legacy ... [-W#pragma-messages]"
LEGACY = re.compile(r'(?:note|warning): [\'‘]?(?:#pragma message: )?Legacy MAVLink WIP item (\w+) ships unguarded')
IDENTIFIER = re.compile(r'[A-Za-z_]\w*')


def item_for_symbol(symbol, definitions):
    """The MAVLink definition a symbol from a diagnostic belongs to."""
    # a function is shown with its signature: find the helper name in it
    helper = re.search(r'\bmavlink_msg_\w+', symbol)

    if helper:
        item = mavlink_flags.message_for_symbol(helper.group(0), definitions)

        if item is not None:
            return item.name, item.xml_file

    names = IDENTIFIER.findall(symbol)

    for name in reversed(names):
        item = definitions.get(name)

        if item is not None:
            return item.name, item.xml_file

    return (names[-1] if names else symbol), ''


def relative(path):
    path = os.path.normpath(path)

    if os.path.isabs(path) and path.startswith(mavlink_flags.PX4_SOURCE_DIR + os.sep):
        return os.path.relpath(path, mavlink_flags.PX4_SOURCE_DIR)

    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log', help='compiler output of the build')
    parser.add_argument('--summary', help='append a Markdown report to this file (e.g. $GITHUB_STEP_SUMMARY)')
    parser.add_argument('--title', default='', help='heading for the Markdown report, e.g. the target')
    parser.add_argument('--dialect', default=mavlink_flags.DEFAULT_DIALECT)
    parser.add_argument('--mavlink-dir', default=mavlink_flags.DEFAULT_MAVLINK_DIR)
    args = parser.parse_args()

    definitions = mavlink_flags.load(args.dialect, args.mavlink_dir)
    uses = {}
    legacy = set()

    with open(args.log, encoding='utf-8', errors='replace') as log:
        for line in log:
            match = DEPRECATED.search(line)

            if match:
                name, xml_file = item_for_symbol(match.group('symbol'), definitions)
                location = f'{relative(match.group("path"))}:{match.group("line")}'
                uses.setdefault((location, name), xml_file)

            for name in LEGACY.findall(line):
                legacy.add(name)

    for (location, name), xml_file in sorted(uses.items()):
        source = f' ({xml_file})' if xml_file else ''
        print(f'{location}: {name} is WIP{source} and used outside a guard. '
              f'Add: #if PX4_MAVLINK_WIP({name}) ... #endif')

    if uses:
        print(f'\n{len(uses)} unguarded MAVLink WIP use(s). Reproduce with "make mavlink_wip_check"; '
              'the rules are in the comment at the top of the generated mavlink_wip.h.', file=sys.stderr)

    if legacy:
        print('Legacy WIP items shipping unguarded (src/modules/mavlink/mavlink_wip_legacy.yaml): ' +
              ', '.join(sorted(legacy)))

    if args.summary:
        lines = [f'## MAVLink WIP release-stage build {args.title}'.rstrip() + '\n']

        if uses:
            lines.append(f'**{len(uses)} unguarded WIP use(s)**. Each must be guarded so it is excluded from '
                         'releases. Reproduce locally with `make mavlink_wip_check`.\n')

            for (location, name), xml_file in sorted(uses.items()):
                lines.append(f'- `{location}` {name} is WIP{f" ({xml_file})" if xml_file else ""}. '
                             f'Add: `#if PX4_MAVLINK_WIP({name})` ... `#endif`')
        else:
            lines.append('No unguarded WIP uses.')

        if legacy:
            lines.append('\nLegacy WIP items shipping unguarded: ' + ', '.join(sorted(legacy)))

        with open(args.summary, 'a', encoding='utf-8') as summary:
            summary.write('\n'.join(lines) + '\n\n')

    return 1 if uses else 0


if __name__ == '__main__':
    sys.exit(main())
