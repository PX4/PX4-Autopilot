#!/usr/bin/env python3
"""
Check that PX4 source only uses MAVLink definitions marked <wip> behind a
PX4_MAVLINK_WIP(<NAME>) guard, so that they cannot reach a release.

Beta and release builds already fail to compile on an unguarded use of a WIP
message, enum or enum entry. This lint covers what the compiler cannot see:

  - VEHICLE_CMD_* (msg/versioned/VehicleCommand.msg) and NAV_CMD_*
    (src/modules/navigator/navigation.h) constants whose number is a WIP
    MAV_CMD: PX4 handles commands by number, not by MAV_CMD name
  - stream names of WIP messages in the mavlink module (stream tables)
  - code in modules that none of the checked targets builds
  - guards that name an unknown definition, or one that is no longer WIP
    (released: remove the guard)
  - the old dialect-presence guard, #if defined(MAVLINK_MSG_ID_<WIP message>)
  - the legacy list src/modules/mavlink/mavlink_wip_legacy.yaml: entries must
    be WIP and still used, waivers must name an entry, and with --base the
    list may not grow

A use counts as guarded inside #if PX4_MAVLINK_WIP(...), or in a header that
is only included from guarded code. For an item on the legacy list, a message
or enum use may instead sit inside PX4_MAVLINK_WIP_LEGACY_BEGIN(<NAME>) /
PX4_MAVLINK_WIP_LEGACY_END, a command handled by number must sit inside
#if PX4_MAVLINK_WIP_LEGACY(<NAME>) (so that switching the list off removes
it), and a stream name needs nothing more.

Usage:
    check_wip.py                       report problems, exit 1 if there are any
    check_wip.py --base origin/main    also fail if the legacy list grew
    check_wip.py --list-guarded        list the guarded WIP uses (no checks)
    check_wip.py --summary FILE        also append a Markdown report to FILE
"""

import argparse
import bisect
import os
import re
import subprocess
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import mavlink_flags  # noqa: E402

PX4_SOURCE_DIR = mavlink_flags.PX4_SOURCE_DIR
LEGACY_FILE = 'src/modules/mavlink/mavlink_wip_legacy.yaml'
VEHICLE_COMMAND_MSG = 'msg/versioned/VehicleCommand.msg'
NAVIGATION_H = 'src/modules/navigator/navigation.h'
MAVLINK_MODULE = 'src/modules/mavlink/'
SOURCE_ROOTS = ('src', 'platforms', 'boards')
SOURCE_EXTENSIONS = ('.c', '.cpp', '.h', '.hpp')

GUARD = re.compile(r'(!\s*)?PX4_MAVLINK_WIP\s*\(\s*(\w*)\s*\)')
LEGACY_GUARD = re.compile(r'(!\s*)?\bPX4_MAVLINK_WIP_LEGACY\s*\(\s*(\w*)\s*\)')
LEGACY_BEGIN = re.compile(r'\bPX4_MAVLINK_WIP_LEGACY_BEGIN\s*\(\s*(\w*)\s*\)')
LEGACY_END = re.compile(r'\bPX4_MAVLINK_WIP_LEGACY_END\b')
DIRECTIVE = re.compile(r'^\s*#\s*(\w+)\s*(.*)$')
INCLUDE = re.compile(r'"([^"]+)"')
MSG_ID = re.compile(r'\bMAVLINK_MSG_ID_(\w+)')
IDENTIFIER = re.compile(r'(::|\.|->)?\s*\b([A-Za-z_]\w*)')
COMMAND_CONSTANT = re.compile(r'\b((?:VEHICLE|NAV)_CMD_\w+)\s*=\s*(\d+)')


class Finding:
    def __init__(self, path, line, message, hint=''):
        self.path = path
        self.line = line
        self.message = message
        self.hint = hint

    def __str__(self):
        text = f'{self.path}:{self.line}: error: {self.message}'
        return text + (f'\n    {self.hint}' if self.hint else '')


def guard_hint(name):
    return f'Add: #if PX4_MAVLINK_WIP({name}) ... #endif'


# ----------------------------------------------------------------------------
# Source files


def submodule_paths():
    paths = []
    try:
        with open(os.path.join(PX4_SOURCE_DIR, '.gitmodules'), encoding='utf-8') as gitmodules:
            for line in gitmodules:
                match = re.match(r'\s*path\s*=\s*(\S+)', line)

                if match:
                    paths.append(match.group(1).rstrip('/') + '/')
    except FileNotFoundError:
        pass

    return paths


def source_files():
    skip = submodule_paths()

    for root in SOURCE_ROOTS:
        for directory, subdirectories, files in os.walk(os.path.join(PX4_SOURCE_DIR, root)):
            relative = os.path.relpath(directory, PX4_SOURCE_DIR).replace(os.sep, '/') + '/'

            if any(relative.startswith(path) for path in skip):
                subdirectories[:] = []
                continue

            for file_name in sorted(files):
                if file_name.endswith(SOURCE_EXTENSIONS):
                    yield relative + file_name


TOKENS = re.compile(r'//[^\n]*|/\*.*?(?:\*/|\Z)|"(?:\\.|[^"\\\n])*"?|\'(?:\\.|[^\'\\\n])*\'?', re.DOTALL)
INCLUDE_PREFIX = re.compile(r'[ \t]*#[ \t]*include[ \t]*$')


def strip_comments(text):
    """Return (code, strings): the text with comments and string/char
    literals blanked out (line breaks kept), and the string literals found,
    as (line, value). #include "..." paths stay in the code."""
    strings = []
    newlines = [index for index, char in enumerate(text) if char == '\n']

    def blank(match):
        token = match.group(0)

        if token[0] == '"':
            line_start = text.rfind('\n', 0, match.start()) + 1

            if INCLUDE_PREFIX.match(text, line_start, match.start()):
                return token

            line = bisect.bisect_left(newlines, match.start()) + 1
            strings.append((line, token[1:-1] if token.endswith('"') else token[1:]))
            return '""'

        if token[0] == "'":
            return "' '"

        return re.sub(r'[^\n]', ' ', token)

    return TOKENS.sub(blank, text), strings


class Frame:
    """One #if level: whether its current branch is a guarded branch, and the
    legacy commands its PX4_MAVLINK_WIP_LEGACY() condition keeps."""

    def __init__(self, condition):
        self.enter(condition)

    def enter(self, condition):
        self.kind = guard_kind(condition)  # 'guard', 'not-guard' (#if !PX4_MAVLINK_WIP) or None
        self.guarded = self.kind == 'guard'
        self.legacy = frozenset(match.group(2) for match in LEGACY_GUARD.finditer(condition)
                                if not match.group(1))

    def other_branch(self):
        self.guarded = self.kind == 'not-guard'
        self.legacy = frozenset()


def guard_kind(condition):
    match = GUARD.search(condition)

    if not match:
        return None

    return 'not-guard' if match.group(1) else 'guard'


class ParsedFile:
    """The guard state of every line of one source file."""

    def __init__(self, path, text):
        self.path = path
        self.code, self.strings = strip_comments(text)
        self.lines = self.code.split('\n')
        self.guarded = []  # per line: inside a guarded branch
        self.waivers = []  # per line: legacy items waived (PX4_MAVLINK_WIP_LEGACY_BEGIN/END)
        self.legacy_guards = []  # per line: legacy commands kept by #if PX4_MAVLINK_WIP_LEGACY()
        self.directives = []  # (line, name, argument)
        self.includes = []  # (line, path, guarded, waivers)
        self._parse()

    def _parse(self):
        stack = []
        waivers = []
        logical = ''
        start = 0

        for number, text in enumerate(self.lines, start=1):
            guarded = any(frame.guarded for frame in stack)
            waived = frozenset(waivers)
            self.guarded.append(guarded)
            self.waivers.append(waived)
            self.legacy_guards.append(frozenset().union(*(frame.legacy for frame in stack)))

            if logical or re.match(r'^\s*#', text):
                if not logical:
                    start = number

                logical += text.rstrip('\\') + ' '

                if text.endswith('\\'):
                    continue

                match = DIRECTIVE.match(logical)
                logical = ''

                if not match:
                    continue

                name, argument = match.group(1), match.group(2)
                self.directives.append((start, name, argument))

                if name in ('if', 'ifdef', 'ifndef'):
                    stack.append(Frame(argument if name == 'if' else ''))

                elif name == 'elif' and stack:
                    stack[-1].enter(argument)

                elif name == 'else' and stack:
                    stack[-1].other_branch()

                elif name == 'endif' and stack:
                    stack.pop()

                elif name == 'include':
                    include = INCLUDE.search(argument)

                    if include:
                        self.includes.append((start, include.group(1), guarded, waived))

                continue

            for match in LEGACY_BEGIN.finditer(text):
                waivers.append(match.group(1))

            if LEGACY_END.search(text) and waivers:
                waivers.pop()


def include_coverage(parsed):
    """Files that are only included from guarded code (or legacy waivers),
    so that their uses count as guarded. Maps path -> (guarded, waivers)."""
    sites = defaultdict(list)

    for path, parsed_file in parsed.items():
        for line, include, guarded, waivers in parsed_file.includes:
            target = os.path.normpath(os.path.join(os.path.dirname(path), include)).replace(os.sep, '/')

            if target in parsed:
                sites[target].append((path, guarded, waivers))

    coverage = {}

    # iterate, so that a header included from a covered header is covered too
    for _ in range(10):
        changed = False

        for target, includers in sites.items():
            all_guarded = True
            waivers = None

            for includer, guarded, site_waivers in includers:
                includer_guarded, includer_waivers = coverage.get(includer, (False, frozenset()))
                site_waivers = site_waivers | includer_waivers

                if not (guarded or includer_guarded):
                    all_guarded = False
                    waivers = site_waivers if waivers is None else waivers & site_waivers

            value = (all_guarded, frozenset() if waivers is None or all_guarded else waivers)

            if coverage.get(target) != value:
                coverage[target] = value
                changed = True

        if not changed:
            break

    return coverage


# ----------------------------------------------------------------------------
# Inputs


def read_legacy(text):
    """Parse the legacy list: {name: {'reason': ..., ...}}."""
    import yaml

    data = yaml.safe_load(text) or {}
    entries = {}

    for entry in data.get('legacy') or []:
        entries[entry['name']] = entry

    return entries


def load_legacy():
    path = os.path.join(PX4_SOURCE_DIR, LEGACY_FILE)

    if not os.path.exists(path):
        return {}

    with open(path, encoding='utf-8') as legacy:
        return read_legacy(legacy.read())


def legacy_at(base):
    """The legacy list at git revision base, or None if it did not exist."""
    result = subprocess.run(['git', 'show', f'{base}:{LEGACY_FILE}'], cwd=PX4_SOURCE_DIR,
                            capture_output=True, text=True)

    if result.returncode != 0:
        return None

    return read_legacy(result.stdout)


def wip_command_constants(definitions):
    """Map VEHICLE_CMD_*/NAV_CMD_* constant names to the WIP MAV_CMD entry
    with the same number."""
    constants = {}

    for source in (VEHICLE_COMMAND_MSG, NAVIGATION_H):
        with open(os.path.join(PX4_SOURCE_DIR, source), encoding='utf-8') as text:
            for match in COMMAND_CONSTANT.finditer(text.read()):
                entry = definitions.mav_cmd(int(match.group(2)))

                if entry is not None and entry.wip:
                    constants[match.group(1)] = entry

    return constants


# ----------------------------------------------------------------------------
# Checks


class Checker:

    def __init__(self, definitions, legacy):
        self.definitions = definitions
        self.legacy = legacy
        self.commands = wip_command_constants(definitions)
        self.findings = []
        self.guarded_uses = defaultdict(set)  # item -> {path:line}
        self.legacy_uses = defaultdict(set)  # item -> {path:line}

    def wip_item(self, symbol):
        """The WIP item a code identifier refers to, or None."""
        if symbol in self.commands:
            return self.commands[symbol]

        if symbol.startswith(('mavlink_', 'MAVLINK_MSG_ID_')):
            item = mavlink_flags.message_for_symbol(symbol, self.definitions)
        else:
            # a bare name is only a MAVLink symbol for enums and their entries;
            # messages are reached through their generated helpers
            item = self.definitions.get(symbol)
            item = item if item is not None and item.kind != 'message' else None

        return item if item is not None and item.wip else None

    def use(self, path, line, item, guarded, waivers, legacy_guards, what, kind):
        """Record one use. kind: 'symbol' (message or enum), 'command' (a
        VEHICLE_CMD_*/NAV_CMD_* number) or 'stream' (a stream name)."""
        location = f'{path}:{line}'

        if guarded:
            self.guarded_uses[item.name].add(location)
            return

        if item.name in self.legacy:
            # a message or enum use needs the waiver to compile; a command
            # needs the legacy guard so that switching the list off removes it
            if ((kind == 'symbol' and item.name in waivers)
                    or (kind == 'command' and item.name in legacy_guards) or kind == 'stream'):
                self.legacy_uses[item.name].add(location)
                return

            if kind == 'command':
                self.findings.append(Finding(
                    path, line, f'{what} is on the legacy list but not inside its legacy guard, so '
                    '-DPX4_MAVLINK_WIP_LEGACY=OFF would not remove it.',
                    f'Add: #if PX4_MAVLINK_WIP_LEGACY({item.name}) ... #endif'))
                return

        elif item.name in waivers or item.name in legacy_guards:
            # waived or legacy-guarded but not on the legacy list: reported at the waiver or guard
            return

        self.findings.append(Finding(path, line, f'{what} is WIP ({item.xml_file}) and used outside a guard.',
                                     guard_hint(item.name)))

    def check_guard_name(self, path, line, name, legacy_waiver=False, construct=None):
        construct = construct or ('PX4_MAVLINK_WIP_LEGACY_BEGIN' if legacy_waiver else 'PX4_MAVLINK_WIP')
        item = self.definitions.get(name)

        if item is None:
            self.findings.append(Finding(path, line, f'{construct}({name}) names no MAVLink definition '
                                         '(check the spelling: an unknown name is always 0).'))

        elif not item.wip:
            self.findings.append(Finding(path, line, f'{name} has been released (no longer WIP): '
                                         f'remove its {construct}() guard.'))

        elif legacy_waiver and name not in self.legacy:
            self.findings.append(Finding(path, line, f'{name} is not on the legacy list ({LEGACY_FILE}), '
                                         'and the list may not grow.', guard_hint(name)))

    def check_file(self, parsed_file, coverage):
        path = parsed_file.path
        file_guarded, file_waivers = coverage.get(path, (False, frozenset()))

        for line, name, argument in parsed_file.directives:
            for match in GUARD.finditer(argument):
                self.check_guard_name(path, line, match.group(2))

            for match in LEGACY_GUARD.finditer(argument):
                self.check_guard_name(path, line, match.group(2), legacy_waiver=True,
                                      construct='PX4_MAVLINK_WIP_LEGACY')

            if name in ('if', 'ifdef', 'ifndef', 'elif'):
                for match in MSG_ID.finditer(argument):
                    item = self.definitions.get(match.group(1))

                    if item is not None and item.wip:
                        self.findings.append(Finding(
                            path, line, f'MAVLINK_MSG_ID_{item.name} only tells whether the dialect has '
                            f'{item.name}, not whether this is a release build.',
                            f'Replace the condition with PX4_MAVLINK_WIP({item.name}).'))

        for number, text in enumerate(parsed_file.lines, start=1):
            if re.match(r'^\s*#', text):
                continue

            guarded = parsed_file.guarded[number - 1] or file_guarded
            waivers = parsed_file.waivers[number - 1] | file_waivers
            legacy_guards = parsed_file.legacy_guards[number - 1]

            for match in LEGACY_BEGIN.finditer(text):
                self.check_guard_name(path, number, match.group(1), legacy_waiver=True)

            # the waiver's own argument is not a use
            text = LEGACY_BEGIN.sub(' ', text)

            for match in IDENTIFIER.finditer(text):
                member, symbol = match.group(1), match.group(2)

                # uORB constants mirror MAVLink entry names (vehicle_command_s::...);
                # only the command numbers matter there
                if member and symbol not in self.commands:
                    continue

                item = self.wip_item(symbol)

                if item is not None:
                    numeric = symbol in self.commands
                    what = f'{symbol} (={item.value}, {item.name})' if numeric else symbol
                    self.use(path, number, item, guarded, waivers, legacy_guards, what,
                             'command' if numeric else 'symbol')

        if path.startswith(MAVLINK_MODULE):
            for number, value in parsed_file.strings:
                item = self.definitions.get(value)

                if item is not None and item.wip and item.kind == 'message':
                    guarded = parsed_file.guarded[number - 1] or file_guarded
                    self.use(path, number, item, guarded, frozenset(), frozenset(), f'Stream name "{value}"',
                             'stream')

    def check_legacy(self, base_legacy):
        for name, entry in self.legacy.items():
            item = self.definitions.get(name)

            if item is None or not item.wip:
                self.findings.append(Finding(LEGACY_FILE, 1, f'{name} is no longer WIP: remove it from the legacy list '
                                             'and remove its PX4_MAVLINK_WIP_LEGACY_BEGIN/END waivers.'))

            elif not self.legacy_uses.get(name):
                self.findings.append(Finding(LEGACY_FILE, 1, f'{name} is no longer used unguarded: remove it from '
                                             'the legacy list.'))

            if not entry.get('reason'):
                self.findings.append(Finding(LEGACY_FILE, 1, f'{name} has no reason.'))

        if base_legacy is not None:
            for name in sorted(set(self.legacy) - set(base_legacy)):
                self.findings.append(Finding(LEGACY_FILE, 1, f'{name} was added to the legacy list, which may '
                                             'only shrink. Guard the new use instead.', guard_hint(name)))


def parse_sources():
    parsed = {}

    for path in source_files():
        with open(os.path.join(PX4_SOURCE_DIR, path), encoding='utf-8', errors='replace') as source:
            text = source.read()

        if any(token in text for token in ('PX4_MAVLINK_WIP', 'mavlink', 'MAVLINK', 'MAV_', '_CMD_')):
            parsed[path] = ParsedFile(path, text)

    return parsed


def list_guarded(parsed):
    """Guarded uses by guard name, found without reading the XML."""
    uses = defaultdict(set)

    for path, parsed_file in parsed.items():
        for line, name, argument in parsed_file.directives:
            for match in GUARD.finditer(argument):
                if match.group(2):
                    uses[match.group(2)].add(f'{path}:{line}')

    return uses


def write_summary(path, checker):
    lines = ['## MAVLink WIP check\n']

    if checker.findings:
        lines.append(f'**{len(checker.findings)} problem(s)**: WIP MAVLink definitions must be guarded so that '
                     'they are excluded from releases. See the comment at the top of the generated '
                     '`mavlink_wip.h`. Reproduce locally with `make mavlink_wip_check`.\n')

        for finding in checker.findings:
            lines.append(f'- `{finding.path}:{finding.line}` {finding.message}' +
                         (f' {finding.hint}' if finding.hint else ''))

        lines.append('')
    else:
        lines.append('No unguarded WIP uses found by the lint.\n')

    if checker.guarded_uses:
        lines.append('<details><summary>WIP definitions used in source but excluded from beta and release '
                     f'builds ({len(checker.guarded_uses)})</summary>\n')

        for name in sorted(checker.guarded_uses):
            lines.append(f'- {name}: ' + ', '.join(sorted(checker.guarded_uses[name])[:5]) +
                         (' ...' if len(checker.guarded_uses[name]) > 5 else ''))

        lines.append('\n</details>\n')

    if checker.legacy_uses:
        lines.append(f'<details><summary>Legacy WIP uses that ship unguarded ({len(checker.legacy_uses)}, '
                     f'listed in {LEGACY_FILE})</summary>\n')

        for name in sorted(checker.legacy_uses):
            lines.append(f'- {name}: ' + ', '.join(sorted(checker.legacy_uses[name])[:5]))

        lines.append('\n</details>\n')

    with open(path, 'a', encoding='utf-8') as summary:
        summary.write('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--base', help='git revision to compare the legacy list with (it may only shrink)')
    parser.add_argument('--dialect', default=mavlink_flags.DEFAULT_DIALECT,
                        help='dialect whose include tree defines the known definitions (default: %(default)s)')
    parser.add_argument('--mavlink-dir', default=mavlink_flags.DEFAULT_MAVLINK_DIR,
                        help='mavlink/mavlink checkout (default: the PX4 submodule)')
    parser.add_argument('--list-guarded', action='store_true',
                        help='only list the WIP features guarded in source, one per line')
    parser.add_argument('--summary', help='append a Markdown report to this file (e.g. $GITHUB_STEP_SUMMARY)')
    args = parser.parse_args()

    parsed = parse_sources()

    if args.list_guarded:
        for name, uses in sorted(list_guarded(parsed).items()):
            print(f'{name} (guarded in {len(uses)} place{"s" if len(uses) > 1 else ""}, '
                  f'e.g. {min(uses)})')
        return 0

    definitions = mavlink_flags.load(args.dialect, args.mavlink_dir)
    checker = Checker(definitions, load_legacy())
    coverage = include_coverage(parsed)

    for parsed_file in parsed.values():
        checker.check_file(parsed_file, coverage)

    checker.check_legacy(legacy_at(args.base) if args.base else None)

    for finding in checker.findings:
        print(finding)

    if args.summary:
        write_summary(args.summary, checker)

    if checker.findings:
        print(f'\n{len(checker.findings)} MAVLink WIP problem(s). WIP definitions must be guarded so that they are '
              'excluded from releases: see the comment at the top of the generated mavlink_wip.h. '
              'Run "make mavlink_wip_check" to also run a release-stage build.', file=sys.stderr)
        return 1

    print(f'MAVLink WIP check passed: {len(checker.guarded_uses)} WIP definition(s) used behind guards, '
          f'{len(checker.legacy_uses)} legacy item(s) shipping unguarded.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
