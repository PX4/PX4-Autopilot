#!/usr/bin/env python3
"""
Lifecycle flags (wip, deprecated, superseded) of the MAVLink definitions in a
dialect and everything it includes, read with pymavlink's own parser.

Shared by generate_wip_guards.py (which writes the PX4_MAVLINK_WIP() guard
header), check_wip.py (the source lint) and wip_build_report.py (the
compiler-output report). Only WIP is acted on today; deprecated and superseded
are recorded so that later checks can be built on the same data.
"""

import contextlib
import io
import os
import sys
from dataclasses import dataclass, field

WIP = 'wip'
DEPRECATED = 'deprecated'
SUPERSEDED = 'superseded'
FLAGS = (WIP, DEPRECATED, SUPERSEDED)

PX4_SOURCE_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..'))
DEFAULT_MAVLINK_DIR = os.path.join(PX4_SOURCE_DIR, 'src', 'modules', 'mavlink', 'mavlink')

# The dialect whose include tree holds every definition PX4 can build with.
DEFAULT_DIALECT = 'development'


@dataclass
class Item:
    """A message, enum or enum entry, with the flags that apply to it."""
    name: str
    kind: str  # 'message', 'enum' or 'entry'
    xml_file: str
    flags: set = field(default_factory=set)
    enum: str = ''  # for entries: the enum they belong to
    value: int = None  # for entries: their value

    @property
    def wip(self):
        return WIP in self.flags


class Definitions:
    """All definitions in a dialect's include tree, keyed by name."""

    def __init__(self):
        self.items = {}
        self.xml_files = []

    def add(self, item):
        existing = self.items.get(item.name)
        if existing is None:
            self.items[item.name] = item
        else:
            # An enum can be extended in several files. Keep the first
            # declaration and add the flags of the others.
            existing.flags |= item.flags

    def get(self, name):
        return self.items.get(name)

    def wip_items(self):
        return [item for item in self.items.values() if item.wip]

    def messages(self):
        return [item for item in self.items.values() if item.kind == 'message']

    def mav_cmd(self, value):
        """The MAV_CMD entry with this value, or None."""
        for item in self.items.values():
            if item.kind == 'entry' and item.enum == 'MAV_CMD' and item.value == value:
                return item
        return None


def _flags_of(element):
    return {flag for flag in FLAGS if getattr(element, flag, None)}


def _import_mavparse(mavlink_dir):
    if mavlink_dir not in sys.path:
        sys.path.insert(0, mavlink_dir)
    try:
        from pymavlink.generator import mavparse
    except ImportError as error:
        raise SystemExit(f'cannot import pymavlink from {mavlink_dir} ({error}). '
                         'Is the src/modules/mavlink/mavlink submodule checked out '
                         '(git submodule update --init --recursive src/modules/mavlink/mavlink)?')
    return mavparse


def load(dialect=DEFAULT_DIALECT, mavlink_dir=DEFAULT_MAVLINK_DIR):
    """Parse <dialect>.xml and its includes and return their Definitions."""
    mavparse = _import_mavparse(mavlink_dir)
    xml_dir = os.path.join(mavlink_dir, 'message_definitions', 'v1.0')
    definitions = Definitions()
    pending = [dialect + '.xml']
    seen = set()

    while pending:
        file_name = pending.pop(0)
        if file_name in seen:
            continue
        seen.add(file_name)
        path = os.path.join(xml_dir, file_name)
        if not os.path.exists(path):
            raise SystemExit(f'MAVLink definition file not found: {path}')
        # mavparse prints progress and validation notes (already reported by
        # mavgen); keep them out of the build output. Errors still raise.
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            xml = mavparse.MAVXML(path, '2.0')
        definitions.xml_files.append(path)

        for message in xml.message:
            definitions.add(Item(message.name, 'message', file_name, _flags_of(message)))

        for enum in xml.enum:
            enum_flags = _flags_of(enum)
            definitions.add(Item(enum.name, 'enum', file_name, set(enum_flags)))

            for entry in enum.entry:
                if entry.end_marker:
                    continue
                definitions.add(Item(entry.name, 'entry', file_name, enum_flags | _flags_of(entry),
                                     enum=enum.name, value=entry.value))

        pending.extend(xml.include)

    return definitions


def message_for_symbol(symbol, definitions):
    """The message whose generated helper (mavlink_msg_<name>_*), struct
    (mavlink_<name>_t) or id constant (MAVLINK_MSG_ID_<NAME>) this symbol is,
    or None. The longest matching name wins, so mavlink_msg_esc_info_send is
    ESC_INFO even if a message called ESC existed."""
    if symbol.startswith('MAVLINK_MSG_ID_'):
        item = definitions.get(symbol[len('MAVLINK_MSG_ID_'):])
        return item if item is not None and item.kind == 'message' else None

    if symbol.startswith('mavlink_msg_'):
        rest = symbol[len('mavlink_msg_'):].upper()
        # the message name is followed by _<helper>, so try every '_' split,
        # longest first
        for end in range(len(rest) - 1, 0, -1):
            if rest[end] == '_':
                item = definitions.get(rest[:end])

                if item is not None and item.kind == 'message':
                    return item

        return None

    if symbol.startswith('mavlink_') and symbol.endswith('_t'):
        item = definitions.get(symbol[len('mavlink_'):-len('_t')].upper())
        return item if item is not None and item.kind == 'message' else None

    return None
