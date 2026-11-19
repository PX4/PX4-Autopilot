#!/usr/bin/env python3
"""Tests for the MAVLink WIP tools: mavlink_flags, generate_wip_guards,
check_wip and wip_build_report. They use made-up definitions, so they do not
need the mavlink submodule."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import check_wip  # noqa: E402
import generate_wip_guards  # noqa: E402
import mavlink_flags  # noqa: E402
import wip_build_report  # noqa: E402
from mavlink_flags import Item  # noqa: E402


def definitions():
    result = mavlink_flags.Definitions()

    for item in (
            Item('ESC', 'message', 'common.xml'),
            Item('ESC_INFO', 'message', 'common.xml'),
            Item('ESC_EEPROM', 'message', 'development.xml', {'wip'}),
            Item('BATTERY_INFO', 'message', 'common.xml', {'wip'}),
            Item('HEARTBEAT', 'message', 'minimal.xml'),
            Item('ESC_FIRMWARE', 'enum', 'development.xml', {'wip'}),
            Item('ESC_FIRMWARE_AM32', 'entry', 'development.xml', {'wip'}, enum='ESC_FIRMWARE', value=1),
            Item('MAV_CMD', 'enum', 'common.xml'),
            Item('MAV_CMD_NAV_WAYPOINT', 'entry', 'common.xml', set(), enum='MAV_CMD', value=16),
            Item('MAV_CMD_ACTUATOR_GROUP_TEST', 'entry', 'development.xml', {'wip'}, enum='MAV_CMD', value=309),
    ):
        result.add(item)

    return result


def check(files, legacy=None, base_legacy=None):
    """Run the lint on {path: text} and return the finding messages."""
    checker = check_wip.Checker(definitions(), legacy or {})
    parsed = {path: check_wip.ParsedFile(path, text) for path, text in files.items()}
    coverage = check_wip.include_coverage(parsed)

    for parsed_file in parsed.values():
        checker.check_file(parsed_file, coverage)

    checker.check_legacy(base_legacy)
    return [f'{finding.path}:{finding.line} {finding.message}' for finding in checker.findings]


class MessageForSymbolTest(unittest.TestCase):

    def test_helpers_struct_and_id(self):
        d = definitions()
        self.assertEqual(mavlink_flags.message_for_symbol('mavlink_msg_esc_eeprom_send_struct', d).name, 'ESC_EEPROM')
        self.assertEqual(mavlink_flags.message_for_symbol('mavlink_msg_esc_eeprom_get_index', d).name, 'ESC_EEPROM')
        self.assertEqual(mavlink_flags.message_for_symbol('mavlink_esc_eeprom_t', d).name, 'ESC_EEPROM')
        self.assertEqual(mavlink_flags.message_for_symbol('MAVLINK_MSG_ID_ESC_EEPROM', d).name, 'ESC_EEPROM')

    def test_longest_name_wins(self):
        d = definitions()
        self.assertEqual(mavlink_flags.message_for_symbol('mavlink_msg_esc_info_send', d).name, 'ESC_INFO')
        self.assertEqual(mavlink_flags.message_for_symbol('mavlink_msg_esc_send', d).name, 'ESC')

    def test_not_a_message(self):
        d = definitions()
        self.assertIsNone(mavlink_flags.message_for_symbol('mavlink_msg_unknown_send', d))
        self.assertIsNone(mavlink_flags.message_for_symbol('MAVLINK_MSG_ID_ESC_FIRMWARE', d))
        self.assertIsNone(mavlink_flags.message_for_symbol('mavlink_send_uart_bytes', d))


class GenerateTest(unittest.TestCase):

    def test_one_define_per_wip_item(self):
        with mock.patch.object(mavlink_flags, 'load', return_value=definitions()):
            header = generate_wip_guards.generate('development', '')

        self.assertIn('#define PX4_MAVLINK_WIP_ITEM_ESC_EEPROM 1 // message, development.xml', header)
        self.assertIn('#define PX4_MAVLINK_WIP_ITEM_MAV_CMD_ACTUATOR_GROUP_TEST 1', header)
        self.assertNotIn('PX4_MAVLINK_WIP_ITEM_HEARTBEAT', header)
        self.assertIn('#define PX4_MAVLINK_WIP(name)', header)


class StripCommentsTest(unittest.TestCase):

    def test_comments_strings_and_includes(self):
        code, strings = check_wip.strip_comments(
            '#include "streams/ESC_EEPROM.hpp" // ESC_EEPROM\n'
            '/* mavlink_msg_esc_eeprom_send\n   more */ int a = 1;\n'
            'f("ESC_EEPROM", \'x\');\n')
        self.assertIn('"streams/ESC_EEPROM.hpp"', code)
        self.assertNotIn('mavlink_msg_esc_eeprom_send', code)
        self.assertEqual(code.count('\n'), 4)
        self.assertEqual(strings, [(4, 'ESC_EEPROM')])


class GuardTrackingTest(unittest.TestCase):

    def test_branches(self):
        parsed = check_wip.ParsedFile('a.cpp', '\n'.join([
            'a;',                                   # 1
            '#if PX4_MAVLINK_WIP(ESC_EEPROM)',      # 2
            'b;',                                   # 3
            '#  if defined(OTHER)',                 # 4
            'c;',                                   # 5
            '#  endif',                             # 6
            '#else',                                # 7
            'd;',                                   # 8
            '#endif',                               # 9
            '#if !PX4_MAVLINK_WIP(ESC_EEPROM)',     # 10
            'e;',                                   # 11
            '#else',                                # 12
            'f;',                                   # 13
            '#endif',                               # 14
        ]))
        guarded = {line: parsed.guarded[line - 1] for line in (1, 3, 5, 8, 11, 13)}
        self.assertEqual(guarded, {1: False, 3: True, 5: True, 8: False, 11: False, 13: True})

    def test_waiver_region(self):
        parsed = check_wip.ParsedFile('a.cpp', 'PX4_MAVLINK_WIP_LEGACY_BEGIN(BATTERY_INFO)\nx;\n'
                                      'PX4_MAVLINK_WIP_LEGACY_END\ny;\n')
        self.assertEqual(parsed.waivers[1], {'BATTERY_INFO'})
        self.assertEqual(parsed.waivers[3], set())


class CheckTest(unittest.TestCase):

    def test_unguarded_use(self):
        findings = check({'src/a.cpp': 'void f() { mavlink_msg_esc_eeprom_send_struct(0, 0); }\n'})
        self.assertEqual(len(findings), 1)
        self.assertIn('mavlink_msg_esc_eeprom_send_struct is WIP (development.xml)', findings[0])

    def test_guarded_use(self):
        self.assertEqual(check({'src/a.cpp': '#if PX4_MAVLINK_WIP(ESC_EEPROM)\n'
                                'void f() { mavlink_msg_esc_eeprom_send_struct(0, 0); }\n#endif\n'}), [])

    def test_released_and_comment_uses_are_fine(self):
        self.assertEqual(check({'src/a.cpp': 'int x = MAVLINK_MSG_ID_HEARTBEAT; // ESC_EEPROM\n'}), [])

    def test_wip_enum_entry(self):
        findings = check({'src/a.cpp': 'int x = ESC_FIRMWARE_AM32;\n'})
        self.assertEqual(len(findings), 1)
        self.assertIn('ESC_FIRMWARE_AM32 is WIP', findings[0])

    def test_uorb_mirror_of_entry_name_is_fine(self):
        self.assertEqual(check({'src/a.cpp': 'int x = esc_s::ESC_FIRMWARE_AM32;\n'}), [])

    def test_old_guard(self):
        findings = check({'src/a.cpp': '#if defined(MAVLINK_MSG_ID_ESC_EEPROM)\n#endif\n'})
        self.assertEqual(len(findings), 1)
        self.assertIn('only tells whether the dialect has ESC_EEPROM', findings[0])

    def test_guard_for_released_or_unknown_item(self):
        findings = check({'src/a.cpp': '#if PX4_MAVLINK_WIP(HEARTBEAT)\n#endif\n'
                                       '#if PX4_MAVLINK_WIP(ESC_EPROM)\n#endif\n'})
        self.assertEqual(len(findings), 2)
        self.assertIn('HEARTBEAT has been released', findings[0])
        self.assertIn('ESC_EPROM) names no MAVLink definition', findings[1])

    def test_command_by_number(self):
        findings = check({'src/a.cpp': 'if (cmd == vehicle_command_s::VEHICLE_CMD_ACTUATOR_GROUP_TEST) {}\n'})
        self.assertEqual(len(findings), 1)
        self.assertIn('VEHICLE_CMD_ACTUATOR_GROUP_TEST (=309, MAV_CMD_ACTUATOR_GROUP_TEST) is WIP', findings[0])

    def test_stream_name(self):
        findings = check({'src/modules/mavlink/a.cpp': 'const char *s = "ESC_EEPROM";\n',
                          'src/other.cpp': 'const char *s = "ESC_EEPROM";\n'})
        self.assertEqual(len(findings), 1)
        self.assertIn('src/modules/mavlink/a.cpp:1 Stream name "ESC_EEPROM" is WIP', findings[0])

    def test_header_included_only_under_guard(self):
        files = {
            'src/m/main.cpp': '#if PX4_MAVLINK_WIP(ESC_EEPROM)\n#include "streams/ESC_EEPROM.hpp"\n#endif\n',
            'src/m/streams/ESC_EEPROM.hpp': 'void f() { mavlink_msg_esc_eeprom_send_struct(0, 0); }\n',
        }
        self.assertEqual(check(files), [])

    def test_header_also_included_unguarded(self):
        files = {
            'src/m/main.cpp': '#if PX4_MAVLINK_WIP(ESC_EEPROM)\n#include "streams/ESC_EEPROM.hpp"\n#endif\n',
            'src/m/other.cpp': '#include "streams/ESC_EEPROM.hpp"\n',
            'src/m/streams/ESC_EEPROM.hpp': 'void f() { mavlink_msg_esc_eeprom_send_struct(0, 0); }\n',
        }
        self.assertEqual(len(check(files)), 1)


class LegacyTest(unittest.TestCase):
    LEGACY = {
        'BATTERY_INFO': {'name': 'BATTERY_INFO', 'reason': 'common.xml WIP'},
        'MAV_CMD_ACTUATOR_GROUP_TEST': {'name': 'MAV_CMD_ACTUATOR_GROUP_TEST', 'reason': 'by number'},
    }
    USES = {
        'src/modules/mavlink/a.cpp': 'PX4_MAVLINK_WIP_LEGACY_BEGIN(BATTERY_INFO)\n'
                                     'void f() { mavlink_msg_battery_info_send_struct(0, 0); }\n'
                                     'PX4_MAVLINK_WIP_LEGACY_END\n',
        'src/b.cpp': 'int c = vehicle_command_s::VEHICLE_CMD_ACTUATOR_GROUP_TEST;\n',
    }

    def test_waived_and_numeric_legacy_uses(self):
        self.assertEqual(check(self.USES, self.LEGACY), [])

    def test_legacy_symbol_still_needs_a_waiver(self):
        findings = check({'src/a.cpp': 'void f() { mavlink_msg_battery_info_send_struct(0, 0); }\n',
                          'src/b.cpp': self.USES['src/b.cpp']}, self.LEGACY)
        self.assertEqual(len(findings), 2)  # the use, and the entry now has no waived use
        self.assertIn('mavlink_msg_battery_info_send_struct is WIP', findings[0])

    def test_waiver_not_on_list(self):
        findings = check({'src/a.cpp': 'PX4_MAVLINK_WIP_LEGACY_BEGIN(ESC_EEPROM)\n'
                          'void f() { mavlink_msg_esc_eeprom_send_struct(0, 0); }\nPX4_MAVLINK_WIP_LEGACY_END\n'})
        self.assertEqual(len(findings), 1)
        self.assertIn('ESC_EEPROM is not on the legacy list', findings[0])

    def test_list_may_only_shrink(self):
        base = {'BATTERY_INFO': self.LEGACY['BATTERY_INFO']}
        findings = check(self.USES, self.LEGACY, base_legacy=base)
        self.assertEqual(len(findings), 1)
        self.assertIn('MAV_CMD_ACTUATOR_GROUP_TEST was added to the legacy list', findings[0])

    def test_unused_or_released_entries(self):
        legacy = dict(self.LEGACY, HEARTBEAT={'name': 'HEARTBEAT', 'reason': 'x'})
        findings = check({'src/b.cpp': self.USES['src/b.cpp']}, legacy)
        self.assertEqual(len(findings), 2)
        self.assertIn('BATTERY_INFO is no longer used unguarded', findings[0])
        self.assertIn('HEARTBEAT is no longer WIP', findings[1])

    def test_read_legacy(self):
        entries = check_wip.read_legacy('legacy:\n  - name: BATTERY_INFO\n    reason: r\n')
        self.assertEqual(entries, {'BATTERY_INFO': {'name': 'BATTERY_INFO', 'reason': 'r'}})


class BuildReportTest(unittest.TestCase):

    def test_gcc_lines(self):
        lines = [
            "/px4/src/modules/mavlink/streams/X.hpp:87:3: error: ‘void mavlink_msg_esc_eeprom_send_struct("
            "mavlink_channel_t, const mavlink_esc_eeprom_t*)’ is deprecated: MAVLink WIP item used outside a guard. "
            "Wrap it in '#if PX4_MAVLINK_WIP(<NAME>)' [-Werror=deprecated-declarations]",
            "a.cpp:5:9: error: 'ESC_FIRMWARE_AM32' is deprecated: MAVLink WIP item used outside a guard.",
        ]
        d = definitions()
        found = []

        for line in lines:
            match = wip_build_report.DEPRECATED.search(line)
            self.assertIsNotNone(match, line)
            found.append((match.group('line'), wip_build_report.item_for_symbol(match.group('symbol'), d)))

        self.assertEqual(found, [('87', ('ESC_EEPROM', 'development.xml')),
                                 ('5', ('ESC_FIRMWARE_AM32', 'development.xml'))])

    def test_clang_line(self):
        line = ("t.cpp:14:26: error: 'mavlink_msg_esc_eeprom_send' is deprecated: MAVLink WIP item used outside "
                "a guard [-Werror,-Wdeprecated-declarations]")
        match = wip_build_report.DEPRECATED.search(line)
        self.assertEqual(wip_build_report.item_for_symbol(match.group('symbol'), definitions())[0], 'ESC_EEPROM')

    def test_legacy_note(self):
        gcc = ("x.cpp:6:1: note: ‘#pragma message: Legacy MAVLink WIP item BATTERY_INFO ships unguarded "
               "in this build (see mavlink_wip_legacy.yaml)’")
        clang = ("t.cpp:6:1: warning: Legacy MAVLink WIP item BATTERY_INFO ships unguarded in this build "
                 "(see mavlink_wip_legacy.yaml) [-W#pragma-messages]")
        self.assertEqual(wip_build_report.LEGACY.findall(gcc), ['BATTERY_INFO'])
        self.assertEqual(wip_build_report.LEGACY.findall(clang), ['BATTERY_INFO'])


if __name__ == '__main__':
    unittest.main()
