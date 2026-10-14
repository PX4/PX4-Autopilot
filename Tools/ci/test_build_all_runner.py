"""Check build_all_runner.py's slot split and what it reports per build.

A wrong split fails silently: two slots writing the same build/ directory
overwrite each other's firmware, and an unbalanced split throws away the
time the slots are meant to save. With two builds sharing one log stream,
the failure excerpt and memory report printed per target are what people
read, so they must contain the actual error and the full memory table.
"""

import unittest

from build_all_runner import (assign_slots, check_plan, failure_excerpt,
                              flash_usage, memory_report)

# ninja keeps printing the jobs already running after the failing one
NINJA_FAILURE = (
    ['[1/990] Building C object src/lib/crc/crc.c.obj'] +
    ['FAILED: src/modules/ekf2/CMakeFiles/ekf2.dir/EKF2.cpp.obj',
     '/usr/bin/arm-none-eabi-g++ -c ../../src/modules/ekf2/EKF2.cpp',
     '../../src/modules/ekf2/EKF2.cpp:42:5: \x1b[01;31m\x1b[Kerror: \x1b[m\x1b[K'
     "'foo' was not declared in this scope"] +
    [f'[{n}/990] Building CXX object src/modules/other{n}.cpp.obj' for n in range(2, 80)] +
    ['ninja: build stopped: subcommand failed.',
     'make: *** [Makefile:232: atl_mantis-edu_default] Error 1']
)

MEMORY_TAIL = [
    '[988/990] Linking CXX executable atl_mantis-edu_default.elf',
    'Memory region         Used Size  Region Size  %age Used',
    '   FLASH_ITCM:           0 B      1952 KB      0.00%',
    '   FLASH_AXIM:     1405156 B      1952 KB     70.30%',
    '        SRAM1:       48272 B       368 KB     12.81%',
    '[989/990] Generating ../../atl_mantis-edu_default.bin',
]


class FailureExcerptTest(unittest.TestCase):
    def test_starts_at_the_error_not_the_tail(self):
        excerpt = failure_excerpt(NINJA_FAILURE)
        self.assertTrue(excerpt[0].startswith('FAILED:'))
        self.assertTrue(any("'foo' was not declared" in line for line in excerpt))

    def test_skips_other_jobs_progress_lines(self):
        excerpt = failure_excerpt(NINJA_FAILURE)
        self.assertFalse(any('Building CXX object src/modules/other' in l for l in excerpt))

    def test_keeps_the_make_exit_line(self):
        self.assertEqual(failure_excerpt(NINJA_FAILURE)[-1], NINJA_FAILURE[-1])

    def test_no_error_line_falls_back_to_tail(self):
        lines = [f'line {n}' for n in range(100)]
        self.assertEqual(failure_excerpt(lines)[-1], 'line 99')


class MemoryReportTest(unittest.TestCase):
    def test_extracts_whole_table(self):
        report = memory_report(MEMORY_TAIL)
        self.assertEqual(report, MEMORY_TAIL[1:5])
        self.assertEqual(flash_usage(report), 70.30)

    def test_posix_build_has_no_report(self):
        self.assertEqual(memory_report(['[1/2] Linking CXX executable px4']), [])
        self.assertIsNone(flash_usage([]))

MISC_STM32F7 = [
    'atl_mantis-edu_default', 'av_x-v1_default', 'corvon_v5_default',
    'cubepilot_cubeyellow_default', 'freefly_can-rtk-gps_canbootloader',
    'freefly_can-rtk-gps_default', 'holybro_kakutef7_default',
    'holybro_pix32v5_default', 'modalai_fc-v1_default', 'mro_ctrl-zero-f7_default',
    'mro_ctrl-zero-f7-oem_default', 'mro_x21-777_default', 'radiolink_PIX6_default',
    'sky-drones_smartap-airlink_default',
]


class AssignSlotsTest(unittest.TestCase):
    def test_one_slot_keeps_order(self):
        self.assertEqual(assign_slots(MISC_STM32F7, 1), [MISC_STM32F7])

    def test_two_slots_split_evenly_in_order(self):
        plan = assign_slots(MISC_STM32F7, 2)
        self.assertEqual([len(p) for p in plan], [7, 7])
        self.assertEqual(plan[0] + plan[1], MISC_STM32F7)

    def test_uneven_count_differs_by_at_most_one(self):
        plan = assign_slots(MISC_STM32F7[:13], 3)
        sizes = [len(p) for p in plan]
        self.assertEqual(sum(sizes), 13)
        self.assertLessEqual(max(sizes) - min(sizes), 1)

    def test_deb_stays_with_default(self):
        # make modalai_voxl2_deb builds into build/modalai_voxl2_default
        plan = assign_slots(['modalai_voxl2_default', 'modalai_voxl2_deb'], 2)
        self.assertEqual(plan, [['modalai_voxl2_default', 'modalai_voxl2_deb']])

    def test_metadata_stays_with_sitl_default(self):
        # the metadata rules all build inside build/px4_sitl_default
        base = ['airframe_metadata', 'parameters_metadata', 'extract_events',
                'px4_sitl_allyes', 'px4_sitl_default', 'px4_sitl_sih']
        for slots in (2, 3, 4):
            plan = assign_slots(base, slots)
            sitl = [p for p in plan if 'px4_sitl_default' in p][0]
            self.assertTrue({'airframe_metadata', 'parameters_metadata',
                             'extract_events'} <= set(sitl), plan)

    def test_plan_splitting_a_build_dir_is_refused(self):
        with self.assertRaises(SystemExit):
            check_plan([['modalai_voxl2_default'], ['modalai_voxl2_deb']])
        with self.assertRaises(SystemExit):
            check_plan([['px4_sitl_default'], ['parameters_metadata']])
        check_plan(assign_slots(['airframe_metadata', 'px4_sitl_allyes',
                                 'px4_sitl_default', 'modalai_voxl2_default',
                                 'modalai_voxl2_deb'], 4))

    def test_more_slots_than_targets(self):
        self.assertEqual(assign_slots(['a', 'b'], 4), [['a'], ['b']])


if __name__ == '__main__':
    unittest.main()
