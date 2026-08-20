#!/usr/bin/env python3
"""
Mission upload/download/clear stress test for on-bench PX4 hardware.

v1.18 risk area: dataman and the mission shared-state path were reworked and
a brand-new mutex now guards mission shared state (#27813). A regression
there shows up as a SILENT HANG (a MISSION_REQUEST that never arrives, a
MISSION_ACK that never comes, a download that stalls mid-list) or as a
corrupted round-trip. Repetition is the test: single-shot operations do not
hit races, so this script repeatedly uploads a large mission, reads it back
and compares it item-by-item, then clears it, with a hard timeout on every
wait so a stall becomes a FAIL naming the stalled step and the seq reached.

With a second connection given, iterations alternate between the two links
(iteration parity picks the link) to exercise the new mission shared-state
mutex from two MAVLink channels.

Usage:
    mission_stress.py CONNECTION [CONNECTION2] [-b BAUD]
                      [--baudrate2 BAUD2] [--iterations N] [--items K]
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import px4bench
from px4bench import missions

DEFAULT_ITERATIONS = 10
DEFAULT_ITEMS = 220        # <= smallest CONFIG_NUM_MISSION_ITMES_SUPPORTED (500)
DEFAULT_BAUD2 = 57600


def run_iteration(report, mav, iteration, items_k, link_label):
    """One full upload/download/compare/clear cycle. Returns True on full pass."""
    expected = missions.generate_mission(iteration, items_k)

    up_ok, up_dur, up_detail = missions.upload_mission(report, mav, expected, iteration)
    if not report.check('iter{}_upload'.format(iteration), up_ok,
                        '[{}] {} ({:.1f}s)'.format(link_label, up_detail, up_dur)):
        return False

    downloaded, dl_dur, dl_detail = missions.download_mission(mav)
    if downloaded is None:
        report.fail('iter{}_download'.format(iteration),
                    '[{}] {} ({:.1f}s)'.format(link_label, dl_detail, dl_dur))
        return False

    cmp_ok, cmp_detail = missions.compare_mission(expected, downloaded)
    if not report.check('iter{}_compare'.format(iteration), cmp_ok,
                        '[{}] {}'.format(link_label, cmp_detail)):
        return False

    clr_ok, clr_detail = missions.clear_mission(mav)
    if not report.check('iter{}_clear'.format(iteration), clr_ok,
                        '[{}] {}'.format(link_label, clr_detail)):
        return False

    vfy_ok, vfy_detail = missions.verify_cleared(mav)
    report.check('iter{}_verify_clear'.format(iteration), vfy_ok,
                 '[{}] {}'.format(link_label, vfy_detail))

    if up_ok and downloaded is not None and cmp_ok and clr_ok and vfy_ok:
        report.info('iter {} PASS [{}] upload {:.1f}s download {:.1f}s'.format(
            iteration, link_label, up_dur, dl_dur))
        return True
    return False


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    px4bench.add_connection_args(parser, dual_link=True)
    parser.add_argument('--baudrate2', type=int, default=DEFAULT_BAUD2,
                        help='baud rate for the second connection (default: %(default)s)')
    parser.add_argument('--iterations', type=int, default=DEFAULT_ITERATIONS,
                        help='upload/download/clear iterations (default: %(default)s)')
    parser.add_argument('--items', type=int, default=DEFAULT_ITEMS,
                        help='mission items per iteration (default: %(default)s)')
    args = parser.parse_args()

    report = px4bench.Reporter('mission_stress')

    try:
        mav1 = px4bench.connect(args.connection, baud=args.baudrate,
                                timeout=args.connect_timeout)
    except (TimeoutError, OSError) as e:
        report.fail('connect', 'link 1 {}: {}'.format(args.connection, e))
        sys.exit(report.finish())
    report.info('link 1 connected to system {} component {}'.format(
        mav1.target_system, mav1.target_component))

    mav2 = None
    if args.connection2:
        try:
            mav2 = px4bench.connect(args.connection2, baud=args.baudrate2,
                                    timeout=args.connect_timeout)
            report.info('link 2 connected to system {} component {}'.format(
                mav2.target_system, mav2.target_component))
        except (TimeoutError, OSError) as e:
            report.fail('connect', 'link 2 {}: {}'.format(args.connection2, e))
            sys.exit(report.finish())

    try:
        for iteration in range(args.iterations):
            if mav2 is not None and (iteration % 2 == 1):
                mav, label = mav2, 'link2'
            else:
                mav, label = mav1, 'link1'
            run_iteration(report, mav, iteration, args.items, label)
    finally:
        for m in (mav1, mav2):
            try:
                if m is not None:
                    m.close()
            except Exception:
                pass

    sys.exit(report.finish())


if __name__ == '__main__':
    main()
