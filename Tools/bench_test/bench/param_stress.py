#!/usr/bin/env python3
"""
Parameter-subsystem stress test for on-bench PX4 hardware.

v1.18 risk area: the parameters backend received a concurrency and locking
rework (DynamicSparseLayer races, AtomicTransaction). A regression there
shows up as a SILENT HANG (a PARAM_VALUE that never arrives, a full download
that stalls halfway) or as a dropped write that only surfaces after a
reboot. Repetition is the test: single-shot operations do not hit races, so
this script hammers the parameter protocol over MAVLink with hard timeouts
on every wait, and a stall becomes a named FAIL instead of a hung process.

Three phases:
  1. Full parameter download (PARAM_REQUEST_LIST), stall-detected.
  2. Set / readback loop on a harmless scratch parameter.
  3. Persistence across a reboot (skippable with --skip-reboot).

The scratch parameter (SDLOG_UTC_OFFSET) only shifts log-file naming
timestamps, so it is safe to thrash on a bench. Its original value is
always restored while a connection is alive, even on failure.

Usage:
    param_stress.py CONNECTION [-b BAUD] [--iterations N]
                    [--param NAME] [--skip-reboot]
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import px4bench
from px4bench.params import (READ_TIMEOUT_S, SET_ECHO_TIMEOUT_S,
                             drain_param_values, param_float_to_int32,
                             param_id_str, param_is_saved, read_param,
                             read_until, set_param_int32, wait_param_echo)

COMMIT_TIMEOUT_S = 8.0

DEFAULT_PARAM = 'SDLOG_UTC_OFFSET'
DEFAULT_ITERATIONS = 50

# Persistence-phase marker value (within SDLOG_UTC_OFFSET min/max -1000..1000).
MARKER_VALUE = 777

# Download stall thresholds.
DOWNLOAD_STALL_S = 10.0      # FAIL if no new param index for this long
DOWNLOAD_OVERALL_CAP_S = 180.0


def phase_full_download(report, mav):
    """PARAM_REQUEST_LIST; collect until param_count reached, stall-detected.

    Returns a dict {name: int32_value} of everything received (best effort),
    so later phases can confirm the scratch parameter exists.
    """
    report.info('Phase 1: full parameter download')
    mav.mav.param_request_list_send(mav.target_system, mav.target_component)

    received = {}          # index -> (name, int32_value)
    by_name = {}           # name -> int32_value
    advertised = None
    start = time.monotonic()
    last_new = start

    while True:
        now = time.monotonic()
        if now - start > DOWNLOAD_OVERALL_CAP_S:
            report.fail('param_download',
                        'overall cap {:.0f}s hit with {}/{} params'.format(
                            DOWNLOAD_OVERALL_CAP_S, len(received),
                            advertised if advertised is not None else '?'))
            return by_name
        if now - last_new > DOWNLOAD_STALL_S:
            report.fail('param_download',
                        'stalled: no new param for {:.0f}s at {}/{}'.format(
                            DOWNLOAD_STALL_S, len(received),
                            advertised if advertised is not None else '?'))
            break

        m = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
        if m is None:
            continue

        if advertised is None:
            advertised = m.param_count
        idx = m.param_index
        name = param_id_str(m.param_id)
        if idx not in received:
            last_new = now
        received[idx] = (name, param_float_to_int32(m.param_value))
        by_name[name] = param_float_to_int32(m.param_value)

        if advertised is not None and len(received) >= advertised:
            break

    duration = time.monotonic() - start

    if advertised is None:
        report.fail('param_download', 'no PARAM_VALUE received at all')
        return by_name

    # Retry any missing indices once via PARAM_REQUEST_READ before failing.
    if len(received) < advertised:
        missing = [i for i in range(advertised) if i not in received]
        report.info('retrying {} missing param indices once'.format(len(missing)))
        for idx in missing:
            mav.mav.param_request_read_send(
                mav.target_system, mav.target_component, b'', idx)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                m = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                if m is None:
                    continue
                r_idx = m.param_index
                name = param_id_str(m.param_id)
                received[r_idx] = (name, param_float_to_int32(m.param_value))
                by_name[name] = param_float_to_int32(m.param_value)
                if r_idx == idx:
                    break

    report.check('param_download',
                 len(received) >= advertised,
                 'received {}/{} params in {:.1f}s'.format(
                     len(received), advertised, duration))
    return by_name


def phase_set_readback(report, mav, param, iterations):
    """Loop set/readback with a value that changes every iteration.

    value_i = ((i * 37) % 1999) - 999  (stays within -1000..1000).
    Individual failures are counted but the loop continues.
    """
    report.info('Phase 2: set/readback loop, {} iterations'.format(iterations))
    read_failures = 0
    mismatch_failures = 0

    for i in range(iterations):
        value = ((i * 37) % 1999) - 999

        drain_param_values(mav)
        set_param_int32(mav, param, value)
        matched, seen = wait_param_echo(mav, param, value, SET_ECHO_TIMEOUT_S)

        # read_param is the source of truth: a set propagates asynchronously,
        # so the echo can be a stale queued PARAM_VALUE from a prior set that
        # slipped past the drain. read_until confirms the board actually holds
        # the new value; an echo that disagrees with a confirmed read-back is
        # not counted as a failure.
        confirmed, readback = read_until(mav, param, value, COMMIT_TIMEOUT_S)
        if not confirmed:
            if readback is None:
                report.fail('param_readback',
                            'set iteration {}: no PARAM_VALUE on readback'.format(i))
                read_failures += 1
            else:
                report.fail('param_readback',
                            'set iteration {}: readback {} != set {}'.format(
                                i, readback, value))
                mismatch_failures += 1
        elif not matched:
            # board holds the right value but the echo never showed it; this is
            # the stale-echo case, informational not a failure
            report.info('  iteration {}: value confirmed by read-back; echo saw '
                        '{} (stale queued echo tolerated)'.format(i, seen or 'nothing'))

        if (i + 1) % 10 == 0:
            report.info('  {}/{} iterations done'.format(i + 1, iterations))

    report.check('param_set_readback_loop',
                 read_failures == 0 and mismatch_failures == 0,
                 '{} iterations: {} read, {} mismatch failures (readback is the '
                 'source of truth; stale echoes tolerated)'.format(
                     iterations, read_failures, mismatch_failures))


def phase_persistence(report, mav, param, conn_str, baud, original_value):
    """Write a marker, save, reboot, verify it survived, then restore.

    Returns the (possibly new) mavutil connection so the caller can keep the
    restore-in-finally path alive.
    """
    report.info('Phase 3: persistence across reboot')

    # 1. Set the marker and confirm it is actually committed to RAM by reading
    #    it back, not merely by matching the echo. A PARAM_SET propagates
    #    asynchronously; if we saved on the strength of the echo alone, save
    #    could race ahead of the commit and persist the PRIOR value (the last
    #    value Phase 2 wrote). read_until closes that gap, and it trusts the
    #    board's read over any stale queued echo.
    drain_param_values(mav)
    set_param_int32(mav, param, MARKER_VALUE)
    committed, seen = read_until(mav, param, MARKER_VALUE, COMMIT_TIMEOUT_S)
    if not committed:
        report.fail('persistence_set',
                    'marker {} not committed to RAM within {:.0f}s (read back: '
                    '{})'.format(MARKER_VALUE, COMMIT_TIMEOUT_S,
                                 seen if seen is not None else 'nothing'))
        return mav

    # 2. Save, then confirm the SAVED state before rebooting: param show must
    #    report the '+' (saved) flag AND the marker value. A '*' (unsaved) or a
    #    wrong value means save did not persist the marker; retry once, then
    #    fail without rebooting on an unsaved marker.
    shell = px4bench.MavlinkShell(mav)
    if not shell.open():
        report.fail('persistence_shell', 'could not open nsh shell for param save')
        return mav
    try:
        saved_ok = False
        saved, shown = None, None
        for attempt in range(2):
            out, timed_out = shell.run('param save', timeout=10)
            if timed_out:
                report.fail('persistence_save',
                            "'param save' did not complete within 10s (stalled)")
                return mav
            report.info("'param save' output: {}".format(out.strip() or '(none)'))
            saved, shown, show_timed_out = param_is_saved(shell, param)
            if show_timed_out:
                report.fail('persistence_save',
                            "'param show {}' stalled confirming the saved "
                            'state'.format(param))
                return mav
            if saved is True and shown == MARKER_VALUE:
                saved_ok = True
                break
            report.info('param show reports saved={} value={} after save '
                        'attempt {}'.format(saved, shown, attempt + 1))
        if not saved_ok:
            report.fail('persistence_save',
                        'marker {} not confirmed saved (param show saved={} '
                        'value={}); not rebooting on an unsaved marker'.format(
                            MARKER_VALUE, saved, shown))
            return mav
    finally:
        shell.close()

    # 3. Reboot and read the marker back.
    try:
        newmav, elapsed = px4bench.reboot_and_reconnect(mav, conn_str, baud, timeout=60)
    except TimeoutError as e:
        report.fail('persistence_reboot', str(e))
        return mav
    report.info('rebooted and reconnected in {:.1f}s'.format(elapsed))
    mav = newmav

    survived, _ = read_param(mav, param, READ_TIMEOUT_S)
    if survived is None:
        report.fail('persistence_readback',
                    'no PARAM_VALUE for {} after reboot (stalled)'.format(param))
    else:
        report.check('persistence',
                     survived == MARKER_VALUE,
                     'after reboot {} = {} (expected {})'.format(
                         param, survived, MARKER_VALUE))

    # Restore original inside this phase too, then verify by read-back.
    drain_param_values(mav)
    set_param_int32(mav, param, original_value)
    restored, seen = read_until(mav, param, original_value, COMMIT_TIMEOUT_S)
    report.check('persistence_restore',
                 restored,
                 'restored {} to {} (read back: {})'.format(
                     param, original_value, seen if seen is not None else 'nothing'))
    return mav


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    px4bench.add_connection_args(parser)
    parser.add_argument('--iterations', type=int, default=DEFAULT_ITERATIONS,
                        help='set/readback iterations (default: %(default)s)')
    parser.add_argument('--param', default=DEFAULT_PARAM,
                        help='scratch parameter to thrash (default: %(default)s)')
    parser.add_argument('--skip-reboot', action='store_true',
                        help='skip the reboot persistence phase')
    args = parser.parse_args()

    report = px4bench.Reporter('param_stress')

    try:
        mav = px4bench.connect(args.connection, baud=args.baudrate,
                               timeout=args.connect_timeout)
    except (TimeoutError, OSError) as e:
        report.fail('connect', str(e))
        sys.exit(report.finish())

    report.info('connected to system {} component {}'.format(
        mav.target_system, mav.target_component))

    original_value = None

    try:
        by_name = phase_full_download(report, mav)

        # Guard: a typo must not silently thrash the wrong (or no) param.
        if args.param not in by_name:
            report.fail('param_exists',
                        '{} not found in downloaded set; aborting before any set'.format(
                            args.param))
            sys.exit(report.finish())

        original_value, _ = read_param(mav, args.param, READ_TIMEOUT_S)
        if original_value is None:
            report.fail('param_original',
                        'could not read original value of {}'.format(args.param))
            sys.exit(report.finish())
        report.info('original {} = {}'.format(args.param, original_value))

        phase_set_readback(report, mav, args.param, args.iterations)

        if args.skip_reboot:
            report.info('Phase 3 skipped (--skip-reboot)')
        else:
            mav = phase_persistence(report, mav, args.param, args.connection,
                                    args.baudrate, original_value)

    finally:
        # Always restore the original value while a connection is alive.
        if original_value is not None and mav is not None:
            try:
                drain_param_values(mav)
                set_param_int32(mav, args.param, original_value)
                restored, seen = read_until(mav, args.param, original_value,
                                            COMMIT_TIMEOUT_S)
                report.check('final_restore',
                             restored,
                             'restored {} to {} (read back: {})'.format(
                                 args.param, original_value,
                                 seen if seen is not None else 'nothing'))
            except Exception as e:
                report.fail('final_restore',
                            'exception while restoring {}: {}'.format(args.param, e))
        try:
            if mav is not None:
                mav.close()
        except Exception:
            pass

    sys.exit(report.finish())


if __name__ == '__main__':
    main()
