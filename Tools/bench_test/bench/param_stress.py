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
                             param_id_str, read_param, set_param_int32,
                             wait_param_echo)

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
    echo_failures = 0
    read_failures = 0
    mismatch_failures = 0

    for i in range(iterations):
        value = ((i * 37) % 1999) - 999

        drain_param_values(mav)
        set_param_int32(mav, param, value)
        matched, seen = wait_param_echo(mav, param, value, SET_ECHO_TIMEOUT_S)
        if not matched:
            report.fail('param_set_echo',
                        'set iteration {}: no echo of {} within {}s (saw: {})'.format(
                            i, value, SET_ECHO_TIMEOUT_S, seen or 'nothing'))
            echo_failures += 1
            # keep going: still try to read back

        readback, _ = read_param(mav, param, READ_TIMEOUT_S)
        if readback is None:
            report.fail('param_readback',
                        'set iteration {}: no PARAM_VALUE on readback'.format(i))
            read_failures += 1
            continue
        if readback != value:
            report.fail('param_readback',
                        'set iteration {}: readback {} != set {}'.format(
                            i, readback, value))
            mismatch_failures += 1

        if (i + 1) % 10 == 0:
            report.info('  {}/{} iterations done'.format(i + 1, iterations))

    report.check('param_set_readback_loop',
                 echo_failures == 0 and read_failures == 0 and mismatch_failures == 0,
                 '{} iterations: {} echo, {} read, {} mismatch failures'.format(
                     iterations, echo_failures, read_failures, mismatch_failures))


def phase_persistence(report, mav, param, conn_str, baud, original_value):
    """Write a marker, save, reboot, verify it survived, then restore.

    Returns the (possibly new) mavutil connection so the caller can keep the
    restore-in-finally path alive.
    """
    report.info('Phase 3: persistence across reboot')

    drain_param_values(mav)
    set_param_int32(mav, param, MARKER_VALUE)
    matched, seen = wait_param_echo(mav, param, MARKER_VALUE, SET_ECHO_TIMEOUT_S)
    if not matched:
        report.fail('persistence_set',
                    'no echo of marker {} (saw: {})'.format(MARKER_VALUE, seen or 'nothing'))
        return mav

    # Force an explicit flush to storage before we pull the power.
    shell = px4bench.MavlinkShell(mav)
    if not shell.open(timeout=5):
        report.fail('persistence_shell', 'could not open nsh shell for param save')
        return mav
    try:
        out, timed_out = shell.run('param save', timeout=10)
    finally:
        shell.close()
    if timed_out:
        report.fail('persistence_save',
                    "'param save' did not complete within 10s (stalled)")
        return mav
    report.info("'param save' output: {}".format(out.strip() or '(none)'))

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

    # Restore original inside this phase too, then verify.
    drain_param_values(mav)
    set_param_int32(mav, param, original_value)
    matched, seen = wait_param_echo(mav, param, original_value, SET_ECHO_TIMEOUT_S)
    report.check('persistence_restore',
                 matched,
                 'restored {} to {} (saw: {})'.format(param, original_value, seen))
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
                matched, seen = wait_param_echo(mav, args.param, original_value,
                                                SET_ECHO_TIMEOUT_S)
                report.check('final_restore',
                             matched,
                             'restored {} to {} (saw: {})'.format(
                                 args.param, original_value, seen))
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
