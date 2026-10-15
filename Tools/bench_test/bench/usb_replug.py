#!/usr/bin/env python3
"""
Guided USB replug test for a PX4 board on real NuttX hardware.

USB re-enumeration is a stress test for the MAVLink instance lifecycle: every
time the cable is replugged the CDC ACM link tears down and comes back, and a
new MAVLink instance is (re)started for it. Before the v1.18 concurrency
rework this path shared state across the teardown; the failure we hunt is a
board that leaks a MAVLink instance per replug (instance count climbs) or
leaks RAM per replug (free used bytes climbs cycle over cycle).

This test is operator-assisted but keyboard-free during the loop: it prints
"UNPLUG now" / "REPLUG now" and detects the device node vanishing and
returning programmatically (the node name can change across enumeration on
macOS, so we follow the returned path). After each replug it reconnects,
re-reads the MAVLink instance count and free RAM, and compares against the
baseline.

Every wait has a timeout. A device that never comes back, or a shell command
that never completes, is reported as a FAIL naming what stalled, not a hung
script.
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import px4bench


def parse_free_used(output):
    """Parse NuttX `free` output, return (total_bytes, used_bytes) or (None, None).

    NuttX `free` prints a header line then one or more memory-region lines:

                     total       used       free    largest ...
        Umem:       262144      53016     209128     198600 ...

    The exact region label varies by build (Umem/Kmem/Mem), and column count
    varies, so we parse tolerantly: find the header line (contains both
    "total" and "used"), remember which column index each is, then take the
    first following line that has enough integer fields and read those two
    columns. If there is no recognizable header we fall back to the first data
    line and assume the classic order total, used as the first two integers.
    """
    lines = output.splitlines()

    for i, line in enumerate(lines):
        low = line.lower()
        if 'total' in low and 'used' in low:
            # header found; take the first following line with numeric columns.
            # A data line may carry a leading region label (e.g. "Umem:"),
            # so strip a non-numeric first field and read the classic NuttX
            # column order: total, used as the first two integers.
            for data in lines[i + 1:]:
                fields = data.split()
                if len(fields) < 3:
                    continue
                if fields and not re.match(r'^-?\d', fields[0]):
                    fields = fields[1:]
                ints = [int(t) for t in fields if re.match(r'^\d+$', t)]
                if len(ints) >= 2:
                    return ints[0], ints[1]
            break

    # no header found: first line with >=2 integers, classic order
    for line in lines:
        fields = line.split()
        if fields and not re.match(r'^-?\d', fields[0]):
            fields = fields[1:]
        ints = [int(t) for t in fields if re.match(r'^\d+$', t)]
        if len(ints) >= 2:
            return ints[0], ints[1]

    return None, None


def read_baseline(shell, report):
    """Read instance count and free used bytes at baseline. Returns dict or None."""
    ms_out, timed_out = shell.run('mavlink status', timeout=10)
    if timed_out:
        report.fail('baseline mavlink status',
                    'shell command mavlink status stalled (no completion within 10s)')
        return None
    instances = px4bench.count_mavlink_instances(ms_out)

    free_out, timed_out = shell.run('free', timeout=10)
    if timed_out:
        report.fail('baseline free',
                    'shell command free stalled (no completion within 10s)')
        return None
    total, used = parse_free_used(free_out)
    if used is None:
        report.fail('baseline free parse', 'could not parse used bytes from free output')
        return None

    report.info('baseline: {} mavlink instance(s), free used={} bytes total={} bytes'.format(
        instances, used, total))
    return {'instances': instances, 'used': used, 'total': total,
            'free_raw': free_out, 'mavlink_raw': ms_out}


def main():
    parser = argparse.ArgumentParser(
        description='PX4 guided USB replug test: watch MAVLink instance and RAM '
                    'lifecycle across cable re-enumeration.')
    px4bench.add_connection_args(parser)
    parser.add_argument('--cycles', type=int, default=5,
                        help='number of unplug/replug cycles (default: %(default)s)')
    parser.add_argument('--heartbeat-timeout', type=float, default=30,
                        help='seconds to wait for heartbeat after replug (default: %(default)s)')
    parser.add_argument('--ram-tolerance', type=int, default=8192,
                        help='allowed free-used growth in bytes per cycle before FAIL '
                             '(default: %(default)s)')
    args = parser.parse_args()

    report = px4bench.Reporter('usb_replug')

    if not px4bench.is_serial_device(args.connection):
        print('error: usb_replug requires a serial device connection '
              '(got {!r}); USB re-enumeration cannot be observed on a network link.'.format(
                  args.connection), file=sys.stderr)
        sys.exit(2)

    device = args.connection

    try:
        mav = px4bench.connect(device, baud=args.baudrate, timeout=args.connect_timeout)
    except (TimeoutError, OSError) as e:
        report.fail('baseline connect', str(e))
        sys.exit(report.finish())
    report.ok('baseline connect', 'heartbeat from {}'.format(device))

    shell = px4bench.MavlinkShell(mav)
    if not shell.open():
        report.fail('baseline shell', 'no nsh output on SERIAL_CONTROL within {:.0f}s'.format(
            px4bench.SHELL_OPEN_TIMEOUT))
        shell.close()
        mav.close()
        sys.exit(report.finish())
    try:
        baseline = read_baseline(shell, report)
    finally:
        shell.close()
        mav.close()
    if baseline is None:
        sys.exit(report.finish())

    report.ok('baseline', '{} instance(s), {} bytes used'.format(
        baseline['instances'], baseline['used']))

    last_used = baseline['used']

    for i in range(1, args.cycles + 1):
        print('-' * 60, flush=True)
        print('Cycle {}/{}: UNPLUG the USB cable now.'.format(i, args.cycles), flush=True)

        gone = px4bench.wait_device_gone(device, timeout=60)
        if gone is None:
            # give the operator one guided retry before failing the cycle
            print('Device {} still present after 60s. Please UNPLUG it now '
                  '(retrying for 30s).'.format(device), flush=True)
            gone = px4bench.wait_device_gone(device, timeout=30)
        if gone is None:
            report.fail('cycle {} unplug'.format(i),
                        'device {} never disappeared (unplug not detected within 90s)'.format(
                            device))
            continue
        report.info('cycle {}: device gone after {:.1f}s'.format(i, gone))

        print('Cycle {}/{}: REPLUG the USB cable now.'.format(i, args.cycles), flush=True)
        newdev, back_elapsed = px4bench.wait_device_back(device, timeout=60)
        if newdev is None:
            report.fail('cycle {} replug'.format(i),
                        'device did not reappear within 60s after unplug')
            continue
        if newdev != device:
            report.info('cycle {}: device re-enumerated as {} (was {})'.format(
                i, newdev, device))
        device = newdev  # follow the possibly-renamed node
        report.info('cycle {}: device back after {:.1f}s'.format(i, back_elapsed))

        # reconnect + heartbeat within the configured budget
        try:
            mav = px4bench.connect(device, baud=args.baudrate,
                                   timeout=args.heartbeat_timeout)
        except (TimeoutError, OSError) as e:
            report.fail('cycle {} reconnect'.format(i),
                        'no heartbeat within {}s after replug: {}'.format(
                            args.heartbeat_timeout, e))
            continue
        report.ok('cycle {} heartbeat'.format(i),
                  'reconnected on {}'.format(device))

        shell = px4bench.MavlinkShell(mav)
        if not shell.open():
            report.fail('cycle {} shell'.format(i),
                        'no nsh output on SERIAL_CONTROL within {:.0f}s'.format(
                            px4bench.SHELL_OPEN_TIMEOUT))
            shell.close()
            mav.close()
            continue

        # One shell session per cycle, torn down in finally so the leak this
        # test hunts is not one the test itself introduces.
        try:
            # instance count must not grow (a leak piles up instances on replug)
            ms_out, ms_timed_out = shell.run('mavlink status', timeout=10)
            free_out, free_timed_out = (None, False)
            if not ms_timed_out:
                free_out, free_timed_out = shell.run('free', timeout=10)
        finally:
            shell.close()
            mav.close()

        if ms_timed_out:
            report.fail('cycle {}'.format(i),
                        'shell command mavlink status stalled (no completion within 10s)')
            continue
        instances = px4bench.count_mavlink_instances(ms_out)
        report.check('cycle {} instances'.format(i),
                     instances == baseline['instances'],
                     'instances={} (baseline={})'.format(instances, baseline['instances']))

        # free used must not exceed baseline + tolerance * cycle_index (steady growth = leak)
        if free_timed_out:
            report.fail('cycle {}'.format(i),
                        'shell command free stalled (no completion within 10s)')
            continue
        _, used = parse_free_used(free_out)
        if used is None:
            report.fail('cycle {} free parse'.format(i),
                        'could not parse used bytes from free output')
            continue
        budget = baseline['used'] + args.ram_tolerance * i
        report.check('cycle {} ram'.format(i), used <= budget,
                     'used={} bytes, budget={} bytes (baseline {} + {}*{}), '
                     'delta-since-last={:+d}'.format(
                         used, budget, baseline['used'], args.ram_tolerance, i,
                         used - last_used))
        last_used = used

    print('-' * 60, flush=True)
    total_growth = last_used - baseline['used']
    report.info('free used: first={} bytes, last={} bytes, net growth={:+d} bytes '
                'over {} cycle(s)'.format(
                    baseline['used'], last_used, total_growth, args.cycles))

    sys.exit(report.finish())


if __name__ == '__main__':
    main()
