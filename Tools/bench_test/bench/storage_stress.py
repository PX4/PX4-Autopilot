#!/usr/bin/env python3
"""
SD card storage health test using the firmware's own benchmark commands.

Storage misbehavior on a flight controller is a flight-safety issue: the
logger drops data when writes stall, and dataman lives on the same card.
The firmware already ships the right tools; this test drives them over the
MAVLink shell and turns their output into named checks:

- sd_bench (src/systemcmds/sd_bench): sequential write/read throughput,
  max write latency, and fsync latency, run with data verification (-v).
- sd_stress (src/systemcmds/sd_stress): file create/rename/delete churn,
  100 files per iteration.

Probe-and-skip: firmware without the commands (command not found) or a
board without a mounted SD card skips with a warning instead of failing.
The two phases degrade independently: sd_bench present but sd_stress
absent runs the bench phase and skips the stress phase.

Thresholds are deliberately generous FAIL floors (a slow but working card
should warn, not flap); see the README for the reasoning behind the
defaults.
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import px4bench

NO_SD_MARKERS = ("Can't open benchmark file",)

# WARN (informational) when a metric is below WARN_FACTOR * its FAIL floor,
# even though the check itself passes.
WARN_FACTOR = 4.0


def parse_sd_bench(output):
    """Parse sd_bench output (src/systemcmds/sd_bench/sd_bench.cpp).

    Sections: 'Testing Sequential Write Speed...' then per-run lines
      '  Run  N: X KB/s, max write time: N ms (=Y KB/s), fsync: N ms'
    and '  Avg   : X KB/s'; then 'Testing Sequential Read Speed of N blocks'
    with per-run lines and a second Avg line.

    Returns dict with write_avg_kbs, read_avg_kbs, max_write_ms,
    max_fsync_ms (any may be None) and data_errors (list of lines).
    """
    res = {'write_avg_kbs': None, 'read_avg_kbs': None,
           'max_write_ms': None, 'max_fsync_ms': None, 'data_errors': []}
    section = None
    write_run_re = re.compile(
        r'Run\s+\d+:\s+([\d.]+)\s+KB/s, max write time: (\d+) ms.*fsync: (\d+) ms')
    avg_re = re.compile(r'Avg\s*:\s*([\d.]+)\s+KB/s')

    for line in output.splitlines():
        if 'Testing Sequential Write Speed' in line:
            section = 'write'
            continue
        if 'Testing Sequential Read Speed' in line:
            section = 'read'
            continue
        if ('Read data error' in line or 'Write error' in line
                or 'Read error' in line):
            res['data_errors'].append(line.strip())
            continue
        m = write_run_re.search(line)
        if m and section == 'write':
            wt, fs = int(m.group(2)), int(m.group(3))
            if res['max_write_ms'] is None or wt > res['max_write_ms']:
                res['max_write_ms'] = wt
            if res['max_fsync_ms'] is None or fs > res['max_fsync_ms']:
                res['max_fsync_ms'] = fs
            continue
        m = avg_re.search(line)
        if m:
            if section == 'write' and res['write_avg_kbs'] is None:
                res['write_avg_kbs'] = float(m.group(1))
            elif section == 'read' and res['read_avg_kbs'] is None:
                res['read_avg_kbs'] = float(m.group(1))
    return res


def parse_sd_stress(output):
    """Parse sd_stress output: 'iteration N took NNNNNN us: OK|FAIL' lines."""
    ok = fail = 0
    worst_us = 0
    for m in re.finditer(r'iteration \d+ took\s*(\d+) us: (OK|FAIL)', output):
        if m.group(2) == 'OK':
            ok += 1
        else:
            fail += 1
        worst_us = max(worst_us, int(m.group(1)))
    return ok, fail, worst_us


def check_metric(report, name, value, floor, unit, higher_is_better=True):
    """FAIL below/above the generous floor; WARN when merely slow."""
    if value is None:
        report.fail(name, 'could not parse {} from output'.format(name))
        return
    if higher_is_better:
        ok = value >= floor
        slow = value < floor * WARN_FACTOR
    else:
        ok = value <= floor
        slow = value > floor / WARN_FACTOR
    detail = '{:.1f} {} (floor: {} {})'.format(value, unit, floor, unit)
    report.check(name, ok, detail)
    if ok and slow:
        report.info('WARNING: {} is {}; working but slow for this class of '
                    'card'.format(name, detail))


def main():
    parser = argparse.ArgumentParser(
        description='SD storage health via sd_bench and sd_stress over the MAVLink shell.')
    px4bench.add_connection_args(parser)
    parser.add_argument('--report-dir', default='bench_reports',
                        help='base directory for report output (default: %(default)s)')
    parser.add_argument('--bench-runs', type=int, default=3,
                        help='sd_bench runs (default: %(default)s)')
    parser.add_argument('--bench-duration-ms', type=int, default=2000,
                        help='sd_bench per-run duration in ms (default: %(default)s)')
    parser.add_argument('--bench-block', type=int, default=4096,
                        help='sd_bench block size in bytes (default: %(default)s)')
    parser.add_argument('--min-write-kbs', type=float, default=100,
                        help='FAIL floor for avg sequential write (default: %(default)s KB/s)')
    parser.add_argument('--min-read-kbs', type=float, default=200,
                        help='FAIL floor for avg sequential read (default: %(default)s KB/s)')
    parser.add_argument('--max-fsync-ms', type=float, default=500,
                        help='FAIL ceiling for worst fsync latency (default: %(default)s ms)')
    parser.add_argument('--stress-runs', type=int, default=2,
                        help='sd_stress iterations, 100 files each (default: %(default)s)')
    parser.add_argument('--stress-bytes', type=int, default=100,
                        help='sd_stress bytes per file (default: %(default)s)')
    args = parser.parse_args()

    report = px4bench.Reporter('storage_stress')
    report_dir = px4bench.make_report_dir(args.report_dir, 'storage_stress')
    report.info('report dir: {}'.format(report_dir))

    try:
        mav = px4bench.connect(args.connection, baud=args.baudrate,
                               timeout=args.connect_timeout)
    except (TimeoutError, OSError) as e:
        report.fail('connect', str(e))
        sys.exit(report.finish())
    report.ok('connect', 'heartbeat from {}'.format(args.connection))

    shell = px4bench.MavlinkShell(mav)
    if not shell.open(timeout=5):
        report.fail('shell open', 'no nsh output on SERIAL_CONTROL within 5s')
        mav.close()
        sys.exit(report.finish())

    def save(name, output):
        try:
            with open(os.path.join(report_dir, name + '.txt'), 'w') as f:
                f.write(output)
        except OSError:
            pass

    # probe both commands; '-x' is an unknown flag, so an existing command
    # replies with its usage text while a missing one replies not-found
    bench_present, _ = px4bench.shell_command_exists(shell, 'sd_bench -x')
    stress_present, _ = px4bench.shell_command_exists(shell, 'sd_stress -x')
    ran_any = False
    no_sd = False

    if bench_present is None or stress_present is None:
        report.fail('probe', 'command probe stalled over the shell')
        shell.close()
        mav.close()
        sys.exit(report.finish())

    if bench_present:
        cmd = 'sd_bench -r {} -d {} -b {} -v'.format(
            args.bench_runs, args.bench_duration_ms, args.bench_block)
        timeout = args.bench_runs * args.bench_duration_ms * 2 / 1000.0 + 60
        report.info('running {} (timeout {:.0f}s)'.format(cmd, timeout))
        out, timed_out = shell.run(cmd, timeout=timeout)
        save('sd_bench', out)
        if timed_out:
            report.fail('sd_bench', 'sd_bench stalled (no completion within '
                                    '{:.0f}s), output saved'.format(timeout))
        elif any(marker in out for marker in NO_SD_MARKERS):
            no_sd = True
            report.info('WARNING: no SD card mounted (sd_bench could not open '
                        'its benchmark file); skipping storage checks')
        else:
            ran_any = True
            res = parse_sd_bench(out)
            check_metric(report, 'sd_bench write', res['write_avg_kbs'],
                         args.min_write_kbs, 'KB/s')
            check_metric(report, 'sd_bench read', res['read_avg_kbs'],
                         args.min_read_kbs, 'KB/s')
            check_metric(report, 'sd_bench fsync', res['max_fsync_ms'],
                         args.max_fsync_ms, 'ms', higher_is_better=False)
            if res['max_write_ms'] is not None:
                report.info('worst single write: {} ms'.format(res['max_write_ms']))
            report.check('sd_bench data verify', not res['data_errors'],
                         '; '.join(res['data_errors'][:3]) if res['data_errors']
                         else 'no read/write/verify errors (-v)')
    else:
        report.info('WARNING: sd_bench not in this firmware (command not '
                    'found); bench phase skipped. Enable '
                    'CONFIG_SYSTEMCMDS_SD_BENCH=y')

    if no_sd:
        pass  # no point stressing a card that is not there
    elif stress_present:
        cmd = 'sd_stress -r {} -b {}'.format(args.stress_runs, args.stress_bytes)
        timeout = args.stress_runs * 45 + 30
        report.info('running {} (timeout {:.0f}s)'.format(cmd, timeout))
        out, timed_out = shell.run(cmd, timeout=timeout)
        save('sd_stress', out)
        if timed_out:
            report.fail('sd_stress', 'sd_stress stalled (no completion within '
                                     '{:.0f}s), output saved'.format(timeout))
        else:
            ok, fail, worst_us = parse_sd_stress(out)
            if ok == 0 and fail == 0:
                if 'failed' in out:
                    no_sd = True
                    report.info('WARNING: sd_stress could not use the SD card; '
                                'skipping (output saved)')
                else:
                    report.fail('sd_stress', 'no iteration results in output')
            else:
                ran_any = True
                report.check('sd_stress', fail == 0,
                             '{} OK, {} FAIL iterations, worst {:.0f} ms '
                             '(100 files each)'.format(ok, fail, worst_us / 1000.0))
    else:
        report.info('WARNING: sd_stress not in this firmware (command not '
                    'found); stress phase skipped. Enable '
                    'CONFIG_SYSTEMCMDS_SD_STRESS=y')

    shell.close()
    mav.close()

    if not ran_any and report.num_failed == 0:
        print('storage_stress: skipped (no usable storage commands or no SD '
              'card); this is a warning, not a failure.', flush=True)
        sys.exit(px4bench.EXIT_SKIP)
    sys.exit(report.finish())


if __name__ == '__main__':
    main()
