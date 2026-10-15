#!/usr/bin/env python3
"""
Boot-health snapshot for a PX4 board on real NuttX hardware.

After the v1.18 concurrency and locking rework we want a fast, repeatable
way to confirm a freshly booted board is healthy: no task stuck in an ERROR
state, the expected work queues running, uORB actually publishing, and the
MAVLink instances up. This script connects over MAVLink, opens an nsh shell
over SERIAL_CONTROL, runs a fixed set of status commands, saves each raw
output for later comparison, and reports PASS/FAIL per check.

It also has an offline baseline mode: point it at an old report dir and a new
one and it flags uORB topics whose publication rate drifted beyond a
tolerance, topics that disappeared, and work queues that went missing. That
turns "does it still boot" into "does it still boot the same way it used to".

Every wait has a timeout. A command that never completes is reported as a FAIL
naming the command, never a hung script, because a silent hang is exactly the
failure mode we are hunting.
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import px4bench


# Commands to run in capture mode: (command, per-command timeout seconds).
# Order matters: cheap/liveness checks first, heavier dumps later.
CAPTURE_COMMANDS = [
    ('top once', 15),
    ('work_queue status', 10),
    ('perf', 25),
    ('uorb top -1', 15),
    ('mavlink status', 10),
    ('free', 10),
    ('ver all', 10),
]


def sanitize_cmd(cmd):
    """Turn a shell command into a safe filename stem."""
    stem = re.sub(r'[^A-Za-z0-9]+', '_', cmd).strip('_')
    return stem or 'cmd'


def save_output(report_dir, cmd, output):
    """Write a command's raw output to <report_dir>/<sanitized_cmd>.txt."""
    path = os.path.join(report_dir, sanitize_cmd(cmd) + '.txt')
    try:
        with open(path, 'w') as f:
            f.write(output)
    except OSError as e:
        print('[INFO] could not save {}: {}'.format(path, e), flush=True)
    return path


def top_has_error_task(output):
    """Return (has_error, list_of_offending_lines).

    The `top` STATE column is one of the tstate_name() strings; the default /
    unknown state prints as ERROR (print_load.cpp:132). STATE is a
    whitespace-padded token, so we match ERROR as a standalone word. We only
    look at lines that start with a PID (an integer) to avoid matching the
    header or any incidental use of the word.
    """
    offenders = []
    for line in output.splitlines():
        fields = line.split()
        if not fields:
            continue
        if not fields[0].isdigit():
            continue  # not a task row (header, footer, blank)
        if 'ERROR' in fields[1:]:
            offenders.append(line.strip())
    return (len(offenders) > 0, offenders)


def parse_work_queues(output):
    """Return the set of work-queue names present in `work_queue status` output.

    WorkQueue::print_status prints one line per queue starting with the queue
    name including its prefix, e.g. `wq:rate_ctrl`, `wq:SPI1`, `wq:lp_default`
    (WorkQueue.cpp:232, names in WorkQueueManager.hpp). We extract every
    `wq:<name>` token found anywhere on a line so we tolerate leading counters
    or formatting.
    """
    names = set()
    for line in output.splitlines():
        for m in re.findall(r'wq:[A-Za-z0-9_]+', line):
            names.add(m)
    return names


def parse_uorb_top(output):
    """Parse `uorb top -1` output into {topic_key: rate}.

    Format (uORBDeviceMaster.cpp:387-396, showTop):
        header:  "update: 1s, topics: N, total publications: M, X.X kB/s"
        columns: "TOPIC NAME              INST #SUB RATE #Q SIZE"
        rows:    "<name> %2i %4i %4i %2i %4i"
                  = name  INST #SUB RATE #Q SIZE

    So a data row is: topic name (may contain no spaces; uORB topic names are
    single tokens), then five integers INST, #SUB, RATE, #Q, SIZE. RATE is the
    publication rate we compare across baselines. We key each entry by
    "name#inst" so multi-instance topics stay distinct, and we skip any line
    that does not match tolerantly (headers, escape codes, blanks).
    """
    rates = {}
    # a topic name token followed by exactly five integers at end of line
    row_re = re.compile(r'^([A-Za-z][A-Za-z0-9_]*)\s+'
                        r'(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s*$')
    for raw in output.splitlines():
        # strip any leftover ANSI escape sequences (clear-line/clear-screen)
        line = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', raw).strip()
        if not line:
            continue
        m = row_re.match(line)
        if not m:
            continue
        name = m.group(1)
        inst = int(m.group(2))
        rate = int(m.group(4))
        rates['{}#{}'.format(name, inst)] = rate
    return rates


def run_capture(args, report):
    report_dir = px4bench.make_report_dir(args.report_dir, 'boot_health')
    report.info('report dir: {}'.format(report_dir))

    require_wq = [q.strip() for q in args.require_wq.split(',') if q.strip()]

    try:
        mav = px4bench.connect(args.connection, baud=args.baudrate,
                               timeout=args.connect_timeout)
    except (TimeoutError, OSError) as e:
        report.fail('connect', str(e))
        return report_dir
    report.ok('connect', 'heartbeat from {}'.format(args.connection))

    shell = px4bench.MavlinkShell(mav)
    if not shell.open():
        report.fail('shell open', 'no nsh output on SERIAL_CONTROL within {:.0f}s'.format(
            px4bench.SHELL_OPEN_TIMEOUT))
        shell.close()
        mav.close()
        return report_dir
    report.ok('shell open', 'nsh responded')

    try:
        _capture(report, shell, report_dir, require_wq)
    finally:
        shell.close()
        mav.close()
    return report_dir


def _capture(report, shell, report_dir, require_wq):
    """Run the capture commands and evaluate them on an open shell."""
    outputs = {}
    for cmd, timeout in CAPTURE_COMMANDS:
        output, timed_out = shell.run(cmd, timeout=timeout)
        save_output(report_dir, cmd, output)
        outputs[cmd] = output
        if timed_out:
            report.fail('cmd ' + cmd,
                        'shell command {} stalled (no completion within {}s)'.format(
                            cmd, timeout))
        else:
            report.ok('cmd ' + cmd, '{} bytes'.format(len(output)))

    # top: no task in ERROR state
    if 'top once' in outputs and outputs['top once']:
        has_err, offenders = top_has_error_task(outputs['top once'])
        report.check('top no ERROR task', not has_err,
                     'offending rows: ' + ' | '.join(offenders) if has_err
                     else 'no task in ERROR state')

    # work queues: required set present
    if 'work_queue status' in outputs:
        present = parse_work_queues(outputs['work_queue status'])
        missing = [q for q in require_wq if q not in present]
        report.check('work queues present', not missing,
                     'missing: {}'.format(', '.join(missing)) if missing
                     else 'all required present ({} queues total)'.format(len(present)))

    # uorb: parsed at least one topic
    if 'uorb top -1' in outputs:
        rates = parse_uorb_top(outputs['uorb top -1'])
        report.check('uorb topics parsed', len(rates) > 0,
                     '{} topics'.format(len(rates)))

    # mavlink instances: informational count
    if 'mavlink status' in outputs:
        n = px4bench.count_mavlink_instances(outputs['mavlink status'])
        report.info('mavlink instances: {}'.format(n))


def read_saved(report_dir, cmd):
    """Read a previously saved command output from a report dir, or ''."""
    path = os.path.join(report_dir, sanitize_cmd(cmd) + '.txt')
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return ''


def run_baseline(old_dir, new_dir, tolerance, report):
    report.info('baseline: OLD={} NEW={} tolerance={}%'.format(old_dir, new_dir, tolerance))

    for d in (old_dir, new_dir):
        if not os.path.isdir(d):
            report.fail('baseline dir', 'not a directory: {}'.format(d))
            return

    old_uorb = read_saved(old_dir, 'uorb top -1')
    new_uorb = read_saved(new_dir, 'uorb top -1')
    if not old_uorb:
        report.fail('baseline uorb old', 'no saved uorb output in {}'.format(old_dir))
    if not new_uorb:
        report.fail('baseline uorb new', 'no saved uorb output in {}'.format(new_dir))

    if old_uorb and new_uorb:
        old_rates = parse_uorb_top(old_uorb)
        new_rates = parse_uorb_top(new_uorb)
        report.info('uorb topics: OLD={} NEW={}'.format(len(old_rates), len(new_rates)))

        # topics present in OLD but missing in NEW -> FAIL
        for topic in sorted(old_rates):
            if topic not in new_rates:
                report.fail('uorb missing ' + topic,
                            'present in baseline, absent now (was {} Hz)'.format(old_rates[topic]))

        # rate drift beyond tolerance -> FAIL
        for topic in sorted(old_rates):
            if topic not in new_rates:
                continue
            old_r = old_rates[topic]
            new_r = new_rates[topic]
            if old_r == 0:
                # baseline was idle; only flag if it is now clearly active
                if new_r > 0:
                    report.info('uorb {} went {}->{} Hz (baseline idle)'.format(
                        topic, old_r, new_r))
                continue
            drift = abs(new_r - old_r) * 100.0 / old_r
            if drift > tolerance:
                report.fail('uorb rate ' + topic,
                            '{} Hz -> {} Hz ({:.0f}% drift > {}%)'.format(
                                old_r, new_r, drift, tolerance))

        # new topics -> info only
        for topic in sorted(new_rates):
            if topic not in old_rates:
                report.info('uorb new topic {} ({} Hz)'.format(topic, new_rates[topic]))

    # work queue name sets
    old_wq_raw = read_saved(old_dir, 'work_queue status')
    new_wq_raw = read_saved(new_dir, 'work_queue status')
    if not old_wq_raw or not new_wq_raw:
        report.info('work_queue status missing from one dir, skipping queue diff')
    else:
        old_wq = parse_work_queues(old_wq_raw)
        new_wq = parse_work_queues(new_wq_raw)
        missing = sorted(old_wq - new_wq)
        report.check('work queues retained', not missing,
                     'missing now: {}'.format(', '.join(missing)) if missing
                     else '{} queues, none lost'.format(len(old_wq)))
        added = sorted(new_wq - old_wq)
        if added:
            report.info('new work queues: {}'.format(', '.join(added)))


def main():
    parser = argparse.ArgumentParser(
        description='PX4 boot-health snapshot and offline baseline diff.')
    # connection is optional so --baseline can run without hardware
    parser.add_argument('connection', nargs='?',
                        help='MAVLink connection: serial device (/dev/tty.usbmodem01), '
                             'udp:IP:PORT, or tcp:IP:PORT')
    parser.add_argument('--baudrate', '-b', type=int, default=px4bench.DEFAULT_BAUD,
                        help='serial baud rate (default: %(default)s)')
    parser.add_argument('--connect-timeout', type=float, default=20,
                        help='seconds to wait for the first heartbeat (default: %(default)s)')
    parser.add_argument('--report-dir', default='bench_reports',
                        help='base directory for report output (default: %(default)s)')
    parser.add_argument('--require-wq', default='wq:rate_ctrl,wq:hp_default,wq:lp_default',
                        help='comma-separated work queues that must be present '
                             '(default: %(default)s)')
    parser.add_argument('--baseline', nargs=2, metavar=('OLDDIR', 'NEWDIR'),
                        help='offline mode: diff two existing report dirs, no board needed')
    parser.add_argument('--tolerance', type=float, default=30,
                        help='baseline: max %% uORB rate drift before FAIL (default: %(default)s)')
    args = parser.parse_args()

    if args.baseline:
        report = px4bench.Reporter('boot_health_baseline')
        run_baseline(args.baseline[0], args.baseline[1], args.tolerance, report)
        sys.exit(report.finish())

    if not args.connection:
        parser.error('a CONNECTION is required unless --baseline OLDDIR NEWDIR is given')

    report = px4bench.Reporter('boot_health')
    report_dir = run_capture(args, report)
    code = report.finish()
    print('report dir: {}'.format(report_dir))
    print('baseline later with: {} --baseline {} <NEWDIR>'.format(
        os.path.basename(sys.argv[0]), report_dir))
    sys.exit(code)


if __name__ == '__main__':
    main()
