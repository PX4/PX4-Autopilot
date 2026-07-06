# PX4 bench-test suite

PX4 v1.18 merged a large TSAN/concurrency series (PR #27606 plus follow-ups
#27809 and #27813) that reworked mavlink locking, uORB callbacks, WorkQueue
lifetime, parameters, and dataman. CI builds NuttX firmware but never boots it.
The characteristic failure mode of this class of bug on NuttX is a silent hang
(for example a mutex that was previously zero-initialized and locked from the
wrong context), not a crash. This suite makes hangs visible on real
Pixhawk-class hardware on a bench: every operation has a timeout, and a timeout
is reported as FAIL naming exactly what stalled.

The suite never arms the vehicle. Logging is triggered with `logger on` /
`logger off`. A hang is the finding, not a nuisance: when a check times out the
tool prints which operation stalled and exits nonzero rather than blocking.

## Requirements

- Python 3.
- Required: `pymavlink`, `pyserial`.
- Optional: `pyulog` (better .ulg verification; magic-byte check is used as
  fallback when absent).

```
pip3 install --user pymavlink pyserial
pip3 install --user pyulog   # optional
```

Finding the board on macOS:

```
ls /dev/tty.usbmodem*
```

Close QGroundControl before running anything. QGC holds the serial port and the
tools will not be able to open it.

Every script prints `[PASS]` / `[FAIL]` per check and exits nonzero on any
failure.

## Quick start

Full suite over a single USB link:

```
./run_bench_suite.py /dev/tty.usbmodem*
```

Full suite with a second link (telemetry radio); enables the dual-link tests:

```
./run_bench_suite.py /dev/tty.usbmodem* /dev/tty.usbserial-RADIO
```

The operator-assisted USB re-enumeration test is not part of the automated
suite. Run it manually:

```
./usb_replug.py /dev/tty.usbmodem*
```

## Risk map

| Test | Subsystem exercised | What a failure looks like |
| --- | --- | --- |
| boot_health | WorkQueue lifetime + uORB callback rework | Task in ERROR state, a required work queue missing, or a topic not publishing |
| usb_replug | mavlink instance lifecycle across USB re-enumeration | Heartbeat does not return, mavlink instance count changes, or free RAM grows across cycles |
| dual_link_forwarding | reworked mavlink nested-send lock path (silently deadlockable on NuttX) | Silent traffic gap > 5s on either link, param download stalls, or a link is dead afterward |
| param_torture | parameter subsystem locking | A set/readback mismatch, or the param does not persist across reboot |
| mission_torture | dataman + new mission shared-state mutex | Upload/download mismatch, or an operation stalls |
| mavftp_log | logger + MAVFTP path | Log does not start/stop, download stalls, or the .ulg fails integrity check |
| reboot_loop | boot-time initialization ordering (locked zero-init mutex bugs bite at boot) | Board does not reconnect within the cycle timeout |

`CONNECTION` is a serial device (`/dev/tty.usbmodem*`) or `udp:IP:PORT` /
`tcp:IP:PORT`. Default baud is 57600.

## Tests

### boot_health

Purpose: capture a snapshot of the running system after the WorkQueue and uORB
callback rework and check it is healthy.

```
./boot_health.py CONNECTION [-b BAUD] [--report-dir DIR] [--require-wq LIST]
```

Captures `top once`, `work_queue status`, `perf`, `uorb top -1`,
`mavlink status`, `free`, and `ver all` over the MAVLink shell into a
timestamped report dir. PASS means no task is in ERROR state, the required work
queues are present (`wq:rate_ctrl`, `wq:hp_default`, `wq:lp_default`), and
topics are publishing. A FAIL names the failing check; if the shell itself
stalls, that stall is the finding. Attach the report dir when reporting.

Baseline diff of two report dirs (uorb rates within tolerance, work queue sets
equal):

```
./boot_health.py --baseline OLDDIR NEWDIR [--tolerance PCT]
```

### usb_replug (operator-assisted)

Purpose: exercise mavlink instance lifecycle across USB re-enumeration.

```
./usb_replug.py CONNECTION [-b BAUD] [--cycles M] [--heartbeat-timeout S] [--ram-tolerance BYTES]
```

Guides you through unplug/replug cycles (default 5). PASS means the heartbeat
returns after each replug, the mavlink instance count stays constant, and free
RAM does not grow. A FAIL names the cycle and the failing condition; a heartbeat
that never returns within `--heartbeat-timeout` is the finding.

### dual_link_forwarding (flagship, needs 2 links)

Purpose: exercise the reworked mavlink nested-send lock path that was silently
deadlockable on NuttX.

```
./dual_link_forwarding.py CONNECTION CONNECTION2 [-b BAUD] [--baudrate2 BAUD2] [--duration S]
```

Verifies heartbeats on both links, then runs 60s of sustained bidirectional
traffic (a silent gap > 5s on either link is FAIL), then a full param download
over the radio link WHILE a shell session runs over USB, then proves both links
are still alive afterward. PASS means every stage completed and both links
survived. A hang anywhere reports FAIL naming the stage instead of blocking.

### param_torture

Purpose: hammer the parameter subsystem locking.

```
./param_torture.py CONNECTION [-b BAUD] [--iterations N] [--param NAME] [--skip-reboot]
```

Full param download, then 50 set/readback iterations on `SDLOG_UTC_OFFSET` (a
harmless scratch param, restored afterward), then param save + reboot +
persistence verification. PASS means every readback matched, the value
persisted across reboot, and the original value was restored. A FAIL names the
iteration or the persistence step. Use `--skip-reboot` to avoid the reboot
stage.

### mission_torture

Purpose: exercise dataman and the new mission shared-state mutex.

```
./mission_torture.py CONNECTION [CONNECTION2] [-b BAUD] [--baudrate2 BAUD2] [--iterations N] [--items K]
```

220-waypoint mission upload/download/compare/clear, 10 iterations. With two
links it alternates uploads between the links. PASS means every download matched
the upload across all iterations. A FAIL names the iteration and the mismatch,
or the operation that stalled.

### mavftp_log

Purpose: exercise the logger and the MAVFTP path.

```
./mavftp_log.py CONNECTION [-b BAUD] [--log-duration S] [--outdir DIR]
```

Starts a short log via `logger on` / `logger off` (no arming), downloads the
newest .ulg via MAVFTP, and verifies ULog integrity (pyulog if installed, magic
bytes otherwise). PASS means the log was created, downloaded, and passed the
integrity check. A FAIL names the stage; a stalled MAVFTP transfer is the
finding.

### reboot_loop

Purpose: exercise boot-time initialization ordering, where locked zero-init
mutex bugs bite.

```
./reboot_loop.py CONNECTION [-b BAUD] [--iterations N] [--cycle-timeout S]
```

10 reboot/reconnect cycles via `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`. PASS means
the board came back within `--cycle-timeout` on every cycle. Any failure to
reconnect in time is FAIL with the elapsed time and the cycle number.

### run_bench_suite

Runs all non-interactive tests in sequence with a per-test watchdog.

```
./run_bench_suite.py CONNECTION [CONNECTION2] [-b BAUD] [--skip NAMES] [--report-dir DIR] [--per-test-timeout S] [--stop-on-fail]
```

Default per-test watchdog is 900s; a test that exceeds it is killed and recorded
as FAIL `<test> hung`. `usb_replug` is operator-assisted and must be run
manually. Dual-link tests run only when a second connection is given.

## Baseline workflow

Use this to compare pre-upgrade and post-upgrade firmware. Each capture
creates a timestamped directory under `--report-dir` and prints its path
(`report dir: ...`); the diff takes those printed paths, not the base
directory:

1. On known-good firmware, capture a report:
   `./boot_health.py CONNECTION` and note the printed report dir, for
   example `bench_reports/20260706T164512Z_boot_health`.
2. Flash the candidate firmware.
3. Capture again: `./boot_health.py CONNECTION` and note the new report dir.
4. Diff them:
   `./boot_health.py --baseline <old report dir> <new report dir> [--tolerance PCT]`

The diff checks that uorb rates are within tolerance and the work queue sets are
equal. A missing work queue or a rate outside tolerance is FAIL.

## Safety

- Bench use only. Props off.
- The suite never arms the vehicle.
- `param_torture` and `reboot_loop` reboot the board multiple times. Expect the
  link to drop and return.
- `param_torture` writes only to `SDLOG_UTC_OFFSET` and restores the original
  value automatically.

## Troubleshooting

No heartbeat:

- Wrong device. Confirm with `ls /dev/tty.usbmodem*` and pass the exact path.
- QGroundControl (or another GCS) is holding the port. Close it.
- Wrong baud on a radio link. Match the radio's configured baud with `-b` /
  `--baudrate2`.

Shell stalls: if `boot_health` or a shell-based stage hangs, that stall is the
finding, not a tooling bug. Note which command it stalled on and keep the report
dir.

Telemetry radio bandwidth: dual-link traffic, param download, and mission
upload/download over a radio link are much slower than over USB. Allow longer
durations (`--duration`, `--per-test-timeout`) when a radio is in the path.
