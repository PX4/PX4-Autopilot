# px4bench: PX4 bench-test suite

Semi-automated release qualification for PX4 on real NuttX hardware
(Pixhawk-class boards on a bench, connected over USB and optionally a
telemetry radio).

PX4 v1.18 merged a large TSAN/concurrency series (PR #27606 plus follow-ups
#27809 and #27813) that reworked mavlink locking, uORB callbacks, WorkQueue
lifetime, parameters, and dataman. CI builds NuttX firmware but never boots
it. The characteristic failure mode of this class of bug on NuttX is a
silent hang (for example a mutex that was previously zero-initialized and
locked from the wrong context), not a crash. This suite makes hangs visible:
every operation has a timeout, and a timeout is reported as FAIL naming
exactly what stalled. A hang is the finding, not a nuisance.

The bench tests never arm the vehicle; logging is triggered with
`logger on` / `logger off`. The one test that flies does so in simulation on
the FMU (SIH, `SYS_HITL=2`) with `pwm_out_sim` in place of real outputs, and
it lives in its own directory so there is no ambiguity about which is which.

## Layout

```
Tools/bench_test/
  README.md
  pyproject.toml            # pip install -e Tools/bench_test  (optional)
  run_bench_suite.py        # orchestrator for the non-interactive bench tests
  px4bench/                 # shared library package
    __init__.py             # Reporter, connect, MavlinkShell, reboot/replug,
                            # viewer tee, mavlink-status parsers,
                            # pymavlink add_message workaround
    params.py               # param read/set/drain/echo, int32 union encoding
    missions.py             # mission items, upload/download/compare/clear
    ftp.py                  # MAVFTP list/download, ULog magic, log root
  bench/                    # real-firmware bench tests, no simulation
    boot_health.py
    reboot_loop.py
    usb_replug.py
    link_forwarding.py
    param_stress.py
    mission_stress.py
    log_transfer.py
  sih/                      # simulation-in-hardware (SYS_HITL=2) tests
    flight_mission.py
```

Every script is directly executable from any working directory; no install
is required (each carries a path shim to find `px4bench/`). Installing the
library is optional and only needed if you want `import px4bench` from your
own scripts:

```
pip install -e Tools/bench_test          # library only
pip install -e "Tools/bench_test[ulog]"  # + pyulog for deep ULog verification
```

## Requirements

- Python 3.8+.
- Required: `pymavlink`, `pyserial` (`pip3 install --user pymavlink pyserial`).
- Optional: `pyulog` (deep .ulg verification; a magic-byte check is the
  fallback).

Finding the board on macOS: `ls /dev/tty.usbmodem*`. Close QGroundControl
first; it holds the serial port.

`CONNECTION` is a serial device (`/dev/tty.usbmodem*`) or `udp:IP:PORT` /
`tcp:IP:PORT`. Default baud is 57600. Every script prints `[PASS]` /
`[FAIL]` per check and exits nonzero on any failure.

## Quick start

```
# full non-interactive bench suite, single USB link
./run_bench_suite.py /dev/tty.usbmodem01

# with a telemetry radio as a second link (enables link_forwarding and
# alternating mission uploads)
./run_bench_suite.py /dev/tty.usbmodem01 /dev/tty.usbserial-RADIO

# operator-assisted USB re-enumeration test (not part of the suite)
./bench/usb_replug.py /dev/tty.usbmodem01

# simulated flight on the FMU (reconfigures the board, run separately)
./sih/flight_mission.py /dev/tty.usbmodem01
```

## Why pymavlink

The bench tests exercise the raw MAVLink protocol surface on purpose: param
echo semantics (duplicate PARAM_VALUE broadcasts are part of what we
verify), mission handshake retransmits and per-seq item requests, MAVFTP
burst-read internals with stall detection by byte count, and the nsh shell
over SERIAL_CONTROL. MAVSDK abstracts exactly those layers away, and adds an
asyncio runtime plus the mavsdk_server gRPC binary as dependencies. The one
plausible candidate was flight orchestration in `sih/flight_mission.py`, but
that test also depends on the nsh shell (arming via `commander`, explicit
`param save`, enabling viewer streams) and on teeing raw frames to a viewer,
neither of which MAVSDK exposes. Verdict: pymavlink everywhere.

## Risk map

Repetition is the test: single-shot operations do not hit races, which is
why the stress tests loop.

| Test | Subsystem exercised | Why it exists / what failure looks like |
| --- | --- | --- |
| bench/boot_health | WorkQueue lifetime + uORB callback rework | Task in ERROR state, required work queue missing, topic not publishing; baseline mode catches rate drift across firmware versions |
| bench/reboot_loop | boot-time initialization ordering | Locked zero-init mutex bugs bite at boot; a board that boots once but not every time. No reconnect within timeout = FAIL with elapsed time |
| bench/usb_replug | mavlink instance lifecycle across USB re-enumeration | Instance count climbing or free RAM growing per replug cycle = leak in link teardown |
| bench/link_forwarding | reworked mavlink nested-send lock path (silently deadlockable on NuttX) | Silent traffic gap > 5s on either link, a stalled param download, or a dead link after heavy simultaneous two-link use |
| bench/param_stress | parameter storage concurrency from the 2026-07-02 TSAN batch (DynamicSparseLayer races, AtomicTransaction) | Echo/readback mismatch under repetition, a download that stalls mid-list, or a save that does not survive reboot |
| bench/mission_stress | new mission shared-state mutex (#27813) + dataman TSAN fixes | Upload/download round-trip corruption or a handshake that stalls; with two links the mutex is hit from two channels |
| bench/log_transfer | logger + MAVFTP (FILE_TRANSFER_PROTOCOL replies are nested sends) | A burst-read download that stops mid-file, or a ULog that fails integrity checks |
| sih/flight_mission | commander/navigator/land-detector flight logic on real NuttX scheduling | Arming, takeoff, waypoint progression, RTL, land detection or auto-disarm failing per-phase timeouts |

## Bench tests (real firmware, no simulation)

### boot_health

Snapshot of a running board: `top once`, `work_queue status`, `perf`,
`uorb top -1`, `mavlink status`, `free`, `ver all` over the MAVLink shell,
saved into a timestamped report dir.

```
./bench/boot_health.py CONNECTION [-b BAUD] [--report-dir DIR] [--require-wq LIST]
./bench/boot_health.py --baseline OLDDIR NEWDIR [--tolerance PCT]
```

PASS: no task in ERROR state, required work queues present
(`wq:rate_ctrl,wq:hp_default,wq:lp_default` by default), topics publishing.
Baseline mode diffs two report dirs offline: uORB rate drift beyond
tolerance, lost topics, or lost work queues are FAIL.

### reboot_loop

```
./bench/reboot_loop.py CONNECTION [-b BAUD] [--iterations N] [--cycle-timeout S]
```

Reboots via `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` (default 10 cycles). Each
cycle must produce a heartbeat and a completed shell command within the
timeout; a board that never comes back aborts the loop with the elapsed time.

### usb_replug (operator-assisted)

```
./bench/usb_replug.py CONNECTION [-b BAUD] [--cycles M] [--heartbeat-timeout S] [--ram-tolerance BYTES]
```

Prints UNPLUG/REPLUG instructions and detects the device node vanishing and
returning programmatically (no keyboard needed mid-test). PASS per cycle:
heartbeat back within the timeout, mavlink instance count equal to baseline,
free RAM within budget.

### link_forwarding (needs 2 links)

```
./bench/link_forwarding.py CONNECTION CONNECTION2 [-b BAUD] [--baudrate2 BAUD2] [--duration S]
```

Four phases: liveness on both links, sustained bidirectional traffic
(silent gap > 5s = FAIL), the nested-send hammer (full param download over
the radio while an nsh shell session runs over USB), and post-stress
liveness on both links. A hang in any phase is a FAIL naming the phase.

### param_stress

```
./bench/param_stress.py CONNECTION [-b BAUD] [--iterations N] [--param NAME] [--skip-reboot]
```

Full param download, then a set/readback loop (default 50) on
`SDLOG_UTC_OFFSET` (harmless scratch param, always restored), then
`param save` + reboot + persistence verification.

### mission_stress

```
./bench/mission_stress.py CONNECTION [CONNECTION2] [-b BAUD] [--baudrate2 BAUD2] [--iterations N] [--items K]
```

220-item mission upload/download/item-by-item compare/clear/verify-cleared,
default 10 iterations, alternating links when two are given.

### log_transfer

```
./bench/log_transfer.py CONNECTION [-b BAUD] [--log-duration S] [--report-dir DIR]
```

`logger on`, record, `logger off` (no arming), then download the newest
.ulg via MAVFTP and verify it (pyulog when available, ULog magic bytes
otherwise). A stalled transfer reports the byte count it reached.

## SIH test (simulation on the FMU)

### flight_mission

```
./sih/flight_mission.py CONNECTION [-b BAUD] [--airframe N] [--alt M]
                        [--viewer] [--viewer-port PORT] [--keep-config]
                        [--board-dev DEV] [--report-dir DIR]
```

Switches the board to a SIH airframe (`SYS_AUTOSTART=1100`, `SYS_HITL=2`,
physics simulated on the FMU, `pwm_out_sim` in place of real outputs), flies
takeoff, a 3-waypoint square, and RTL as an auto mission with per-phase
timeouts (arming, airborne, waypoint progression, touchdown, auto-disarm),
downloads the flight ULog, and restores the original configuration even on
failure. `--viewer` tees every MAVLink frame to UDP so Hawkeye
(`hawkeye -udp 19410 -mc`) renders the flight live from the serial-connected
board.

This is deliberately separated from `bench/`: it reconfigures the board and
exercises simulated flight logic, while the bench tests exercise real
firmware paths without simulation.

## Baseline workflow

Each `boot_health` capture creates a timestamped directory under
`--report-dir` and prints its path; the diff takes those printed paths:

1. On known-good firmware: `./bench/boot_health.py CONNECTION`, note the
   printed report dir.
2. Flash the candidate firmware.
3. Capture again, note the new report dir.
4. `./bench/boot_health.py --baseline <old dir> <new dir> [--tolerance PCT]`

## Adding a new test

Conventions the suite relies on; new tests should follow all of them:

- Use `px4bench.Reporter` for every check; end with
  `sys.exit(report.finish())` so the exit code reflects the result.
- Use `px4bench.add_connection_args(parser)` so the CLI surface stays
  uniform (`CONNECTION`, `--baudrate/-b`, `--connect-timeout`; add
  `--report-dir` if the test writes artifacts).
- Every wait has a timeout. A timeout is a FAIL whose detail names what
  stalled (command, phase, seq, byte count). Never block forever: a hung
  test hides the hang it was supposed to expose.
- Restore any board state you change (params, airframe), in a `finally`,
  even when the test failed.
- No arming in `bench/`. Anything that flies (simulated) goes in `sih/`.
- Reuse the library: connection and shell primitives from `px4bench`,
  protocol helpers from `px4bench.params` / `.missions` / `.ftp`. If two
  tests need the same helper, it moves into the library.
- Shebang, exec bit, and the parent-dir path shim so the script runs
  without installation; add non-interactive tests to `run_bench_suite.py`.

Two behaviors are hardware-learned and must not be "simplified" away:
parameter echoes are drained before a set and matched by expected value
(PX4 emits multiple PARAM_VALUE per set), and the shell strips every
BENCHDONE sentinel from captured output (the second safety echo of the
previous command can arrive late).

## Safety

- Bench use only. Props off.
- `bench/` never arms the vehicle. `sih/flight_mission.py` arms only in
  simulation (`SYS_HITL=2`, `pwm_out_sim`); nothing is driven on the output
  rails.
- `param_stress`, `reboot_loop`, and `flight_mission` reboot the board.
  Expect the link to drop and return.
- Scratch params and airframe configuration are restored automatically,
  including on failure.

## Troubleshooting

No heartbeat:

- Wrong device. Confirm with `ls /dev/tty.usbmodem*` and pass the exact path.
- QGroundControl (or another GCS) is holding the port. Close it.
- Wrong baud on a radio link. Match the radio's configured baud with `-b` /
  `--baudrate2`.

Shell stalls: if a shell-based stage hangs, that stall is the finding, not a
tooling bug. Note which command it stalled on and keep the report dir.

Telemetry radio bandwidth: dual-link traffic, param download, and mission
transfer over a radio are much slower than USB. Allow longer durations
(`--duration`, `--per-test-timeout`) when a radio is in the path.
