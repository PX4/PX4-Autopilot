# px4bench: PX4 bench-test suite

Semi-automated release qualification for PX4 on real NuttX hardware
(Pixhawk-class boards on a bench, connected over USB and optionally a
telemetry radio).

CI builds every NuttX target but never boots one, and SITL runs the same
code on a host OS whose threading, scheduling, and libc behave differently
from the RTOS on the flight controller. That gap hides an entire class of
defects that only exist on real hardware: boot-time initialization ordering,
link lifecycle across USB re-enumeration, storage and filesystem behavior,
loop-rate drift, and concurrency bugs that are invisible on glibc but fatal
on NuttX. A board can pass every SITL test and still ship with a dead
telemetry link.

px4bench closes that gap with repeatable, scriptable verification on the
bench: qualify a release candidate before cutting a branch, baseline a board
before and after a firmware upgrade, or hand a beta to a test team knowing
the basics hold. Failures on embedded targets are often silent hangs rather
than crashes, so the suite is built around one rule: every operation has a
timeout, and a timeout is reported as FAIL naming exactly what stalled. A
hang is the finding, not a nuisance.

The tests under `bench/` never arm the vehicle; logging is triggered with
`logger on` / `logger off`. The default suite ends with one test that flies
in simulation on the FMU (SIH, `SYS_HITL=2`, `pwm_out_sim` in place of real
outputs): it arms the flight controller, so it runs behind an explicit
arming confirmation (or `--allow-arming` for automation) and the board must
be bare, with nothing on the output rails. It lives in `sih/` so the
simulation/no-simulation boundary stays obvious.

## Layout

```
Tools/bench_test/
  README.md
  pyproject.toml            # pip install -e Tools/bench_test  (optional)
  run_bench_suite.py        # orchestrator for the non-interactive bench tests
  px4bench/                 # shared library package
    __init__.py             # Reporter, connect, MavlinkShell, reboot/replug,
                            # mavlink-status parsers,
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
    storage_stress.py
    log_transfer.py
    serial_loopback.py
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
# flash a known build, verify it, then run the full bench suite
./run_bench_suite.py /dev/tty.usbmodem01 --firmware build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.px4

# with a telemetry radio as a second link (enables link_forwarding and
# alternating mission uploads)
./run_bench_suite.py /dev/tty.usbmodem01 /dev/tty.usbserial-RADIO --expect-hash 0c000d59

# operator-assisted tests (not in the suite: need an operator or a fixture)
./bench/usb_replug.py /dev/tty.usbmodem01
./bench/serial_loopback.py /dev/tty.usbmodem01 --device /dev/ttyS2

# the simulated flight can also run standalone
./sih/flight_mission.py /dev/tty.usbmodem01 --expect-hash 0c000d59
```

The default suite sequence is boot_health, param_stress, mission_stress
(link_forwarding after it when a second link is given), storage_stress,
log_transfer, reboot_loop, and finally the simulated flight
(flight_mission). The flight arms the flight controller: on a TTY you are
asked to type 'arm' to confirm (declining records a skip, not a failure);
non-interactive runs skip it unless `--allow-arming` is passed. Tests that
probe the firmware and find a needed command missing record SKIP with a
warning instead of failing.

## Firmware gate

Qualification means knowing what you tested. Before any test starts the
suite connects, reads the board identity (`ver all`: PX4 git-hash, PX4
version, HW arch, OS version), prints it, writes it to `firmware.json` in
the suite report dir, and stamps it into every test's report dir. What
happens next depends on exactly one of five mutually exclusive flags,
covering four firmware sources:

```
# 1. keep what is on the board (explicit opt-out, still stamped)
./run_bench_suite.py /dev/tty.usbmodem01 --any-firmware

# 2. flash a local .px4, verify the flashed identity, then test
./run_bench_suite.py /dev/tty.usbmodem01 --firmware path/to/px4_fmu-v6xrt_default.px4

# 3. build from this source tree, flash, verify (target inferred from the
#    connected board via its HW arch; pass --target if ambiguous)
./run_bench_suite.py /dev/tty.usbmodem01 --build

# 4. download a GitHub release artifact (needs the gh CLI), flash, verify
./run_bench_suite.py /dev/tty.usbmodem01 --release v1.17.0
./run_bench_suite.py /dev/tty.usbmodem01 --release latest

# verify only, no flash: assert the board already runs a given hash
./run_bench_suite.py /dev/tty.usbmodem01 --expect-hash 0c000d59
```

All flash paths converge: parse the .px4 metadata (refusing unparseable
files), check its `board_id` against the connected board early (wrong-board
images fail before flashing; the uploader enforces it again against the
bootloader), flash via `Tools/px4_uploader.py`, wait for re-enumeration and
a heartbeat, then re-read `ver all` and compare the git hash against the
artifact's identity (named check `firmware_identity`, prefix match). A
mismatch after flashing aborts the suite.

Interactive use: with none of the flags on a TTY, the suite shows the
detected identity and asks: continue on the current firmware, flash a local
.px4, build the inferred target from this tree, download a release, or
abort. In automation (stdin not a TTY) the gate refuses to guess and exits
with an error listing the flags; CI must always state what it is testing.

With `--build`, after resolving the target the gate reads the board config
(default.px4board plus the label overlay for labeled targets) and prints a
bench capability report: which bench-relevant modules the image will
contain (simulator_sih, sd_bench, sd_stress, serial_test) and which suite
tests will therefore run or skip. For anything missing it names a sibling
variant of the same board that has it (for example
`--target px4_fmu-v6xrt_bench`) and the exact config line to add;
interactively it offers to append the line to the local board config,
which stays local and uncommitted (commit it yourself for it to stick).

Wedged-board note: a board whose mavlink is hung cannot soft-reboot into
the bootloader, so the uploader sits in its reboot-request loop. The gate
detects this and prints an operator instruction to unplug and replug USB;
the uploader then catches the bootloader at power-on.

`sih/flight_mission.py` accepts `--expect-hash` as a verify-only gate and
stamps the identity into its report dir; it never flashes.

## Why pymavlink

The bench tests exercise the raw MAVLink protocol surface on purpose: param
echo semantics (duplicate PARAM_VALUE broadcasts are part of what we
verify), mission handshake retransmits and per-seq item requests, MAVFTP
burst-read internals with stall detection by byte count, and the nsh shell
over SERIAL_CONTROL. MAVSDK abstracts exactly those layers away, and adds an
asyncio runtime plus the mavsdk_server gRPC binary as dependencies. The one
plausible candidate was flight orchestration in `sih/flight_mission.py`, but
that test also depends on the nsh shell (arming via `commander`, explicit
`param save`), which MAVSDK does not expose. Verdict: pymavlink everywhere.

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
| bench/storage_stress | SD card and FAT filesystem under the logger's feet | Sequential write/read below the floor, fsync latency spikes that cause log dropouts, or file churn failures |
| bench/log_transfer | logger + MAVFTP (FILE_TRANSFER_PROTOCOL replies are nested sends) | A burst-read download that stops mid-file, or a ULog that fails integrity checks |
| bench/serial_loopback | UART driver TX/RX path through a physical loopback jumper (fixture) | Lost or corrupted bytes through the jumper, or a port that cannot sustain the byte rate |
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

### storage_stress

```
./bench/storage_stress.py CONNECTION [-b BAUD] [--report-dir DIR]
                          [--bench-runs N] [--bench-duration-ms MS] [--bench-block B]
                          [--min-write-kbs X] [--min-read-kbs Y] [--max-fsync-ms Z]
                          [--stress-runs N] [--stress-bytes B]
```

Drives the firmware's own storage tools over the shell: `sd_bench` with
data verification for sequential write/read throughput and fsync latency,
then `sd_stress` for file create/rename/delete churn (100 files per
iteration). Probes first: firmware without the commands, or a board
without an SD card, records SKIP with a warning; the two phases degrade
independently (sd_bench present but sd_stress absent runs only the bench
phase).

Threshold defaults are generous FAIL floors, chosen so a slow but working
card warns instead of flapping:

- `--min-write-kbs 100`: the default logger profile needs roughly
  50-100 KB/s sustained; below 100 KB/s sequential write at 4 KB blocks a
  card cannot log reliably. Healthy cards do an order of magnitude more.
- `--min-read-kbs 200`: far below any usable card; catches only genuinely
  broken read paths.
- `--max-fsync-ms 500`: fsync stalls beyond a few hundred ms are what
  cause logger dropouts; occasional 100-300 ms peaks on cheap cards are
  normal and stay under this ceiling.

Anything above a floor but within 4x of it additionally prints a WARNING
line so marginal cards are visible without failing the suite.

### log_transfer

```
./bench/log_transfer.py CONNECTION [-b BAUD] [--log-duration S] [--report-dir DIR]
```

`logger on`, record, `logger off` (no arming), then download the newest
.ulg via MAVFTP and verify it (pyulog when available, ULog magic bytes
otherwise). A stalled transfer reports the byte count it reached.

### serial_loopback (operator/fixture, not in the suite)

```
./bench/serial_loopback.py CONNECTION --device /dev/ttySn [-b BAUD]
                           [--test-baud BAUD] [--seconds S] [--no-prompt]
                           [--report-dir DIR]
```

Manufacturing/end-of-line check for the UART hardware path. The operator
(or the test fixture) installs a loopback jumper wiring TX to RX on the
UART under test, then `serial_test` transmits a known pattern for
`--seconds` and verifies its own received bytes. PASS means bytes flowed
both ways with zero pattern errors and matching rx/tx counts. The port
must not be claimed by a running service (mavlink instance, GPS, RC);
`--no-prompt` skips the jumper pause for fixtures that pre-install it.
Skips with a warning when the firmware lacks `serial_test`
(CONFIG_SYSTEMCMDS_SERIAL_TEST=y).

## SIH test (simulation on the FMU)

### flight_mission

```
./sih/flight_mission.py CONNECTION [-b BAUD] [--airframe N] [--alt M]
                        [--keep-config] [--allow-arming]
                        [--expect-hash PREFIX] [--report-dir DIR]
```

Switches the board to a SIH airframe (`SYS_AUTOSTART=1100`, `SYS_HITL=2`,
physics simulated on the FMU, `pwm_out_sim` in place of real outputs), flies
takeoff, a 3-waypoint square, and RTL as an auto mission with per-phase
timeouts (arming, airborne, waypoint progression, touchdown, auto-disarm),
downloads the flight ULog, and restores the original configuration even on
failure.

Runs last in the default suite and works standalone. Before doing anything
it probes the live firmware for the SIH module (`simulator_sih status`); if
nsh replies command not found, it warns and records SKIP instead of
failing. It then passes the arming gate: the flight controller WILL ARM
(simulated flight, but the board must be bare), so on a TTY the operator
must type 'arm' to proceed, and non-interactive runs skip unless
`--allow-arming` is given. Reconfiguring the board (airframe, `SYS_HITL`)
is expected and accepted; everything is restored afterwards, `--keep-config`
opts out. The `sih/` directory split only marks the simulation/no-simulation
boundary.

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
- No arming in `bench/`. Anything that flies (simulated) goes in `sih/`
  and must pass `px4bench.arming_gate` first.
- Depend on optional firmware features via probe-and-skip: check with
  `px4bench.shell_command_exists` and exit `px4bench.EXIT_SKIP` (recorded
  as SKIP by the orchestrator) with a warning naming the config option,
  never FAIL for a feature the build simply does not have.
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
- `bench/` never arms the vehicle. `sih/flight_mission.py` (last in the
  default suite) arms only in simulation (`SYS_HITL=2`, `pwm_out_sim`);
  nothing is driven on the output rails, but the board must be bare. It
  asks for confirmation on a TTY and requires `--allow-arming` in
  automation.
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
