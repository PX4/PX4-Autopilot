# Hardware Bench Testing (px4bench)

px4bench is the PX4 toolkit for automated verification on real flight-controller hardware.
It qualifies a firmware build on a board sitting on a bench: no props, no airframe, just a USB cable and optionally a telemetry radio.

Use this guide if you are:

- **Qualifying a release**: verify a release candidate on real NuttX hardware before cutting a branch or handing a beta to a test team.
- **Sanity-checking a change**: prove that a PR or local build boots, communicates, stores parameters and missions, and survives reboots on a real board.
- **Testing flight logic hardware-in-the-loop**: fly a full auto mission in simulation on the FMU itself (SIH) and verify arming, takeoff, mission progression, landing, and disarm on real RTOS scheduling.
- **Running end-of-line checks in production**: manufacturers can run the same suite against every unit that comes off the line, with machine-readable pass/fail results and full firmware traceability per unit.

The toolkit lives in the PX4-Autopilot source tree at [`Tools/bench_test/`](https://github.com/PX4/PX4-Autopilot/tree/main/Tools/bench_test), and its [README](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/bench_test/README.md) is the complete reference.
This page explains what it does and how to apply it.

## Why Bench Testing

CI builds every NuttX target but never boots one, and [SITL](../simulation/index.md) runs flight code on a host operating system whose threading, scheduling, and C library behave differently from the RTOS on a flight controller.
Some defects therefore only exist on real hardware: boot-time initialization ordering, link lifecycle across USB re-enumeration, storage and filesystem behavior, loop-rate drift, and concurrency bugs that are invisible on a desktop OS.
A build can pass every SITL test and still ship with a dead telemetry link.

Failures on embedded targets are frequently silent hangs rather than crashes, so px4bench is built around one rule: **every operation has a timeout, and a timeout is reported as a failure naming exactly what stalled**.
Every check prints `[PASS]` or `[FAIL]`, every test exits nonzero on failure, and every run writes a timestamped report directory containing the raw evidence.

## Firmware Traceability

Qualification means knowing what you tested.
Before any test runs, the suite reads the board's identity (git hash, version, hardware) and records it in `firmware.json` in every report directory.
You state where the firmware under test comes from, and the suite verifies the flashed board actually reports the expected git hash:

```sh
# flash a local .px4 file, verify its identity, then test
./run_bench_suite.py /dev/ttyACM0 --firmware px4_fmu-v6xrt_default.px4

# build the current source tree for the connected board, flash, verify, test
./run_bench_suite.py /dev/ttyACM0 --build

# download a GitHub release artifact, flash, verify, test
./run_bench_suite.py /dev/ttyACM0 --release v1.17.0

# no flashing: assert the board already runs the expected build
./run_bench_suite.py /dev/ttyACM0 --expect-hash 0c000d59

# explicit opt-out: test whatever is on the board (still recorded)
./run_bench_suite.py /dev/ttyACM0 --any-firmware
```

Run interactively with none of these flags, the suite shows what is on the board and asks what to do.
Run non-interactively (CI, production scripting), it refuses to start without an explicit firmware expectation, so automation can never silently test the wrong build.

## The Bench Suite

The bench tests run against normal firmware and never arm the vehicle.

| Test               | What it verifies                                                                                                        |
| ------------------ | ----------------------------------------------------------------------------------------------------------------------- |
| `boot_health`      | System snapshot over the MAVLink shell: no task in error state, required work queues present, uORB topics publishing. Has a baseline mode (see below). |
| `reboot_loop`      | Repeated reboot/reconnect cycles; catches boards that boot once but not every time.                                       |
| `usb_replug`       | USB unplug/replug cycles (operator-assisted); catches link-teardown leaks in mavlink instance count or RAM.               |
| `link_forwarding`  | Simultaneous heavy traffic on two links with forwarding; catches deadlocks and stalls in the MAVLink send path. Requires a second link. |
| `param_stress`     | Full parameter download, 50 set/readback cycles, save, reboot, persistence verification.                                  |
| `mission_stress`   | Repeated 220-waypoint upload/download/compare/clear transactions, alternating links when two are available.               |
| `log_transfer`     | Short log capture, MAVFTP download, ULog integrity verification.                                                          |

```sh
cd Tools/bench_test
./run_bench_suite.py /dev/ttyACM0 --build                       # everything, single link
./run_bench_suite.py /dev/ttyACM0 /dev/ttyUSB0 --expect-hash <hash>  # + radio: adds dual-link tests
```

A per-test watchdog turns a hung board into a named failure instead of a stuck terminal.

## Simulated Flight on the FMU (SIH)

`sih/flight_mission.py` performs a complete hardware-in-the-loop flight without any host-side simulator: it switches the board to a [SIH airframe](../sim_sih/index.md) (`SYS_HITL=2`, physics computed on the autopilot), uploads a mission, arms through the NuttX shell, and asserts takeoff, waypoint progression, RTL, landing, and auto-disarm against per-phase timeouts.
The flight log is downloaded into the report directory automatically for post-flight analysis, and the original board configuration is restored afterwards.

Real outputs are replaced by `pwm_out_sim` in this mode, so nothing is driven on the output rails; still, run it only on a bench board with nothing connected.
The firmware must be built with `CONFIG_MODULES_SIMULATION_SIMULATOR_SIH=y`.

To watch the flight live in 3D, install [Hawkeye](https://github.com/PX4/Hawkeye) and run the test with `--viewer`:

```sh
hawkeye -udp 19410 -mc &
./sih/flight_mission.py /dev/ttyACM0 --viewer
```

## Baseline Comparison

`boot_health` can diff two report directories, comparing uORB publication rates, work queues, and topics between runs:

```sh
./bench/boot_health.py /dev/ttyACM0                      # capture (prints report dir)
# ... flash a different build ...
./bench/boot_health.py /dev/ttyACM0
./bench/boot_health.py --baseline <old-dir> <new-dir>    # FAIL on rate drift or lost topics
```

This supports two workflows:

- **Upgrade regression**: capture on the current release, capture on the candidate, diff. A control-loop rate that quietly dropped between versions is a finding long before it becomes mushy flight behavior.
- **Golden unit** (manufacturing): capture a known-good reference unit once, then diff every produced unit against it. A board with a marginal sensor or misloaded configuration deviates from the golden baseline even when it nominally boots.

## Production End-of-Line Usage

For manufacturers the suite is designed to run unattended per unit:

```sh
./run_bench_suite.py "$PORT" --firmware "$RELEASE_PX4" --report-dir "reports/$SERIAL" \
  || echo "UNIT $SERIAL FAILED"
```

- Exit code is the verdict: `0` all checks passed, nonzero otherwise, with the failing check named in the output.
- `reports/<serial>/` retains the full evidence per unit: firmware identity, system snapshot, parameter and mission transaction results, downloaded logs.
- The firmware gate guarantees each unit was tested against the intended production image, not whatever was in flash.
- `usb_replug` (connector/enumeration exercise) is the only operator-assisted test and can be included where a physical connector check is part of the procedure.

## CI Integration

The suite is built to sit behind a self-hosted runner with a board permanently attached: non-interactive runs demand an explicit firmware source, all results are machine-readable exit codes plus report artifacts, and a wedged board fails fast with a named check rather than hanging the job.
Pairing `--build` (current revision) with the bench suite plus a SIH flight closes the gap where CI compiles NuttX firmware without ever executing it.

## Further Information

- [`Tools/bench_test/README.md`](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/bench_test/README.md): complete reference (all flags, per-test details, troubleshooting, how to add a test)
- [SIH on Hardware](../sim_sih/hardware.md)
- [Test Flights](../test_and_ci/test_flights.md): real flight testing, which bench testing complements but does not replace
