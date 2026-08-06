# Offline Motor Failure Replay

<Badge type="tip" text="main (PX4 v2.0)" />

The Offline Motor Failure Replay tool (`mfd_replay`), with its companion model-fitting tool (`mfd_fit`), is a command line tool that you can run on your development computer to calibrate and verify the [motor failure detector](../config/motor_failure_detection.md)'s [threshold parameters](../config/motor_failure_detection.md#choosing-parameter-values) (`MOTFAIL_*`) against a vehicle's own flight logs.
It allows users to determine the tightest failure detection thresholds that still don't trip on healthy flight from log data.

`mfd_replay` replays recorded logs through exactly the same detector code that runs on the firmware, and prints what the detector would have decided for each motor (this matters because the thresholds are airframe-specific and can only be derived from the vehicle's own flight data).

Thresholds usually have to be calculated from a set of flights, as a single flight is unlikely to fully test the vehicle.
As a result, calibration work is usually done via `batch_replay.sh`, a wrapper that runs `mfd_replay` over every log in a directory.
For more information see [Replaying a Fleet of Logs](#replaying-a-fleet-of-logs).

Typical uses:

- Confirm a candidate configuration never trips on a set of healthy flights.
- Find the tightest thresholds a vehicle's data supports, by sweeping values and re-running.
- Check after the fact whether the detector would have flagged a motor in a flight where something went wrong.

## Building

To build the tool:

1. Build PX4 SITL (to populate the build directory)

   ```bash
   make px4_sitl_default # once, if the build directory does not exist yet
   ```

2. Build the tool binaries:

   ```bash
   cmake -DMFD_REPLAY_TOOL=ON build/px4_sitl_default
   ninja -C build/px4_sitl_default mfd_replay mfd_fit
   ```

This produces `build/px4_sitl_default/mfd_replay` and `build/px4_sitl_default/mfd_fit`.
`mfd_fit` derives the current model from logs; `mfd_replay` then runs the detector with it.

## Fitting the Current Model

`mfd_fit` reads ESC current and motor command from as many logs as you pass it, and fits the line the detector uses.
It counts only armed samples where the ESC reported a current, and skips motors that were switched off or running in reverse.

```bash
./build/px4_sitl_default/mfd_fit ~/logs/healthy/*.ulg
```

```text
# 6 logs, 2103903 armed samples
# I_expected = 32.49*u + -0.53 A   (residual sigma 0.84 A)
# mean command 0.241, mean current 7.30 A

MOTFAIL_C2T 32.49
MOTFAIL_IDLE -0.53

# NOTE: the fitted offset is negative, and MOTFAIL_IDLE cannot be set below 0.
#       Use MOTFAIL_IDLE 0; an offset this small is within the residual anyway.

# per-motor mean residual against this model [A]
#   motor 0: +0.15  (sigma 0.91, n 350092)
#   ...
#   motor 5: -0.15  (sigma 0.82, n 349883)
# worst per-motor bias 0.25 A -- the trip bands have to clear this before any fault.
```

Set what the tool prints, with one exception: the fit sometimes puts the offset below zero, as it has here.
[MOTFAIL_IDLE](../config/motor_failure_detection.md#MOTFAIL_IDLE) cannot be set below 0, so use 0 — an offset that small is inside the residual either way.

Two numbers matter beyond the parameters themselves:

- **residual sigma** — the spread the trip bands have to clear, which caps how tight they can be.
- **worst per-motor bias** — what one shared model costs the worst motor. While it stays small against the bands, per-motor models buy nothing.

Pass the whole set in one command: the fit is pooled over every log, and a hundred logs takes under a second.

::: details Running it over a large set of logs
`mfd_fit` takes the logs themselves rather than a directory, so use a glob, and a recursive one (`**/*.ulg`) if they are nested.

Memory does not grow with the size of a log or with how many you pass (91 logs and 1.5 GB fit in 0.8 s and 5 MB), so there is never a reason to split the work up.
In particular, do not pipe the list through `xargs`: it starts a new process once the list gets long, which would give you several partial fits instead of one.

Check the `# N logs` line against the number of files you passed.
A log without ESC current contributes nothing and is named on stderr, which is easy to miss on a large set, so redirect it with `2> skipped.txt` if you want to know which ones dropped out.
:::

::: warning
Fit on healthy flights only.
The fit cannot tell a faulty motor from a healthy one, so a single failure-test log pulls the model toward that fault and shifts the residual of every healthy motor with it.
:::

Fit the offset, do not assume one.
A borrowed offset biases every healthy motor's residual by that amount, which the undercurrent band then has to absorb.
Using a plausible-looking 2.88 A instead of the fitted value turns all six healthy motors of the earlier example into failures, at bands that were clean:

```text
# config (...):  I_exp = 32.50*u + 2.88 A,  trip if LPF(I-I_exp) < -3.50 for 0.50 s or > 6.00 for 0.50 s
# motor  peak|LPF(r)|[A]  trips  verdict
      0    5.12 - 6.68    5/5    FAILED (all phases)
      ...
      5    5.30 - 6.62    5/5    FAILED (all phases)
```

## Running the Detector on One Log

```bash
./build/px4_sitl_default/mfd_replay flight.ulg
```

The log must contain `esc_status` (per-motor current), `actuator_motors` (the motor commands) and `vehicle_status` (the arming state).
Fields are resolved by name from the log's own format, so one binary reads logs from any PX4 version, with 8 or 12 ESCs, without rebuilding.
A log that is not a ULog, or that is missing a needed field, is reported as an error rather than silently misread.

Output for a healthy flight with a calibrated configuration:

```text
# flight.ulg  (5 phases, 3557 armed samples each)
# config (built-in calibration, overrides: MOTFAIL_C2T MOTFAIL_IDLE MOTFAIL_OVER MOTFAIL_OVR_TIME MOTFAIL_UNDER MOTFAIL_UND_TIME):  I_exp = 32.50*u + 0.00 A,  trip if LPF(I-I_exp) < -3.50 for 0.50 s or > 6.00 for 0.50 s
# motor  peak|LPF(r)|[A]  trips  verdict
      0    2.65 - 4.52    0/5    ok
      1    2.04 - 3.86    0/5    ok
      2    2.50 - 3.65    0/5    ok
      3    2.74 - 4.21    0/5    ok
      4    2.50 - 3.68    0/5    ok
      5    2.42 - 3.74    0/5    ok
```

| Column           | Meaning                                                                                                                                                          |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `motor`          | Motor index, counting from 0, as mapped from each ESC's actuator function.                                                                                       |
| `peak\|LPF(r)\|` | Largest filtered residual magnitude reached during the flight, as a range across the sampling phases (see below). This is the number thresholds are chosen from. |
| `trips`          | How many of the sampling phases flagged this motor.                                                                                                              |
| `verdict`        | `ok`, `FAILED (all phases)`, or `FAILED (sampling-dependent!)` when only some phases flagged it.                                                                 |

Exit codes make the tool usable as a check in a script: `0` if every motor stayed healthy, `1` if any motor was flagged, `2` if the log could not be used.

### The Sampling Phase Sweep

On the vehicle the check runs at 10 Hz, so it sees roughly one ESC sample in ten, and _which_ samples it sees depends on the alignment between the check and the telemetry stream.
That alignment cannot be reconstructed from a log.

The tool therefore runs several detectors over the same log, each starting its 10 Hz grid at a different offset, and reports the spread.
A single-phase answer would be a lower bound on the trip risk, not a verdict.

Read the result as follows:

- `0/5` with a narrow peak range: the configuration is comfortably clear of this flight's residual.
- `0/5` with a wide peak range: it happened not to trip, but the margin is thin and the same flight could trip on the vehicle.
- `1/5` … `4/5` (`sampling-dependent!`): treat this as a false positive, not as a pass.
- `5/5`: it would have tripped.

The tool prints a reminder to this effect whenever any motor's verdict or peak is phase-sensitive.

### Where the Configuration Comes From

For each parameter, in order of precedence:

1. `--set NAME=value` on the command line.
2. The value recorded in the log, if the log contains the `MOTFAIL_*` parameters.
3. A built-in fallback.

The header line always prints the configuration that was used, and names any parameter that came from `--set`.

::: warning
The built-in fallback is an example fit for one particular vehicle, not a recommended default.
On another airframe it will report failures on perfectly healthy flights.
Always replay with the configuration you are actually evaluating.
:::

Overriding one knob at a time is how the band sweeps below are done:

```bash
./build/px4_sitl_default/mfd_replay flight.ulg --set MOTFAIL_UNDER=3.5 --set MOTFAIL_UND_TIME=0.5
```

The accepted names are [MOTFAIL_C2T](../config/motor_failure_detection.md#MOTFAIL_C2T), [MOTFAIL_IDLE](../config/motor_failure_detection.md#MOTFAIL_IDLE), [MOTFAIL_UNDER](../config/motor_failure_detection.md#MOTFAIL_UNDER), [MOTFAIL_UND_TIME](../config/motor_failure_detection.md#MOTFAIL_UND_TIME), [MOTFAIL_OVER](../config/motor_failure_detection.md#MOTFAIL_OVER) and [MOTFAIL_OVR_TIME](../config/motor_failure_detection.md#MOTFAIL_OVR_TIME).

## Choosing the Bands

The goal is the tightest bands that never trip on healthy flight, so the logs have to cover the throttle range the vehicle really flies, including climbs and aggressive manoeuvres: the largest deviation across them is what sets the achievable threshold.
As with the fit, every log has to come from a flight where all motors were healthy.

The steps below replay a whole set of logs with `batch_replay.sh`, which runs `mfd_replay` over every `.ulg` in a directory and summarizes the verdicts — see [Replaying a Fleet of Logs](#replaying-a-fleet-of-logs).

**1. Read the healthy residual.**

Replay the set of flight logs with the fitted model and the bands set out of the way, so nothing trips and the peak column reports the full excursion:

```bash
src/lib/motor_failure_detector/tools/batch_replay.sh ~/logs/healthy \
  --set MOTFAIL_C2T=32.5 --set MOTFAIL_IDLE=0 --set MOTFAIL_OVER=999 --set MOTFAIL_UNDER=999
```

The largest peak over all logs and motors is the floor for a symmetric band.
It is usually much larger than what the two-sided configuration needs, because the biggest excursions are brief.

**2. Sweep each band against its hold time.**

Sweep one direction at a time.
There is no switch to disable a single direction, so park the other one at a value nothing will reach, such as `999`.
Do not use `0` for this: [MOTFAIL_OVER](../config/motor_failure_detection.md#MOTFAIL_OVER)`=0` switches off the whole check rather than one side of it.

For each hold time, the value you are looking for is the lowest band at which the whole set of flight logs still comes back with `FAIL=0`:

```bash
#!/usr/bin/env bash
# undercurrent sweep -- substitute the MOTFAIL_C2T / MOTFAIL_IDLE you fitted above
for hold in 0.1 0.3 0.5 0.7 1.0; do
  for band in 2.0 2.5 3.0 3.5 4.0 4.5; do
    summary=$(src/lib/motor_failure_detector/tools/batch_replay.sh ~/logs/healthy \
      --set MOTFAIL_C2T=32.5 --set MOTFAIL_IDLE=0 --set MOTFAIL_OVER=999 \
      --set MOTFAIL_UNDER="$band" --set MOTFAIL_UND_TIME="$hold" | tail -1)
    echo "UND_TIME=$hold UNDER=$band  $summary"
  done
done
```

Each line of the output is one candidate configuration:

```text
UND_TIME=0.5 UNDER=2.5  total=6  OK=4  FAIL=2  SKIP=0
UND_TIME=0.5 UNDER=3.0  total=6  OK=6  FAIL=0  SKIP=0
```

So at a 0.5 s hold, 2.5 A is too tight, and 3.0 A is the lowest band this corpus supports.
Collecting one such value per hold time gives the table that [choosing parameter values](../config/motor_failure_detection.md#choosing-parameter-values) works from.

Then repeat with the directions swapped: `--set MOTFAIL_UNDER=999`, sweeping [MOTFAIL_OVER](../config/motor_failure_detection.md#MOTFAIL_OVER) and [MOTFAIL_OVR_TIME](../config/motor_failure_detection.md#MOTFAIL_OVR_TIME).
Expect the two sides to come out very differently.

::: warning
Write sweeps as `bash` scripts, and pass the arguments literally or through an array.
In `zsh`, an unquoted variable holding `--set A=1 --set B=2` is passed as a single argument.
The tool then prints a usage error and exits without a verdict table, and a loop that only counts failure lines reads that as "no trips".
Make the loop assert that a table was actually produced, as the `grep -q` guard does, before trusting a zero count.
:::

**3. Verify.**

Re-run the whole set of flight logs with the chosen configuration, including logs that were not used for the fit, and confirm every motor is `ok` with no phase-dependent verdicts.
Then check the detection floor the configuration implies: the undercurrent band divided by the per-motor hover current is the smallest current loss that can be seen at hover.

## Replaying a Fleet of Logs

A single flight doesn't usually contain enough information to calculate thresholds, so the usual unit of work is a set of flight logs.
`batch_replay.sh` runs the tool over every `.ulg` in a directory and summarizes the verdicts:

```bash
src/lib/motor_failure_detector/tools/batch_replay.sh ~/logs/healthy --set MOTFAIL_UNDER=3.5 --set MOTFAIL_UND_TIME=0.5
```

```text
OK    flight1.ulg
OK    flight2.ulg
OK    flight3.ulg
OK    flight4.ulg
OK    flight5.ulg
OK    flight6.ulg
----
total=6  OK=6  FAIL=0  SKIP=0
```

Anything after the directory is passed straight through to `mfd_replay`.
`SKIP` counts logs with no usable ESC data.
Run it from the repository root, or set `MFD_REPLAY` to the path of the binary.

`batch_replay.sh` works through the logs one at a time, which is fine for tens of them and slow for thousands.
For a large set of flight logs, run `mfd_replay` itself with one process per log, and give each its own output file so the verdict tables do not interleave:

```bash
mkdir -p /tmp/verdicts
ls ~/logs/healthy/*.ulg | xargs -P"$(nproc)" -I{} \
  sh -c './build/px4_sitl_default/mfd_replay "$1" > /tmp/verdicts/$(basename "$1").txt' _ {}

grep -l FAILED /tmp/verdicts/*.txt   # the logs that tripped
```

This gives the full per-motor table for every log instead of the one-line summary, which is what you want when you then have to look at _why_ something tripped.

## What the Replay Does Not Reproduce

- **The sampling phase**, which is why the sweep exists.
- **Real faults**, unless the log contains one.
  There is no fault injection: to study the true-positive side, replay a log from a flight where a motor really failed, or modify a log so that the current of one motor reflects the fault you want to study.
- **The parameter reload on disarm.**
  The replay configures the detector once and evaluates only while the log says the vehicle was armed.
- **Anything outside the current check.**
  The ESC telemetry timeout, the arming checks and the failure response all live elsewhere in the firmware and are not part of this tool.

## See Also

- [Motor Failure Detection](../config/motor_failure_detection.md) — what the detector does and how it is configured on the vehicle.
- [Flight Log Analysis](../dev_log/flight_log_analysis.md) — general tooling for working with ULog files.
