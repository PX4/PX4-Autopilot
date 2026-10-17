# Offline Motor Failure Replay

<Badge type="tip" text="main (PX4 v2.0)" />

Two command line tools calibrate the [motor failure detector](../config/motor_failure_detection.md) against a vehicle's own flight logs.
`mfd_fit` derives the current model from a set of logs, and `mfd_replay` runs the real detector over a log and prints what it would have decided for each motor.

They exist because the [threshold parameters](../config/motor_failure_detection.md#choosing-parameter-values) (`MOTFAIL_*`) are specific to an airframe, so the only place to get them is that airframe's own flights.
`mfd_replay` links the detector library itself, so a replay runs the same code that runs in flight.

One flight is rarely enough to characterise a vehicle, so most of the work below runs over a directory of them.

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
# 3 logs, 80697 armed samples
# I_expected = 6.91*u + 0.37 A   (residual sigma 0.50 A)
# mean command 0.387, mean current 3.04 A

MOTFAIL_C2T 6.91
MOTFAIL_IDLE 0.37

# per-motor mean residual against this model [A]
#   motor 1: -0.12  (sigma 0.64, n 13484)
#   ...
#   motor 6: -0.10  (sigma 0.46, n 13360)
# worst per-motor bias 0.12 A -- the trip bands have to clear this before any fault.
```

Set what the tool prints.
On some vehicles the fit puts the offset below zero, and [MOTFAIL_IDLE](../config/motor_failure_detection.md#MOTFAIL_IDLE) cannot be set below 0: use 0 in that case.
Nothing is lost by it, because the bands are then chosen against the residual that results, so they absorb the difference.

Two numbers matter beyond the parameters themselves:

- **residual sigma**: the spread the trip bands have to clear, which caps how tight they can be.
- **worst per-motor bias**: what the shared model costs the worst motor. The detector has one model for all motors, so this is error the bands absorb before any fault does.

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
Taking another airframe's 2.88 A instead of the fitted 0.37 A turns all six healthy motors into failures, at bands that were clean:

```text
# config (...):  I_exp = 6.91*u + 2.88 A,  trip if LPF(I-I_exp) < -2.25 for 0.70 s or > 1.00 for 2.50 s
# motor    min[A]    max[A]  trips  verdict
      1     -4.30     +1.69  5/5    FAILED (all phases)
      ...
      6     -4.75     +1.43  5/5    FAILED (all phases)
```

## Running the Detector on One Log

```bash
./build/px4_sitl_default/mfd_replay flight.ulg \
  --set MOTFAIL_C2T=6.91 --set MOTFAIL_IDLE=0.37 \
  --set MOTFAIL_UNDER=2.25 --set MOTFAIL_UND_TIME=0.7 \
  --set MOTFAIL_OVER=1.0 --set MOTFAIL_OVR_TIME=2.5
```

Run without the `--set` flags it uses a built-in fallback calibration, which belongs to a different vehicle and will report failures on healthy flights.

The log must contain `esc_status` (per-motor current) and `actuator_motors` (the motor commands).
`vehicle_status` is used to evaluate only the armed part of the flight; without it the tool says so and evaluates everything.
Fields are resolved by name from the log's own format, so one binary reads logs from any PX4 version, with 8 or 12 ESCs, without rebuilding.
A log that is not a ULog, or that is missing a needed field, is reported as an error rather than silently misread.

Output for a healthy flight with a calibrated configuration:

```text
# flight.ulg  (5 phases, 728 armed samples each)
# config (built-in calibration, overrides: MOTFAIL_C2T MOTFAIL_IDLE MOTFAIL_OVER MOTFAIL_OVR_TIME MOTFAIL_UNDER MOTFAIL_UND_TIME):  I_exp = 6.91*u + 0.37 A,  trip if LPF(I-I_exp) < -2.25 for 0.70 s or > 1.00 for 2.50 s
# motor    min[A]    max[A]  trips  verdict
      1     -1.79     +4.20  0/5    ok
      2     -3.09     +2.11  0/5    ok
      3     -1.79     +2.67  0/5    ok
      4     -2.13     +2.38  0/5    ok
      5     -2.35     +2.78  0/5    ok
      6     -2.24     +3.94  0/5    ok
```

| Column    | Meaning                                                                                                                                              |
| --------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| `motor`   | Motor index, counting from 0, as mapped from each ESC's actuator function.                                                                           |
| `min[A]`  | Deepest the filtered residual went below the model, over the whole flight and every sampling phase. This is what the undercurrent band has to clear. |
| `max[A]`  | Highest it went above the model, likewise. This is what the overcurrent band has to clear.                                                           |
| `trips`   | How many of the sampling phases flagged this motor.                                                                                                  |
| `verdict` | `ok`, `FAILED (all phases)`, or `FAILED (sampling-dependent!)` when only some phases flagged it.                                                     |

Motors are numbered from 1, the same as in the `Motor 1 undercurrent detected` message the vehicle reports.

Exit codes make the tool usable as a check in a script: `0` if every motor stayed healthy, `1` if any motor was flagged, `2` if the log could not be used.

### The Sampling Phase Sweep

On the vehicle the check runs at 10 Hz, so it sees only a subset of the ESC samples, and _which_ subset depends on the alignment between the check and the telemetry stream.
That alignment cannot be reconstructed from a log.

The tool therefore runs several detectors over the same log, each starting its 10 Hz grid at a different offset, and reports the spread.
A single-phase answer would be a lower bound on the trip risk, not a verdict.

Read the result as follows:

- `0/5`: the configuration did not trip on this flight, and `min`/`max` say by how much it cleared each band.
- `0/5` but with a note printed below the table: one of the extremes moved by more than 30% between phases, so this run happened not to trip and the same flight could trip on the vehicle.
- `1/5` … `4/5` (`sampling-dependent!`): treat this as a false positive, not as a pass.
- `5/5`: it would have tripped.

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
./build/px4_sitl_default/mfd_replay flight.ulg --set MOTFAIL_UNDER=1.75 --set MOTFAIL_UND_TIME=0.7
```

The accepted names are [MOTFAIL_C2T](../config/motor_failure_detection.md#MOTFAIL_C2T), [MOTFAIL_IDLE](../config/motor_failure_detection.md#MOTFAIL_IDLE), [MOTFAIL_UNDER](../config/motor_failure_detection.md#MOTFAIL_UNDER), [MOTFAIL_UND_TIME](../config/motor_failure_detection.md#MOTFAIL_UND_TIME), [MOTFAIL_OVER](../config/motor_failure_detection.md#MOTFAIL_OVER) and [MOTFAIL_OVR_TIME](../config/motor_failure_detection.md#MOTFAIL_OVR_TIME).

## Choosing the Bands

The goal is the tightest bands that never trip on healthy flight, so the logs have to cover the throttle range the vehicle really flies, including climbs and aggressive manoeuvres: the largest deviation across them is what sets the achievable threshold.
As with the fit, every log has to come from a flight where all motors were healthy.

The steps below use [`batch_replay.sh`](#replaying-a-fleet-of-logs) to replay a whole set of logs at once.

**1. Read the healthy residual.**

Replay each log with the fitted model and the bands set out of the way, so nothing trips and the `min`/`max` columns report the full excursion.
Each log prints its own header and table; one of them:

```bash
for log in ~/logs/healthy/*.ulg; do
  ./build/px4_sitl_default/mfd_replay "$log" \
    --set MOTFAIL_C2T=6.91 --set MOTFAIL_IDLE=0.37 --set MOTFAIL_OVER=999 --set MOTFAIL_UNDER=999
done
```

```text
# motor    min[A]    max[A]  trips  verdict
      1     -1.36     +7.84  0/5    ok
      2     -1.32     +2.72  0/5    ok
      3     -1.85     +1.93  0/5    ok
      4     -1.37     +7.72  0/5    ok
      5     -2.17     +1.11  0/5    ok
      6     -1.55     +1.96  0/5    ok
```

Read the two columns separately, because they set the two sweeps.
The deepest `min` across all logs and motors is what the undercurrent band has to clear, and the highest `max` is what the overcurrent band has to clear.
On this vehicle they are very different, about −2.2 A against +7.8 A, which is the whole reason for two bands.

**2. Sweep each band against its hold time.**

`sweep_matrix.sh` sweeps one direction while parking the other at a value nothing reaches, and prints
how many logs false-trip at each (hold time, band) pair.
The first `0` in a row is the lowest band that hold time supports.

```bash
export C2T=6.91 IDLE=0.37   # the model you fitted
src/lib/motor_failure_detector/tools/sweep_matrix.sh ~/logs/healthy under
src/lib/motor_failure_detector/tools/sweep_matrix.sh ~/logs/healthy over
```

Set the grid from the `min`/`max` you just read, with `HOLDS` and `BANDS`:

- **Bands** run from the healthy noise up to that side's extreme.
  Roughly twice the residual sigma makes a sensible bottom, since below that every log trips.
  On the undercurrent side the top is the expected current at hover: a band above it cannot be reached even by a rotor that has stopped completely.
  The overcurrent side has no such ceiling, so start at its `max` and come down by spending hold time instead of amplitude.
- **Hold times** come from the latency you can accept.
  A latch takes about `0.1 s` of sampling plus `0.2 s` of filter plus the hold, so a fault that destabilises the vehicle wants well under a second, while a thermal fault can wait seconds.

```bash
HOLDS="0.1 0.3 0.5 0.7 1.0" BANDS="1.0 1.25 1.5 1.75 2.0 2.5 3.0" \
  src/lib/motor_failure_detector/tools/sweep_matrix.sh ~/logs/healthy under
```

```text
hold[s]      1.0   1.25    1.5   1.75    2.0    2.5    3.0   <- MOTFAIL_UNDER [A]  (logs tripped)
0.1            3      2      2      2      2      1      0
0.3            2      2      2      2      1      1      0
0.5            2      2      1      1      1      0      0
0.7            2      1      1      0      0      0      0
1.0            1      1      1      0      0      0      0
```

Each row gives one answer.
Walk it from left to right and stop at the first `0`:

| hold  | lowest band with no trips |
| ----- | ------------------------- |
| 0.1 s | 3.0 A                     |
| 0.3 s | 3.0 A                     |
| 0.5 s | 2.5 A                     |
| 0.7 s | 1.75 A                    |
| 1.0 s | 1.75 A                    |

Now compare each floor against this side's ceiling, the expected current at hover, which is about 3.0 A here.

That makes the top two rows unusable rather than merely cautious: the smallest band that avoids false positives is already the band a total failure at hover cannot reach.
The 0.5 s row sits just under the ceiling, which leaves almost nothing for a partial loss.
By 0.7 s the floor has dropped to 1.75 A, which leaves real room below the ceiling, and holding longer gains nothing.
That is what picks 0.7 s here, rather than taste.

**Sweep coarse, then refine.**
The first grid only has to show which zone is interesting; a second, finer pass finds the edge.

The overcurrent side is the example.
Run on the default grid, `sweep_matrix.sh ~/logs/healthy over` puts every zero in the 2.5 s row, which says the floor is 0.5 A or lower and the hold that reaches it is 2.5 s or shorter, but not what either actually is.
Re-running with finer holds and smaller bands:

```bash
HOLDS="1.5 1.75 2.0 2.25 2.5" BANDS="0.2 0.3 0.5 0.75 1.0 1.5 2.0 3.0" \
  src/lib/motor_failure_detector/tools/sweep_matrix.sh ~/logs/healthy over
```

```text
hold[s]      0.2    0.3    0.5   0.75    1.0    1.5    2.0    3.0   <- MOTFAIL_OVER [A]  (logs tripped)
1.5            3      3      2      1      1      1      1      1
1.75           2      1      0      0      0      0      0      0
2.0            2      1      0      0      0      0      0      0
2.25           2      1      0      0      0      0      0      0
2.5            2      1      0      0      0      0      0      0
```

Two things are readable now.
The floor is 0.5 A, since 0.3 A still trips a log.
And it arrives abruptly between 1.5 s and 1.75 s: at 1.5 s even a 3.0 A band trips, at 1.75 s a 0.5 A band is clean, and every longer hold is identical.

That second part is physics rather than a coincidence of the grid.
The benign excursion that sets this side must last between 1.5 s and 1.75 s, so a hold past it rejects the excursion completely while a hold just short of it rejects nothing.

Choosing from a pair of tables like these:

- On the **fault side**, latency decides the hold, and the band takes some margin above the floor.
  How much is your call, and it depends on how much data the floor came from.
  A floor measured on three flights is a sample rather than the worst case, so something like 30% hedges against the next flight going deeper, turning a 1.75 A floor into 2.25 A.
  With hundreds of flights behind it, the measured floor is already close to the real one and you can sit nearer to it.
- On the **nuisance side** there is no ceiling from physics, so the band is set by what you want to catch rather than by what a fault can reach.
  Pick the smallest over-draw worth flagging, find the shortest hold whose floor is below it, then hold a little longer than that.
  The floor drops at whatever hold outlasts the benign excursion, so the shortest hold that works is sitting on its edge.
  The extra hold costs only latency, and these faults are thermal.

::: warning
If you write your own sweep instead of using the script, write it in `bash` and pass the `--set` flags
literally or through an array.
In `zsh` an unquoted variable holding `--set A=1 --set B=2` arrives as a single argument, so the tool
prints a usage error and exits without a verdict table, and a loop that only counts failures reads that
as "nothing tripped".
Assert that a table was produced before trusting a zero, as the script's `grep -q` guard does.
:::

**3. Verify.**

Re-run the whole set of flight logs with the chosen configuration, including logs that were not used for the fit, and confirm every motor is `ok` with no phase-dependent verdicts.
Then check the detection floor the configuration implies: the undercurrent band divided by the per-motor hover current is the smallest current loss that can be seen at hover.

## Replaying a Fleet of Logs

A single flight doesn't usually contain enough information to calculate thresholds, so the usual unit of work is a set of flight logs.
`batch_replay.sh` runs the tool over every `.ulg` in a directory and summarizes the verdicts:

```bash
src/lib/motor_failure_detector/tools/batch_replay.sh ~/logs/healthy --set MOTFAIL_UNDER=1.75 --set MOTFAIL_UND_TIME=0.7
```

```text
OK    flight1.ulg
OK    flight2.ulg
OK    flight3.ulg
----
total=3  OK=3  FAIL=0  SKIP=0
```

Anything after the directory is passed straight through to `mfd_replay`.
`SKIP` counts logs with no usable ESC data.
Run it from the repository root, or set `MFD_REPLAY` to the path of the binary.

::: details Replaying thousands of logs
`batch_replay.sh` works through the logs one at a time, which is fine for tens of them and slow for thousands.
For a large set, run `mfd_replay` itself with one process per log, and give each its own output file so the verdict tables do not interleave:

```bash
mkdir -p /tmp/verdicts
ls ~/logs/healthy/*.ulg | xargs -P"$(nproc)" -I{} \
  sh -c './build/px4_sitl_default/mfd_replay "$1" > /tmp/verdicts/$(basename "$1").txt' _ {}

grep -l FAILED /tmp/verdicts/*.txt   # the logs that tripped
```

This gives the full per-motor table for every log instead of the one-line summary, which is what you want when you then have to look at _why_ something tripped.
:::

## What the Replay Does Not Reproduce

- **The sampling phase**, which is why the sweep exists.
- **Real faults**, unless the log contains one.
  There is no fault injection: to study the true-positive side, replay a log from a flight where a motor really failed, or modify a log so that the current of one motor reflects the fault you want to study.
- **Parameter changes during the log.**
  On the vehicle the detector is reconfigured whenever it is disarmed; the replay configures it once and evaluates only the armed samples.
- **Anything outside the current check.**
  The ESC telemetry timeout, the arming checks and the failure response all live elsewhere in the firmware and are not part of this tool.

## See Also

- [Motor Failure Detection](../config/motor_failure_detection.md) — what the detector does and how it is configured on the vehicle.
- [Flight Log Analysis](../dev_log/flight_log_analysis.md) — general tooling for working with ULog files.
