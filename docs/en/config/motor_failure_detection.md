# Motor Failure Detection

<Badge type="tip" text="main (PX4 v2.0)" />

PX4 can detect a motor that has stopped producing the thrust it was commanded to give, as caused by a stalled motor, a lost or broken propeller, a seized bearing, a rubbing blade, and so on.

The mechanism works by comparing the current each ESC reports against the current expected for the command that motor was given.
A motor whose current stays too far from that expectation for long enough is flagged as failed.
The failure is reported to the ground station, and can be [acted on](#failure-action) by removing the motor from the control allocation or by running a failsafe action.

Failure detection is supported for DShot and DroneCAN ESC provided the following [requirements](#requirements) are met.
It is disabled by default ([FD_ACT_EN](#FD_ACT_EN)).

::: warning
The thresholds are airframe specific, and the defaults are not a working configuration.
Values tuned for one vehicle will either false-trip or go blind on another, because both the expected current and the size of the healthy current fluctuations depend on the motors, propellers, ESCs and battery.
Always calibrate on logs from the vehicle you are configuring: see [Choosing Parameter Values](#choosing-parameter-values).
:::

## Requirements

- ESCs that report **per-motor current**, and are configured so that PX4 knows which motor each ESC drives.

  For DShot ESC this needs either a [telemetry wire](../peripherals/dshot.md#esc-telemetry) or [Extended DShot Telemetry](../peripherals/dshot.md#extended-dshot-telemetry-edt).
  [DroneCAN ESC](../dronecan/escs.md) report current as part of their status.
  ESC that report no current, or only report RPM, cannot be checked.

- Motors, not servos: only outputs assigned to a motor function are checked.
- No vehicle-type restriction: a VTOL's multirotor motors and its forward-flight motor are both checked, in every flight mode.
  Only the [failure actions](#failure-action) are multirotor-specific, since they assume the vehicle has rotors to spare.
- Recorded flight logs from the vehicle, to calibrate the model and the thresholds.

## How Detection Works

For every armed motor, the check runs the following four steps at 10 Hz in [commander](../modules/modules_system.md#commander):

1. **Compute expected current.**

   For each motor, the current expected for the command that the control allocator sent to it:

   ```text
   I_expected = MOTFAIL_C2T * u + MOTFAIL_IDLE
   ```

   where `u` is the normalized motor command in the range 0 to 1.
   The _commanded_ signal is used rather than the measured RPM, so the expectation stays valid even when telemetry degrades.

2. **Compute residual.**

   The difference between reported and expected current, `I_measured - I_expected`, is low-pass filtered with a fixed 0.2 s time constant.
   The filter suppresses single noisy samples, so that the thresholds can be set close to the healthy fluctuation band.

3. **Compute two trip bands.**

   The filtered residual fluctuates around zero even on a healthy motor, from noise and model error.
   A trip happens when it leaves that band and stays out, with a separate width and hold time per direction:

   ```text
   undercurrent: LPF(I_measured - I_expected) < -MOTFAIL_UNDER   held for MOTFAIL_UND_TIME
   overcurrent:  LPF(I_measured - I_expected) >  MOTFAIL_OVER    held for MOTFAIL_OVR_TIME
   ```

   Undercurrent means the motor drew less than expected, as a stalled or damaged one would.
   Overcurrent means it drew more, which healthy motors also do during sharp throttle increases.

4. **Latch the fail flag if either band trips.**

   A motor that satisfies either condition is flagged as failed, and reported as `Motor N undercurrent detected` or `Motor N overcurrent detected`.
   The flag is held until the vehicle disarms.
   It is not cleared in flight, because reacting to a motor failure changes what that motor is commanded to do, which removes the evidence the check runs on: a motor that has been switched off would immediately look healthy again.

Special cases:

- A motor is not judged until its ESC has reported a non-zero current at least once, so ESC that only report while spinning do not trip the check on the ground.
- A motor is skipped while its command is `NaN` (the allocation has switched it off), and while it is in reversible mode.
- The filter runs on the ESC sample timestamp, so an interruption in ESC telemetry freezes it instead of integrating stale current.
  A gap longer than 0.3 s resets the filter and cancels a pending trip.
  Complete loss of ESC telemetry is a separate check, enabled with [COM_ARM_CHK_ESCS](../advanced_config/parameter_reference.md#COM_ARM_CHK_ESCS).

::: details Why the Bands Are Not Symmetric

The overcurrent and undercurrent directions do not carry the same information, so a single threshold for both serves neither well.

**The fault side is bounded.**
Current cannot fall below zero, so the largest undercurrent residual that can physically exist is `I_expected` itself.
On a vehicle drawing 8 A per motor in hover, a rotor that stops producing thrust at hover throttle is a residual of at most 8 A, and a partial loss is proportionally smaller.

**The nuisance side is not bounded.**
A fast throttle increase draws acceleration current that a static command-to-current model cannot predict.
The largest residuals seen in perfectly healthy flight are therefore positive, and there is no ceiling on them.

A single band has to clear those positive excursions, and can end up above the largest negative signal a real fault can produce.
That makes a stopped motor structurally undetectable rather than merely slow to detect.
Splitting the bands lets the undercurrent side sit where the faults are, while the overcurrent side stays above the transients.

:::

## Failure Action

Two parameters configure what happens when a failure is detected:

- [CA_FAILURE_MODE](#CA_FAILURE_MODE) selects what the [control allocator](../concept/control_allocation.md) does, such as removing the failed motor from the allocation.
- [COM_ACT_FAIL_ACT](#COM_ACT_FAIL_ACT) selects a failsafe action, in the same way as the other [failsafes](../config/safety.md#motor-failure-trigger).

A quadrotor that loses a rotor cannot hold yaw, so the realistic responses there are landing or termination.
Vehicles with more rotors than the minimum can keep flying with one removed.

## Choosing Parameter Values

There are two things to calibrate, in this order: the current model ([MOTFAIL_C2T](#MOTFAIL_C2T) and [MOTFAIL_IDLE](#MOTFAIL_IDLE)), then the two bands and their hold times ([MOTFAIL_UNDER](#MOTFAIL_UNDER)/[MOTFAIL_UND_TIME](#MOTFAIL_UND_TIME) and [MOTFAIL_OVER](#MOTFAIL_OVER)/[MOTFAIL_OVR_TIME](#MOTFAIL_OVR_TIME)).
The goal is the smallest bands and shortest hold times that still never trip across a set of healthy flights.
That is a tradeoff: a wider band held longer is more robust against false positives; a narrower band held for less time reacts sooner when a motor really does fail.

Both stages are fitted from the vehicle's own logs, which have to come from _healthy_ flights covering the throttle range it really uses, including climbs and aggressive manoeuvres.
Those flights are what set the achievable thresholds.

1. **Find how command maps to current.**

   The model has to say what current a healthy motor draws for a given command, so what you need is the relationship between the commanded value `u` and the reported ESC current, collected over all motors and all logs.
   Use only flights in which every motor was healthy: a log from failure testing, or one with a real fault in it, describes the fault rather than the vehicle and shifts the model away from the rest of the fleet.

   Plotting the two against each other shows that relationship, and fitting a straight line through the points is a convenient way to reduce it to the two numbers the detector wants: the slope of the line goes into [MOTFAIL_C2T](#MOTFAIL_C2T), and its value at zero command into [MOTFAIL_IDLE](#MOTFAIL_IDLE).

   The [`mfd_fit` tool](../debug/motor_failure_replay.md#fitting-the-current-model) does this directly from the logs and prints the two values to set.

   One model is shared by all motors.
   The spread between them is small compared with the trip bands, so nothing is lost by it.

2. **Set the bands from the healthy residual.**

   Use the [Offline Motor Failure Replay Tool](../debug/motor_failure_replay.md) to replay the logs through the detector and find the lowest band that produces no trips anywhere in the set, then add margin on top.

   Around 30% above the lowest false-positive-free value is a reasonable starting point, so a set that first stays clean at 3.0 A might be configured at 4.0 A.

3. **Check the hold time as well as the band.**

   A trip needs the residual to stay outside the band for the whole hold time, so a longer hold allows a lower band.
   How much it allows is very different in the two directions.

   The table below is from 6 healthy flights of one 6-rotor vehicle drawing about 8 A per motor in hover, and gives the lowest band that never trips on any motor of any of those flights:

   Undercurrent:

   | [MOTFAIL_UND_TIME](#MOTFAIL_UND_TIME) | Lowest [MOTFAIL_UNDER](#MOTFAIL_UNDER) |
   | ------------------------------------- | -------------------------------------- |
   | 0.1 s                                 | 3.5 A                                  |
   | 0.3 s                                 | 3.0 A                                  |
   | 0.5 s                                 | 3.0 A                                  |
   | 0.7 s                                 | 3.0 A                                  |
   | 1.0 s                                 | 2.5 A                                  |

   Overcurrent:

   | [MOTFAIL_OVR_TIME](#MOTFAIL_OVR_TIME) | Lowest [MOTFAIL_OVER](#MOTFAIL_OVER) |
   | ------------------------------------- | ------------------------------------ |
   | 0.4 s                                 | 6.0 A                                |
   | 0.7 s                                 | 3.0 A                                |
   | 1.0 s                                 | 2.0 A                                |
   | 1.5 s                                 | 1.5 A                                |
   | 2.5 s                                 | 1.0 A                                |

   The values are specific to that vehicle, but the pattern is general.
   The overcurrent band gains a lot from waiting longer, from 6.0 A at 0.4 s down to 1.0 A at 2.5 s, because the healthy current transients that set it are large but brief.
   The undercurrent band gains much less, 3.5 A down to 2.5 A over ten times the hold, and most of what it gains is already there at 0.3 s, because what limits it is steady model error and waiting does not make that go away.
   A long hold is affordable on the overcurrent side, where the faults are thermal and a few seconds cost nothing, and not on the undercurrent side, where a stopped rotor has to be caught in about a second.

4. **Work out what the configuration can detect.**

   A fault is only visible once it moves the current further than the band, so the undercurrent band and the expected current together set the smallest loss that can be seen, as a fraction of what the motor was drawing:

   ```text
   smallest detectable loss = MOTFAIL_UNDER / I_expected
   ```

   Continuing the example, the 4.0 A band chosen in step 2 against 8 A of hover current is 50%: a rotor losing less than roughly half its current still looks healthy at hover.
   The fraction gets worse as throttle drops, because `I_expected` shrinks with it.
   A complete stop is the same sum with the whole current missing, so it is detectable wherever `I_expected` on its own exceeds the band, which for these numbers is any command above about half the hover command.

   If that floor is too high for the faults you care about, the only way to lower it is a better current model, not a smaller band: the band is already at the smallest width that avoids false trips on healthy motors, so it can't be reduced further.

5. **Verify on the whole set.**

   Include logs that were not used for the fit, and confirm that no motor trips.

::: tip
A _total_ failure is caught far more easily than the floor in step 4 suggests, because it amplifies its own signal.

When a rotor stops producing thrust, the controller commands it harder to recover the missing lift, usually until it saturates.
`I_expected` follows the command up while the real current stays near zero, so the residual keeps growing rather than settling at the value it had at hover.

In one flight test, a rotor that lost its propeller in hover was commanded to full throttle within about 200 ms, which took its residual to nearly 3x the trip band; the failure was flagged 1 s after it happened.

A _partial_ loss gets none of this.
The vehicle still flies, so the command stays near hover and the residual stays small, which is why the floor in step 4 governs partial faults and not total ones.
:::

## Limitations

- **Partial failures near hover are the hard case.**
  The residual is bounded by the expected current, so a small loss at low throttle can be indistinguishable from model error.
- **Calibration is per airframe**, and has to be revisited when motors, propellers, ESCs or battery chemistry change.
- **Only current is used.**
  A motor that draws the expected current but produces no thrust, such as one with a slipping propeller adapter, is not detected.
- **The flag is latched** until disarm, so a fault that recovers stays flagged for the rest of the flight.
- **Detection is not instantaneous.**
  The 10 Hz check rate, the 0.2 s filter and the hold time all add up.
  Expect a few hundred milliseconds at best, and about a second in practice for a complete loss.

## Parameters

| Parameter                                                                                                   | Description                                                                                                                                            |
| ----------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="FD_ACT_EN"></a>[FD_ACT_EN](../advanced_config/parameter_reference.md#FD_ACT_EN)                      | Enable motor failure detection. Disabled by default.                                                                                                   |
| <a id="MOTFAIL_C2T"></a>[MOTFAIL_C2T](../advanced_config/parameter_reference.md#MOTFAIL_C2T)                | Current model slope: expected current per unit of motor command (A).                                                                                   |
| <a id="MOTFAIL_IDLE"></a>[MOTFAIL_IDLE](../advanced_config/parameter_reference.md#MOTFAIL_IDLE)             | Current model offset: expected current at zero command (A).                                                                                            |
| <a id="MOTFAIL_UNDER"></a>[MOTFAIL_UNDER](../advanced_config/parameter_reference.md#MOTFAIL_UNDER)          | Undercurrent trip band (A): how far below the expected current a motor may run.<br>Set to `0` to use [MOTFAIL_OVER](#MOTFAIL_OVER) in both directions. |
| <a id="MOTFAIL_UND_TIME"></a>[MOTFAIL_UND_TIME](../advanced_config/parameter_reference.md#MOTFAIL_UND_TIME) | Time (s) the undercurrent band must be exceeded before the motor is flagged.                                                                           |
| <a id="MOTFAIL_OVER"></a>[MOTFAIL_OVER](../advanced_config/parameter_reference.md#MOTFAIL_OVER)             | Overcurrent trip band (A): how far above the expected current a motor may run.<br>Set to `0` to disable the current check.                             |
| <a id="MOTFAIL_OVR_TIME"></a>[MOTFAIL_OVR_TIME](../advanced_config/parameter_reference.md#MOTFAIL_OVR_TIME) | Time (s) the overcurrent band must be exceeded before the motor is flagged.                                                                            |
| <a id="CA_FAILURE_MODE"></a>[CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE)    | What the control allocator does on a failure, such as removing the failed motor from the allocation.                                                   |
| <a id="COM_ACT_FAIL_ACT"></a>[COM_ACT_FAIL_ACT](../advanced_config/parameter_reference.md#COM_ACT_FAIL_ACT) | Failsafe action on a detected motor failure, such as warning, hold, return, land, or terminate.                                                        |

## Upgrading From PX4 v1.18

In PX4 v1.18 and earlier, the check compared the _instantaneous_ current residual against a single symmetric threshold, `±MOTFAIL_OFF`, held for `MOTFAIL_TIME`.

From PX4 main (v2.0), the residual is low-pass filtered before it is compared, so a sustained deviation is needed rather than isolated samples.
The single threshold is also replaced by two independent bands: [MOTFAIL_UNDER](#MOTFAIL_UNDER)/[MOTFAIL_UND_TIME](#MOTFAIL_UND_TIME) for undercurrent and [MOTFAIL_OVER](#MOTFAIL_OVER)/[MOTFAIL_OVR_TIME](#MOTFAIL_OVR_TIME) for overcurrent.
The current model also gains an offset term ([MOTFAIL_IDLE](#MOTFAIL_IDLE)), and the [offline replay tool](../debug/motor_failure_replay.md) is available to calibrate the new parameters from logs.

None of that changes behaviour on an upgraded vehicle by default: `MOTFAIL_OFF` is renamed to [MOTFAIL_OVER](#MOTFAIL_OVER) and `MOTFAIL_TIME` to [MOTFAIL_OVR_TIME](#MOTFAIL_OVR_TIME) on import, and [MOTFAIL_UNDER](#MOTFAIL_UNDER) defaults to 0.
That selects the single symmetric band, the v1.18 shape of the check, until it is reconfigured.

The thresholds are still worth re-deriving: see [Choosing Parameter Values](#choosing-parameter-values).

## See Also

- [Offline Motor Failure Replay](../debug/motor_failure_replay.md) — replaying recorded logs through the detector to calibrate and verify it.
- [Safety Configuration (Failsafes)](../config/safety.md#motor-failure-trigger) — the failure detector and its other triggers.
- [Control Allocation (Mixing)](../concept/control_allocation.md) — how a removed motor changes the actuator setpoints.
- [Actuator Configuration and Testing](../config/actuators.md) — geometry, motor order and bidirectional motors.
