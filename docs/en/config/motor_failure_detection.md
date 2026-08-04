# Motor Failure Detection

<Badge type="tip" text="main (PX4 v1.19)" /> <Badge type="tip" text="Multicopter" />

PX4 can detect a motor that has stopped producing the thrust it was asked for — a stopped motor, a lost or broken propeller, a seized bearing, a rubbing blade — by comparing the current each ESC reports against the current expected for the command that motor was given.
A motor whose current stays too far from that expectation for long enough is flagged as failed.
The failure is reported to the ground station, and can be [acted on](#failure-response) by removing the motor from the control allocation or by running a failsafe action.

Detection is disabled by default ([FD_ACT_EN](#FD_ACT_EN)).

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
- Recorded flight logs from the vehicle, to calibrate the model and the thresholds.

## How Detection Works

The check runs in [Commander](../modules/modules_system.md#commander) at 10 Hz while the vehicle is armed, once for every motor.

1. **Expected current.**
   For each motor, the current expected for the command that the control allocator sent to it:

   ```text
   I_expected = MOTFAIL_C2T * u + MOTFAIL_IDLE
   ```

   where `u` is the normalized motor command in the range 0 to 1.
   The _commanded_ signal is used rather than the measured RPM, so the expectation stays valid even when telemetry degrades.

2. **Residual.**
   The difference between reported and expected current, `I_measured - I_expected`, is low-pass filtered with a fixed 0.2 s time constant.
   The filter suppresses single noisy samples, so that the thresholds can be set close to the healthy fluctuation band.

3. **Two trip bands.**
   The filtered residual is compared against a separate band in each direction, each with its own hold time:

   ```text
   undercurrent: LPF(I_measured - I_expected) < -MOTFAIL_UNDER   held for MOTFAIL_UND_TIME
   overcurrent:  LPF(I_measured - I_expected) >  MOTFAIL_OVER    held for MOTFAIL_OVR_TIME
   ```

   A motor that satisfies either condition is flagged as failed, and reported as `Motor N undercurrent detected` or `Motor N overcurrent detected`.

4. **Latching.**
   The flag is held until the vehicle disarms.
   It is not cleared in flight, because reacting to a motor failure changes what that motor is commanded to do, which removes the evidence the check runs on: a motor that has been switched off would immediately look healthy again.

Special cases:

- A motor is not judged until its ESC has reported a non-zero current at least once, so ESC that only report while spinning do not trip the check on the ground.
- A motor is skipped while its command is `NaN` (the allocation has switched it off), and while it is in reversible mode.
- The filter runs on the ESC sample timestamp, so an interruption in ESC telemetry freezes it instead of integrating stale current.
  A gap longer than 0.3 s resets the filter and cancels a pending trip.
  Complete loss of ESC telemetry is a separate check, enabled with [COM_ARM_CHK_ESCS](../advanced_config/parameter_reference.md#COM_ARM_CHK_ESCS).

### Why the Bands Are Not Symmetric

The two directions do not carry the same information, so a single threshold for both serves neither well.

**The fault side is bounded.**
Current cannot fall below zero, so the largest undercurrent residual that can physically exist is `I_expected` itself.
On a vehicle drawing 8 A per motor in hover, a rotor that stops producing thrust at hover throttle is a residual of at most 8 A, and a partial loss is proportionally smaller.

**The nuisance side is not bounded.**
A fast throttle increase draws acceleration current that a static command-to-current model cannot predict.
The largest residuals seen in perfectly healthy flight are therefore positive, and there is no ceiling on them.

A single band has to clear those positive excursions, and can end up above the largest negative signal a real fault can produce.
That makes a stopped motor structurally undetectable rather than merely slow to detect.
Splitting the bands lets the undercurrent side sit where the faults are, while the overcurrent side stays above the transients.

### Upgrading From PX4 v1.18

Setting [MOTFAIL_UNDER](#MOTFAIL_UNDER) to 0 selects a single symmetric band: a motor is flagged when the residual leaves `±MOTFAIL_OVER` in either direction for [MOTFAIL_OVR_TIME](#MOTFAIL_OVR_TIME).
That is the shape of the check in PX4 v1.18 and earlier, and it is what an upgraded vehicle keeps.
`MOTFAIL_OFF` is renamed to [MOTFAIL_OVER](#MOTFAIL_OVER) and `MOTFAIL_TIME` to [MOTFAIL_OVR_TIME](#MOTFAIL_OVR_TIME) when the parameters are imported, and [MOTFAIL_UNDER](#MOTFAIL_UNDER) defaults to 0.

The check is not identical to the older one.
The residual is now low-pass filtered before it is compared, so a sustained deviation is needed rather than isolated samples.

## Failure Response

Detection on its own only reports the failure.
The reaction is configured separately:

- [CA_FAILURE_MODE](#CA_FAILURE_MODE) selects what the [control allocator](../concept/control_allocation.md) does, such as removing the failed motor from the allocation.
- [COM_ACT_FAIL_ACT](#COM_ACT_FAIL_ACT) selects a failsafe action, in the same way as the other [failsafes](../config/safety.md).

A multicopter that loses one of four rotors cannot hold yaw, so on a quadrotor the realistic responses are landing or termination.
Vehicles with more rotors than the minimum can keep flying with one removed.

## Choosing Parameter Values

The parameters are fitted from the vehicle's own logs, in two stages: the current model first, then the bands.
Both stages need logs from _healthy_ flights that cover the throttle range the vehicle really uses, including climbs and aggressive manoeuvres.
Those flights are what set the achievable thresholds.

1. **Fit the model.**
   Regress reported ESC current against the commanded value, over all motors and all healthy logs, and use the result for [MOTFAIL_C2T](#MOTFAIL_C2T) and [MOTFAIL_IDLE](#MOTFAIL_IDLE).
   One model is shared by all motors.
   Per-motor fitting is not worth the calibration effort, because the spread between motors is small compared with the trip bands.

2. **Set the bands from the healthy residual.**
   Replay the logs through the detector and find the lowest band that produces no trips anywhere in the set, then add margin on top.
   This is what the [offline replay tool](../debug/motor_failure_replay.md) is for.

   Around 30% above the lowest false-positive-free value is a reasonable starting point, so a set that first stays clean at 3.0 A would be configured at 4.0 A.
   How much margin to use is ultimately a personal choice, and it is the main trade in the whole configuration: a wider band is more robust against false positives, a narrower one reacts sooner when a motor really does fail.

3. **Check the hold time as well as the band.**
   A trip needs the residual to stay outside the band for the whole hold time, so a longer hold allows a lower band.
   How much it allows is very different in the two directions.

   The table below is from 6 healthy flights of one 6-rotor vehicle drawing about 8 A per motor in hover, and gives the lowest band that never trips on any motor of any of those flights:

   | [MOTFAIL_UND_TIME](#MOTFAIL_UND_TIME) | Lowest [MOTFAIL_UNDER](#MOTFAIL_UNDER) | [MOTFAIL_OVR_TIME](#MOTFAIL_OVR_TIME) | Lowest [MOTFAIL_OVER](#MOTFAIL_OVER) |
   | ------------------------------------- | -------------------------------------- | ------------------------------------- | ------------------------------------ |
   | 0.1 s                                 | 3.5 A                                  | 0.4 s                                 | 6.0 A                                |
   | 0.3 s                                 | 3.0 A                                  | 0.7 s                                 | 3.0 A                                |
   | 0.5 s                                 | 3.0 A                                  | 1.0 s                                 | 2.0 A                                |
   | 0.7 s                                 | 3.0 A                                  | 1.5 s                                 | 1.5 A                                |
   | 1.0 s                                 | 2.5 A                                  | 2.5 s                                 | 1.0 A                                |

   The values are specific to that vehicle, but the pattern is general.
   The overcurrent band gains a lot from waiting longer, from 6.0 A at 0.4 s down to 1.0 A at 2.5 s, because the healthy current transients that set it are large but brief.
   The undercurrent band gains much less, 3.5 A down to 2.5 A over ten times the hold, and most of what it gains is already there at 0.3 s, because what limits it is steady model error and waiting does not make that go away.
   A long hold is affordable on the overcurrent side, where the faults are thermal and a few seconds cost nothing, and not on the undercurrent side, where a stopped rotor has to be caught in about a second.

4. **Work out what the configuration can detect.**
   A fault is only visible once it moves the current further than the band, so the undercurrent band and the expected current together set the smallest loss that can be seen:

   ```text
   smallest detectable current loss = MOTFAIL_UNDER / I_expected
   ```

   Continuing the example, the 4.0 A band chosen in step 2 against 8 A of hover current is 50%: a rotor losing less than roughly half its current still looks healthy at hover.
   The fraction gets worse as throttle drops, because `I_expected` shrinks with it.
   A complete stop is the same sum with the whole current missing, so it is detectable wherever `I_expected` on its own exceeds the band — for these numbers, any command above about half the hover command.

   If that floor is too high for the faults you care about, the way down is a better current model rather than a smaller band, because the band cannot go below how far healthy motors already deviate from the model.

5. **Verify on the whole set.**
   Include logs that were not used for the fit, and confirm that no motor trips.

::: tip
A _total_ failure is caught far more easily than the floor in step 4 suggests, because it amplifies its own signal.

When a rotor stops producing thrust, the controller commands it harder to recover the missing lift, usually until it saturates.
`I_expected` follows the command up while the real current stays near zero, so the residual keeps growing rather than settling at the value it had at hover.

On a 6-rotor test vehicle, a rotor that lost its propeller in hover was commanded to full throttle within about 200 ms, which took its residual to nearly 3x the trip band; the failure was flagged 1 s after it happened.

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

## See Also

- [Offline Motor Failure Replay](../debug/motor_failure_replay.md) — replaying recorded logs through the detector to calibrate and verify it.
- [Safety Configuration (Failsafes)](../config/safety.md#motor-failure-trigger) — the failure detector and its other triggers.
- [Control Allocation (Mixing)](../concept/control_allocation.md) — how a removed motor changes the actuator setpoints.
- [Actuator Configuration and Testing](../config/actuators.md) — geometry, motor order and bidirectional motors.
