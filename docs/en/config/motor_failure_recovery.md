# Motor Failure Recovery

<Badge type="tip" text="PX4 v1.18" /> <Badge type="tip" text="Multicopter" />

PX4 can reconfigure [control allocation (mixing)](../concept/control_allocation.md) in flight when a motor failure is detected, so that the vehicle can keep flying on the motors that are left.

The failure action is selected with [CA_FAILURE_MODE](#CA_FAILURE_MODE).
This can be set to `0` (the default) to simply warn the user, or `1` to remove the failed motor from allocation.
Hexarotor frames provide additional recovery options, which are outlined in the following sections.

::: warning
A vehicle flying with a failed motor has less control authority and thrust margin than a healthy one.
Fly gently and land as soon as it is safe to do so.
:::

## Hexarotor

### Stop or Reverse the Opposite Motor

Modes `1` and `2` only differ on a hexarotor; on any other airframe both simply remove the failed motor.

On a hexarotor the rotor opposite the failed one is the rotor that used to cancel its drag (yaw) torque, so what happens to it decides how much yaw authority is left:

- Mode `1` stops it, leaving four symmetric rotors and no yaw bias to trim, but a third of the thrust is gone and the heading can still drift.
- Mode `2` keeps it in the allocation and lets it spin backwards. Driving a rotor in reverse inverts both its thrust and its drag torque, so it can still generate yaw torque. This needs an ESC that can actually reverse the motor.

The opposite motor is taken from the configured [geometry](actuators.md#motor-geometry-multicopter): it is the counter-rotating rotor closest to the failed rotor's antipode, and it is only computed for a 6-rotor multirotor geometry.

### Reverse Thrust Fraction

A propeller spun backwards produces less thrust than it does forwards, and [CA_REV_THR_FRAC](#CA_REV_THR_FRAC) tells the allocator what fraction to expect (default `0.4`, i.e. 40%, which is representative of a standard multicopter propeller).

A value in the right region is best, but it is not critical: a hexarotor still recovers with `1.0` set on a propeller whose real fraction is 0.4.

### ESC Requirements for Reversing

[CA_FAILURE_MODE=2](#CA_FAILURE_MODE) requires that the ESC driving the motor (opposite the failed motor) can spin it backwards.
This requires appropriate firmware and PX4 configuration:

- **DroneCAN:** The ESC must support and be configured for bidirectional operation.
  See [Reversible motors](../dronecan/escs.md#reversible-motors) in _DroneCAN ESC_
- **DShot:** Requires an ESC running in 3D mode.
  A reversible output is encoded using the DShot 3D split range, where neutral sits in the middle of the range.
  3D mode is a persistent ESC setting (see [ESC Commands](../peripherals/dshot.md#commands)), so the vehicle has to be set up to run in 3D mode ([DSHOT_3D_ENABLE](../advanced_config/parameter_reference.md#DSHOT_3D_ENABLE)) for PX4's encoding and the ESC configuration to agree.

PWM, OneShot, and other protocols do not support motor reversal.
If using these you must set [CA_FAILURE_MODE](#CA_FAILURE_MODE)=`1` (or `0`)

::: warning
PX4 does not check whether the ESC can reverse, and a reverse command sent to an ESC that is not configured for it comes out as forward throttle.
Before flying with mode `2`, confirm on the bench that the motor really does spin backwards, for example from the sign of the reported RPM.
:::

## Failure Detection

Recovery reacts to the motor failure flag raised by the [failure detector](safety.md#motor-failure-trigger), which is set when either:

- an ESC stops sending telemetry, or reports a fault.
  This requires [COM_ARM_CHK_ESCS](../advanced_config/parameter_reference.md#COM_ARM_CHK_ESCS) to be enabled.
- the current reported by an ESC is outside the band expected for its commanded thrust.
  This requires [FD_ACT_EN](safety.md#FD_ACT_EN) to be enabled, and the [MOTFAIL\_\*](safety.md#motor-failure-trigger) thresholds to be tuned for the vehicle.

Either path needs ESC telemetry, so recovery is only possible with telemetry-capable ESCs (such as [DroneCAN](../dronecan/escs.md) or [DShot with telemetry](../peripherals/dshot.md#esc-telemetry)).
Note that the two are gated by different parameters: an ESC that goes silent triggers recovery with `FD_ACT_EN` disabled, as long as `COM_ARM_CHK_ESCS` is enabled.

The current-based check **latches**: once a motor is flagged it stays flagged until the vehicle disarms.
This is deliberate, because the recovery stops the failed motor, which would otherwise make it look healthy again and clear the failure.
The ESC offline/fault check does clear if the ESC starts reporting again, in which case all motors are restored to the allocation and any runtime reverse is removed.

To exercise the whole chain you can take an ESC offline with [failure injection](../debug/failure_injection.md): `failure esc off -i <n>`.

## Parameters

| Parameter                                                                                                | Description                                                                                                                                                                                                                                                        |
| -------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="CA_FAILURE_MODE"></a>[CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) | What to do on a single motor failure. <br>`0` (default): Ignore and report failure.<br>`1`: Remove failed motor from allocation. Hexarotor: also stop the opposite motor.<br>`2`: Remove failed motor from allocation. Hexarotor: also reverse the opposite motor. |
| <a id="CA_REV_THR_FRAC"></a>[CA_REV_THR_FRAC](../advanced_config/parameter_reference.md#CA_REV_THR_FRAC) | Fraction of forward thrust that the recovery motor is expected to produce in reverse (default `0.4`). Only used by mode `2`.                                                                                                                                       |

## See Also

- [Safety Configuration (Failsafes)](../config/safety.md), covering the failure detector and motor failure detection.
- [Control Allocation (Mixing)](../concept/control_allocation.md)
- [Actuator Configuration and Testing](actuators.md), covering geometry, motor order and bidirectional motors.
