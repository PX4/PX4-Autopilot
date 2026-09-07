# PX4-Autopilot Main Release Notes

<Badge type="danger" text="Alpha" />

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">This page is on a release branch, and hence probably out of date. <a href="https://docs.px4.io/main/en/releases/main">See the latest version</a>.</p>
  </div>
</div>

This contains changes to the PX4 `main` branch that are not included in the next release ([PX4 v1.18](../releases/1.18.md)).

:::warning
PX4 v1.18 is in beta testing.
Update these notes with features that are going to be in `main` (PX4 v2.0 or later) but not the PX4 v1.18 release.
:::

## Read Before Upgrading

Please continue reading for [upgrade instructions](#upgrade-guide).

## Major Changes

- **[Motor failure recovery for hexarotors](../config/motor_failure_recovery.md).** On a detected single motor failure the control allocator removes the failed motor and, on a hexarotor, additionally stops ([CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) = `1`) or reverses (`2`) the motor opposite it, recovering the yaw authority that is otherwise lost. Reversing keeps the opposite motor in the allocation and needs a reverse-capable ESC; the reverse thrust the allocator expects from a forward propeller is set with [CA_REV_THR_FRAC](../advanced_config/parameter_reference.md#CA_REV_THR_FRAC) (default `0.4`). Disabled by default. ([PX4-Autopilot#28078](https://github.com/PX4/PX4-Autopilot/pull/28078))

## Upgrade Guide

- `COM_ARM_TRAFF` has been replaced by `COM_TRAFF_AVOID`. The old value 3 ("enforce for mission modes only") is migrated to `COM_TRAFF_AVOID=2`, which blocks arming in all modes, not just mission modes. If you relied on being able to arm manually with traffic detected, set `COM_TRAFF_AVOID=1` (warning only) instead.
- **Re-check motor failure handling on hexarotors.** [CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) = `1` now also stops the motor opposite the failed one on a hexarotor (previously only the failed motor was removed from the allocation). Other airframes are unaffected, and `CA_FAILURE_MODE=0` (the default) is unchanged. See [Motor Failure Recovery](../config/motor_failure_recovery.md). ([PX4-Autopilot#28078](https://github.com/PX4/PX4-Autopilot/pull/28078))

## Other changes

- Fast mission Return modes ([RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) = 2 and 4) now skip `DO_JUMP` commands (loops) while following the mission path. ([PX4-Autopilot#26993: fix(navigator): goToNextPositionItem skip loops when required](https://github.com/PX4/PX4-Autopilot/pull/26993))

### Hardware Support

- TBD

### Common

- TBD

### Control

- TBD

### 安全

- [Geofence Aware Return mode](../flight_modes/return.md#geofence_awareness). ([PX4-Autopilot#27145: feat(navigator): Geofence Aware RTL](https://github.com/PX4/PX4-Autopilot/pull/27145), [PX4-Autopilot#28001: docs(navigator): [geofence] added some more warnings about limitations](https://github.com/PX4/PX4-Autopilot/pull/28001)).
- [Flight termination](../advanced_config/flight_termination.md) can now be used instead of a Descent mode as a fallback failsafe mode, allowing safer landing for unpiloted vehicles that carry a parachute.
  See [Battery level failsafe](../config/safety.md#battery-level-failsafe) ([COM_LOW_BAT_ACT](../advanced_config/parameter_reference.md#COM_LOW_BAT_ACT)) and [Position Loss Failsafe Action](../config/safety.md#position-loss-failsafe-action) (new [COM_POS_FS_ACT](../advanced_config/parameter_reference.md#COM_POS_FS_ACT)). ([PX4-Autopilot#28064: feat(commander): add terminate options for critical battery and lost position failsafes](https://github.com/PX4/PX4-Autopilot/pull/28064)).
- [Failure injection](../debug/failure_injection.md) ( [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN)) has been significantly extended. (PX4-Autopilot#27572, PX4-Autopilot#27832, PX4-Autopilot#27950)
  - Now applied on real hardware, not just simulators (injection hooks live in the shared sensor drivers).
  - Command handling is centralized behind a dedicated failure-injection manager module.
  - Multiple sensor instances can be failed simultaneously via a bitmask, and failures can be triggered from an RC switch.
  - Changed default behaviour of injected WRONG failure for Batteries, to publish a wrong level, and not stop publishing
- [Motor failure recovery](../config/motor_failure_recovery.md) for hexarotors: on a single motor failure the control allocator removes the failed motor and additionally stops ([CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) = `1`) or reverses (`2`) the motor opposite it to recover the lost yaw authority. Mode `2` requires a reverse-capable ESC and models the reverse thrust of a forward propeller with the new [CA_REV_THR_FRAC](../advanced_config/parameter_reference.md#CA_REV_THR_FRAC) (default `0.4`). Reversible motor outputs on DroneCAN are now sent as signed `RawCommand` values (negative is reverse). ([PX4-Autopilot#28078](https://github.com/PX4/PX4-Autopilot/pull/28078))
- Added `RTL_TYPE=6` for battery-aware home priority return ([PX4-Autopilot#26968](https://github.com/PX4/PX4-Autopilot/pull/26968)).
  Returns to home if the estimated flight time to home is within the remaining battery time; otherwise returns to the closest rally point.
  Falls back to the closest safe point (home or rally) if battery time remaining is unavailable.

### Estimation

- TBD

### 传感器

- Enable [u-blox Diagnostics with u-center](../gps_compass/u-center.md) while the vehicle's GPS runs as usual. ([PX4-Autopilot#28280](https://github.com/PX4/PX4-Autopilot/pull/28280)).

### 仿真

- TBD

### Debug & Logging

- TBD

### Ethernet

- TBD

### uXRCE-DDS / Zenoh / ROS 2

- TBD

### MAVLink

- TBD

### RC

- TBD

### Multi-Rotor

- TBD

### 垂直起降

- TBD

### Fixed-wing

- TBD

### Rover

- TBD

### ROS 2

- TBD
