# Land Mode (Multicopter)

<img src="../../assets/site/position_fixed.svg" title="Position estimate required (e.g. GPS)" width="30px" />

The _Land_ flight mode causes the vehicle to land at the position where the mode was engaged.
The vehicle will disarm shortly after landing (by default).

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires at least a valid local position estimate (does not require a global position).
  - Flying vehicles can't switch to this mode without valid local position.
  - Flying vehicles will failsafe if they lose the position estimate.
- Mode prevents arming (vehicle cannot be armed while this mode is selected).
- RC control switches can be used to change flight modes on any vehicle.
- Stick movement in a multicopter (or VTOL in hover) will [by default](#MAN_OVERRIDE_SPD) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless prevented by the active failsafe state.
- The mode can be triggered using the [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) MAVLink command, or by explicitly switching to Land mode.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Technical Summary

The vehicle will land at the location at which the mode was engaged.
The vehicle descends at a rate that slows down as it gets closer to the ground, reaching [MPC_LAND_SPEED](#MPC_LAND_SPEED) for the final part of the descent (see [Landing Slowdown](#landing-slowdown)).
It will disarm after landing (by [default](#COM_DISARM_LAND)).

Stick movement will change the vehicle to [Position mode](../flight_modes_mc/position.md) (by [default](#MAN_OVERRIDE_SPD)).

<!-- AUTO-GENERATED: mode_requirements_rotary_wing_auto_land -->

### Mode Requirements

The following requirements must be met to arm in this mode, or to switch to this mode when it is armed.

- [`mode_req_angular_velocity`](../flight_modes/mode_requirements.md#mode_req_angular_velocity) — Angular velocity
- [`mode_req_attitude`](../flight_modes/mode_requirements.md#mode_req_attitude) — Attitude/pose
- [`mode_req_local_alt`](../flight_modes/mode_requirements.md#mode_req_local_alt) — Local altitude relative to EKF2 origin ('0') position
- [`mode_req_local_position_relaxed`](../flight_modes/mode_requirements.md#mode_req_local_position_relaxed) — Position relative to EKF2 origin ('0') point but accepts poor accuracy
- [`mode_req_prevent_arming`](../flight_modes/mode_requirements.md#mode_req_prevent_arming) — Mode prevents arming

<!-- END AUTO-GENERATED: mode_requirements_rotary_wing_auto_land -->

### Landing Slowdown

During any automatic multicopter landing (Land mode, and the landing phase of [Mission](../flight_modes_mc/mission.md), [Return](../flight_modes_mc/return.md) and [Precision Landing](../advanced_features/precland.md)) the descent speed is limited based on the height of the vehicle above the ground:

- Above [MPC_LAND_ALT1](#MPC_LAND_ALT1) the vehicle descends at up to [MPC_Z_VEL_MAX_DN](../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_DN).
- Between [MPC_LAND_ALT1](#MPC_LAND_ALT1) and [MPC_LAND_ALT2](#MPC_LAND_ALT2) the descent speed limit decreases linearly down to [MPC_LAND_SPEED](#MPC_LAND_SPEED).
- Below [MPC_LAND_ALT2](#MPC_LAND_ALT2) the vehicle descends at [MPC_LAND_SPEED](#MPC_LAND_SPEED).
- Below [MPC_LAND_ALT3](#MPC_LAND_ALT3) the vehicle descends at [MPC_LAND_CRWL](#MPC_LAND_CRWL) (only if a distance sensor is available).

The height above ground is taken from the first available source in the following list:

1. A [distance sensor](../sensor/rangefinders.md), if one is available and its measurement is valid.
2. The [landing target](../advanced_features/precland.md), if its absolute position is valid and recent (for example, from an IR beacon or a vision-based target estimator).
3. The altitude of the landing waypoint, in [Mission](../flight_modes_mc/mission.md) and [Return](../flight_modes_mc/return.md) modes only.
   Land mode does not use this source, because it does not provide a meaningful landing altitude.
4. The home altitude.
5. The altitude of the local position origin.

Without a distance sensor the vehicle assumes that the ground is at the altitude of the selected reference.
If the actual ground is lower than that reference, the vehicle starts slowing down too early and descends slowly for longer.
If the actual ground is higher, the vehicle may reach the ground before it has fully slowed down.
For example, when landing in Land mode at a location that is higher than home, set [MPC_LAND_ALT1](#MPC_LAND_ALT1) and [MPC_LAND_ALT2](#MPC_LAND_ALT2) to account for the height difference, or use a distance sensor.

::: info
If the landing target is lost during the descent, the height above ground falls back to the next source in the list, which may cause a sudden change in descent speed.
:::

### Parameters

Land mode behaviour can be configured using the parameters below.

| Parameter                                                                                                   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MPC_LAND_SPEED"></a>[MPC_LAND_SPEED](../advanced_config/parameter_reference.md#MPC_LAND_SPEED)       | The rate of descent during landing. This should be kept fairly low as the ground conditions are not known.                                                                                                                                                                                                                                                                                                                                                                                    |
| <a id="MPC_LAND_ALT1"></a>[MPC_LAND_ALT1](../advanced_config/parameter_reference.md#MPC_LAND_ALT1)          | Height above ground below which the descent speed starts to be reduced. See [Landing Slowdown](#landing-slowdown).                                                                                                                                                                                                                                                                                                                                                                            |
| <a id="MPC_LAND_ALT2"></a>[MPC_LAND_ALT2](../advanced_config/parameter_reference.md#MPC_LAND_ALT2)          | Height above ground below which the vehicle descends at [MPC_LAND_SPEED](#MPC_LAND_SPEED).                                                                                                                                                                                                                                                                                                                                                                                                    |
| <a id="MPC_LAND_ALT3"></a>[MPC_LAND_ALT3](../advanced_config/parameter_reference.md#MPC_LAND_ALT3)          | Height above ground below which the vehicle descends at [MPC_LAND_CRWL](#MPC_LAND_CRWL). Only used if a distance sensor is available.                                                                                                                                                                                                                                                                                                                                                         |
| <a id="MPC_LAND_CRWL"></a>[MPC_LAND_CRWL](../advanced_config/parameter_reference.md#MPC_LAND_CRWL)          | Descent speed below [MPC_LAND_ALT3](#MPC_LAND_ALT3).                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| <a id="COM_DISARM_LAND"></a>[COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND)    | Time-out for auto disarm after landing, in seconds. If set to -1 the vehicle will not disarm on landing.                                                                                                                                                                                                                                                                                                                                                                                      |
| <a id="MAN_OVERRIDE_SPD"></a>[MAN_OVERRIDE_SPD](../advanced_config/parameter_reference.md#MAN_OVERRIDE_SPD) | Speed (normalized stick travel per second) above which moving the sticks controlling a multicopter (or VTOL in hover) gives control back to the pilot by switching to [Position mode](../flight_modes_mc/position.md) (or Altitude mode if position is unavailable). At the default value of 1 a half-stick movement in ~0.5 s triggers it; lower is more sensitive. A stick held statically has zero speed and will not trigger. Set to -1 to disable. <Badge type="tip" text="PX4 v1.18" /> |
| <a id="MPC_AUTO_NUDGING"></a>[MPC_AUTO_NUDGING](../advanced_config/parameter_reference.md#MPC_AUTO_NUDGING) | Bitmask enabling stick nudging in Auto modes (multicopter). Bit 0 (yaw nudging) lets the yaw stick nudge heading during landing. Bit 1 (land nudging) additionally lets the pitch/roll sticks move the vehicle horizontally (within [MPC_LAND_RADIUS](../advanced_config/parameter_reference.md#MPC_LAND_RADIUS)), the throttle stick amend the descent speed, and the yaw stick rotate the heading. Requires stick override to be disabled ([MAN_OVERRIDE_SPD](#MAN_OVERRIDE_SPD) = -1).     |

## See Also

- [Land Mode (FW)](../flight_modes_fw/land.md)
- [Land Mode (VTOL)](../flight_modes_vtol/land.md)
