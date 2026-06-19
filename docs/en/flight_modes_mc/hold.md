# Hold Mode (Multicopter)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Hold_ flight mode causes the vehicle to stop and hover, maintaining its position and altitude.

:::tip
_Hold mode_ can be used to pause a mission or to help you regain control of a vehicle in an emergency.
It is usually activated with a pre-programmed switch.
:::

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires a global 3d position estimate (from GPS or inferred from a [local position](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Flying vehicles can't switch to this mode without global position.
  - Flying vehicles will failsafe if they lose the position estimate.
  - Disarmed vehicles can switch to mode without valid position estimate but can't arm.
- Mode requires wind and flight time are within allowed limits (specified via parameters).
- RC control switches can be used to change flight modes on any vehicle.
- Stick movement will [by default](#MAN_OVERRIDE_SPD) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless prevented by the active failsafe state.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Technical Summary

The vehicle stops and hovers, maintaining its position and altitude.
The vehicle will first ascend to [NAV_MIN_LTR_ALT](#NAV_MIN_LTR_ALT) if the mode is engaged below this altitude.

Stick movement will change the vehicle to [Position mode](../flight_modes_mc/position.md) (by [default](#MAN_OVERRIDE_SPD)).

### Parameters

Hold mode behaviour can be configured using the parameters below.

| Parameter                                                                                                   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="NAV_MIN_LTR_ALT"></a>[NAV_MIN_LTR_ALT](../advanced_config/parameter_reference.md#NAV_MIN_LTR_ALT)    | This is the minimum altitude above Home the system will always obey in Hold mode if switched into this mode without specifying an altitude (e.g. through switch on RC).                                                                                                                                                                                                                                                                                                                       |
| <a id="MAN_OVERRIDE_SPD"></a>[MAN_OVERRIDE_SPD](../advanced_config/parameter_reference.md#MAN_OVERRIDE_SPD) | Speed (normalized stick travel per second) above which moving the sticks controlling a multicopter (or VTOL in hover) gives control back to the pilot by switching to [Position mode](../flight_modes_mc/position.md) (or Altitude mode if position is unavailable). At the default value of 1 a half-stick movement in ~0.5 s triggers it; lower is more sensitive. A stick held statically has zero speed and will not trigger. Set to -1 to disable. <Badge type="tip" text="PX4 v1.18" /> |
| <a id="MPC_AUTO_RC_YAW"></a>[MPC_AUTO_RC_YAW](../advanced_config/parameter_reference.md#MPC_AUTO_RC_YAW) | Enable yaw stick nudging of vehicle heading in Auto modes (multicopter), held when the stick is released. In Hold this applies only when [MPC_LAND_RC_HELP](#MPC_LAND_RC_HELP) is disabled. Requires stick override to be disabled ([MAN_OVERRIDE_SPD](#MAN_OVERRIDE_SPD) = -1). |
| <a id="MPC_LAND_RC_HELP"></a>[MPC_LAND_RC_HELP](../advanced_config/parameter_reference.md#MPC_LAND_RC_HELP) | Enable stick nudging during the autonomous land routine and in Hold (multicopter). In Hold the heading can be nudged with the yaw stick; when enabled it takes precedence over [MPC_AUTO_RC_YAW](#MPC_AUTO_RC_YAW) for yaw. Requires stick override to be disabled ([MAN_OVERRIDE_SPD](#MAN_OVERRIDE_SPD) = -1). |

<!-- Code for this here: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/loiter.cpp#L61 -->

## See Also

[Hold Mode (FW)](../flight_modes_fw/hold.md)
