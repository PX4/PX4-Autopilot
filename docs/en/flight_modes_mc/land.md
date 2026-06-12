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
- RC stick movement in a multicopter (or VTOL in multicopter mode) will [by default](#COM_RC_OVR_SPEED) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless prevented by the active failsafe state.
- The mode can be triggered using the [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) MAVLink command, or by explicitly switching to Land mode.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Technical Summary

The vehicle will land at the location at which the mode was engaged.
The vehicle descends at the rate specified in [MPC_LAND_SPEED](#MPC_LAND_SPEED) and will disarm after landing (by [default](#COM_DISARM_LAND)).

RC stick movement will change the vehicle to [Position mode](../flight_modes_mc/position.md) (by [default](#COM_RC_OVR_SPEED)).

### Parameters

Land mode behaviour can be configured using the parameters below.

| Parameter                                                                                                | Description                                                                                                                                                                                                                                                  |
| -------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="MPC_LAND_SPEED"></a>[MPC_LAND_SPEED](../advanced_config/parameter_reference.md#MPC_LAND_SPEED)    | The rate of descent during landing. This should be kept fairly low as the ground conditions are not known.                                                                                                                                                   |
| <a id="COM_DISARM_LAND"></a>[COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND) | Time-out for auto disarm after landing, in seconds. If set to -1 the vehicle will not disarm on landing.                                                                                                                                                     |
| <a id="COM_RC_OVR_SPEED"></a>[COM_RC_OVR_SPEED](../advanced_config/parameter_reference.md#COM_RC_OVR_SPEED) | Speed (normalized stick travel per second) above which moving the manual control sticks on a multicopter (or VTOL in MC mode) gives control back to the pilot by switching to [Position mode](../flight_modes_mc/position.md) (or Altitude mode if position is unavailable). At the default 1.0 a half-stick movement in ~0.5 s triggers it; lower is more sensitive. A stick held statically has zero speed and will not trigger. Set to 0 to disable manual control override entirely. <Badge type="tip" text="PX4 v1.18" /> |

## See Also

- [Land Mode (FW)](../flight_modes_fw/land.md)
- [Land Mode (VTOL)](../flight_modes_vtol/land.md)
