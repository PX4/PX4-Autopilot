# 着陆模式（多旋翼）

<img src="../../assets/site/position_fixed.svg" title="Position estimate required (e.g. GPS)" width="30px" />

The _Land_ flight mode causes the vehicle to land at the position where the mode was engaged.
这辆车在降落后不久将被解除武装(默认)。

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires at least a valid local position estimate (does not require a global position).
  - Flying vehicles can't switch to this mode without valid local position.
  - Flying vehicles will failsafe if they lose the position estimate.
- Mode prevents arming (vehicle must be armed when switching to this mode).
- 遥控开关可以在任何无人机上更改飞行模式。
- RC stick movement in a multicopter (or VTOL in multicopter mode) will [by default](#COM_RC_OVERRIDE) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless handling a critical battery failsafe.
- The mode can be triggered using the [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) MAVLink command, or by explicitly switching to Land mode.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## 技术总结

无人机将降落在模式所指定的位置。
The vehicle descends at the rate specified in [MPC_LAND_SPEED](#MPC_LAND_SPEED) and will disarm after landing (by [default](#COM_DISARM_LAND)).

RC stick movement will change the vehicle to [Position mode](../flight_modes_mc/position.md) (by [default](#COM_RC_OVERRIDE)).

### 参数

Land mode behaviour can be configured using the parameters below.

| 参数                                                                                                                                                                      | 描述                                                                                                                                                                                                                                         |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="MPC_LAND_SPEED"></a>[MPC_LAND_SPEED](../advanced_config/parameter_reference.md#MPC_LAND_SPEED)                         | 着陆过程中的下降速率。 鉴于地面情况未知，这个值应该设得相当小。                                                                                                                                                                                                           |
| <a id="COM_DISARM_LAND"></a>[COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND)                      | 降落后自动上锁的超时时间，以秒为单位。 如果设定为 -1，无人机将不会在着陆时上锁。                                                                                                                                                                                                 |
| <a id="COM_RC_OVERRIDE"></a>[COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE)                      | Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to [Position mode](../flight_modes_mc/position.md). 可以分别为自动模式和 offboard 模式启用此功能，默认情况下在自动模式下启用此功能。            |
| <a id="COM_RC_STICK_OV"></a>[COM_RC_STICK_OV](../advanced_config/parameter_reference.md#COM_RC_STICK_OV) | The amount of stick movement that causes a transition to [Position mode](../flight_modes_mc/position.md) (if [COM_RC_OVERRIDE](#COM_RC_OVERRIDE) is enabled). |

## See Also

- [Land Mode (FW)](../flight_modes_fw/land.md)
- [Land Mode (VTOL)](../flight_modes_vtol/land.md)
