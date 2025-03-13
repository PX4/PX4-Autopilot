# Return Mode (Fixed-Wing)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Return_ flight mode is used to _fly a vehicle to safety_ on an unobstructed path to a safe destination, where it can land.

Fixed-wing vehicles use the [Mission Landing/Rally Point](../flight_modes/return.md#mission-landing-rally-point-return-type-rtl-type-1) return type by default, and are expected to always have a mission with a landing pattern.
With this configuration, return mode causes the vehicle to ascend to a minimum safe altitude above obstructions (if needed), fly to the start of the landing pattern defined in the mission plan, and then follow it to land.

Fixed-wing supports the [other PX4 return types](../flight_modes/return.md#return-types-rtl-type), including home/rally point return, mission path and closest safe destination.
The default type is recommended.

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires a global 3d position estimate (from GPS or inferred from a [local position](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Flying vehicles can't switch to this mode without global position.
  - Flying vehicles will failsafe if they lose the position estimate.
- Mode requires home position is set.
- Mode prevents arming (vehicle must be armed when switching to this mode).
- 遥控开关可以在任何无人机上更改飞行模式。
- RC stick movement is ignored.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## 技术总结

Fixed-wing vehicles use the _mission landing/rally point_ return type by default.
无人机在该返航类型中：

- Ascends to a safe minimum return altitude defined by [RTL_RETURN_ALT](#RTL_RETURN_ALT) (safely above any expected obstacles).
  The vehicle maintains its initial altitude if that is higher than the minimum return altitude.
  Note that return altitude cannot be configured using the "cone" parameter in fixed-wing vehicles.
- Flies via direct constant-altitude path to the destination, which will be the closest of the start of a _mission landing pattern_ and any rally point, or the home location if no mission landing pattern or rally points are defined.
- If the destination is a _mission landing pattern_ it will follow the pattern to land.
- If the destination is a rally point or home it will descend to the descent altitude, and then loiter or land (depending on landing parameters).

A fixed-wing vehicle is expected to use a landing pattern defined in a mission as the return destination, as this is the safest way to land automatically.
This requirement is usually enforced by the [MIS_TKO_LAND_REQ](#MIS_TKO_LAND_REQ) parameter.

A mission landing pattern consists of a [MAV_CMD_DO_LAND_START](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START), one or more position waypoints, and a [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND).

When the destination is a rally point or the home location, on arrival the vehicle will rapidly descend to the altitude defined by [RTL_DESCEND_ALT](#RTL_DESCEND_ALT), and by default circle above the destination indefinitely at radius [RTL_LOITER_RAD](#RTL_LOITER_RAD).
The vehicle can be forced to land at the destination by changing [RTL_LAND_DELAY](#RTL_LAND_DELAY) so it is not -1.
In this case the vehicle will land in the same way as [Land mode](../flight_modes_fw/land.md).

## 参数

The RTL parameters are listed in [Parameter Reference > Return Mode](../advanced_config/parameter_reference.md#return-mode).
If using a mission landing, only the [RTL_RETURN_ALT](#RTL_RETURN_ALT) and [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) are relevant.
The others are relevant if the destination is a rally point or the home location.

| 参数                                                                                                                                                                         | 描述                                                                                                                                                                                                                                                                                                                                                             |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RTL_TYPE"></a>[RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE)                                                                   | Return type.                                                                                                                                                                                                                                                                                                                                   |
| <a id="RTL_RETURN_ALT"></a>[RTL_RETURN_ALT](../advanced_config/parameter_reference.md#RTL_RETURN_ALT)                            | Return altitude in meters (default: 60m)If already above this value the vehicle will return at its current altitude.                                                                                                                                                                                        |
| <a id="RTL_DESCEND_ALT"></a>[RTL_DESCEND_ALT](../advanced_config/parameter_reference.md#RTL_DESCEND_ALT)                         | 最小返航高度和无人机从较高的返航高度到减速或者停止的初始下降高度（默认： 30米）。                                                                                                                                                                                                                                                                                                                     |
| <a id="RTL_LAND_DELAY"></a>[RTL_LAND_DELAY](../advanced_config/parameter_reference.md#RTL_LAND_DELAY)                            | Time to hover at `RTL_DESCEND_ALT` before landing (default: 0.5s) -by default this period is short so that the vehicle will simply slow and then land immediately. If set to -1 the system will loiter at `RTL_DESCEND_ALT` rather than landing. 延迟能够使你为起落架的展开部署配置时间（自动触发）。 |
| <a id="RTL_LOITER_RAD"></a>[RTL_LOITER_RAD](../advanced_config/parameter_reference.md#RTL_LOITER_RAD)                            | [Fixed-wing Only] The radius of the loiter circle (at [RTL_LAND_DELAY](#RTL_LAND_DELAY)).                                                                                                                                     |
| <a id="MIS_TKO_LAND_REQ"></a>[MIS_TKO_LAND_REQ](../advanced_config/parameter_reference.md#MIS_TKO_LAND_REQ) | Specify whether a mission landing or takeoff pattern is required. Fixed wings generally require this.                                                                                                                                                                                                                          |

## See Also

- [Return Mode (Generic)](../flight_modes/return.md)
- [Return Mode (Multicopter)](../flight_modes_mc/return.md)
- [Return Mode (VTOL)](../flight_modes_vtol/return.md)
