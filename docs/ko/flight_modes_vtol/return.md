# Return Mode (VTOL)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Return_ flight mode is used to _fly a vehicle to safety_ on an unobstructed path to a safe destination, where it may either wait (hover or circle) or land.

VTOL vehicles use the [Mission Landing/Rally Point](../flight_modes/return.md#mission-landing-rally-point-return-type-rtl-type-1) return type by default.
In this return type a vehicle ascends to a minimum safe altitude above obstructions (if needed), and then flies directly to a rally point or the start of a mission landing point (whichever is nearest), or the home position if neither rally points or mission landing pattern is defined.
If the destination is a mission landing pattern, the vehicle will then follow the pattern to land.
If the destination is a rally point or the home location, the vehicle will fly back to the home position and land.

The vehicle will return using the flying mode (MC or FW) it was using at the point when return mode was triggered.
Generally it will follow the same return mode behaviour of the corresponding vehicle type, but will always transition to MC mode (if needed) before landing.

VTOL supports the [other PX4 return types](../flight_modes/return.md#return-types-rtl-type), including home/rally point return, mission path and closest safe destination.
The default type is recommended.

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires a global 3d position estimate (from GPS or inferred from a [local position](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Flying vehicles can't switch to this mode without global position.
  - Flying vehicles will failsafe if they lose the position estimate.
- Mode requires home position is set.
- Mode prevents arming (vehicle must be armed when switching to this mode).
- RC 제어 스위치는 기체의 비행 모드를 변경할 수 있습니다.
- RC stick movement is ignored.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Technical Summary

VTOL vehicles use the [Mission Landing/Rally Point](../flight_modes/return.md#mission-landing-rally-point-return-type-rtl-type-1) return type by default, and return using the flying mode (MC or FW) it was using at the point when return mode was triggered.

### Fixed-wing Mode (FW) Return

If returning as a fixed-wing, the vehicle:

- Ascends to a safe minimum return altitude defined by [RTL_RETURN_ALT](#RTL_RETURN_ALT) (safely above any expected obstacles).
  The vehicle maintains its initial altitude if that is higher than the minimum return altitude.
  <!-- Note that return altitude cannot be configured using the "cone" parameter in fixed-wing vehicles. -->

- Flies via direct constant-altitude path to the destination, which will be the closest of the start of a _mission landing pattern_ and any rally point, or the home location if no mission landing pattern or rally points are defined.

- 목적지가 임무 착륙 패턴인 경우 패턴을 따라 착륙합니다.

  A mission landing pattern for a VTOL vehicle consists of a [MAV_CMD_DO_LAND_START](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START), one or more position waypoints, and a [MAV_CMD_NAV_VTOL_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND).

- If the destination is a rally point or home it will:

  - Loiter/spiral down to [RTL_DESCEND_ALT](#RTL_DESCEND_ALT).
  - Circle for a short time, as defined by [RTL_LAND_DELAY](#RTL_LAND_DELAY).
  - Yaw towards the destination (centre of loiter).
  - Transition to MC mode and land.

    Note that [NAV_FORCE_VT](../advanced_config/parameter_reference.md#NAV_FORCE_VT) is ignored: the vehicle will always land as a multicopter for these destinations.

## Multicopter Mode (MC) Return

If returning as a multicopter:

- The behaviour is the same except that the vehicle flies as a multicopter and respects multicopter settings.
- In particular, if landing on rally point or the home position the vehicle uses the [RTL_CONE_ANG](#RTL_CONE_ANG) instead of just the [RTL_RETURN_ALT](#RTL_RETURN_ALT) for defining the minimum safe return altitude.
  For more information see the explanation of the "cone" in [Return mode (Generic Vehicle) > Minimum Return Altitude](../flight_modes/return.md#minimum-return-altitude).

## 매개변수

The RTL parameters are listed in [Parameter Reference > Return Mode](../advanced_config/parameter_reference.md#return-mode).
If using a mission landing, only the [RTL_RETURN_ALT](#RTL_RETURN_ALT) and [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) are relevant.
The others are relevant if the destination is a rally point or the home location.

| 매개변수                                                                                                                                                                       | 설명                                                                                                                                                                                                                                                                                                                                                                                                                             |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="RTL_TYPE"></a>[RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE)                                                                   | Return type.                                                                                                                                                                                                                                                                                                                                                                                                   |
| <a id="RTL_RETURN_ALT"></a>[RTL_RETURN_ALT](../advanced_config/parameter_reference.md#RTL_RETURN_ALT)                            | Return altitude in meters (default: 60m)If already above this value the vehicle will return at its current altitude.                                                                                                                                                                                                                                                        |
| <a id="RTL_CONE_ANG"></a>[RTL_CONE_ANG](../advanced_config/parameter_reference.md#RTL_CONE_ANG)                                  | 기체 RTL 리턴 고도를 정의하는 원뿔의 반각. 값 (도) : 0, 25, 45, 65, 80, 90. Note that 0 is "no cone" (always return at `RTL_RETURN_ALT` or higher), while 90 indicates that the vehicle must return at the current altitude or `RTL_DESCEND_ALT` (whichever is higher).                                                 |
| <a id="RTL_DESCEND_ALT"></a>[RTL_DESCEND_ALT](../advanced_config/parameter_reference.md#RTL_DESCEND_ALT)                         | 기체가 더 높은 복귀 고도에서 감속하거나 초기 하강을 중지할 최소 복귀 고도 및 고도 (기본값 : 30m)                                                                                                                                                                                                                                                                                                                                 |
| <a id="RTL_LAND_DELAY"></a>[RTL_LAND_DELAY](../advanced_config/parameter_reference.md#RTL_LAND_DELAY)                            | Time to hover at `RTL_DESCEND_ALT` before landing (default: 0.5s) -by default this period is short so that the vehicle will simply slow and then land immediately. If set to -1 the system will loiter at `RTL_DESCEND_ALT` rather than landing. 이 지연은 랜딩 기어가 배치될 시간을 설정합니다. (자동으로 동작함). |
| <a id="RTL_LOITER_RAD"></a>[RTL_LOITER_RAD](../advanced_config/parameter_reference.md#RTL_LOITER_RAD)                            | [Fixed-wing Only] The radius of the loiter circle (at [RTL_LAND_DELAY](#RTL_LAND_DELAY).                                                                                                                                                                                                      |
| <a id="MIS_TKO_LAND_REQ"></a>[MIS_TKO_LAND_REQ](../advanced_config/parameter_reference.md#MIS_TKO_LAND_REQ) | Specify whether a mission landing or takeoff pattern is _required_. Generally fixed-wing vehicles set this to require a landing pattern but VTOL do not.                                                                                                                                                                                                                                       |

## See Also

- [Return Mode (Generic)](../flight_modes/return.md)
- [Return Mode (Multicopter)](../flight_modes_mc/return.md)
- [Return Mode (Fixed-Wing)](../flight_modes_fw/return.md)
