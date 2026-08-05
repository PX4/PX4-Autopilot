# Return Mode (VTOL)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Return_ flight mode is used to _fly a vehicle to safety_ on an unobstructed path to a safe destination, where it may either wait (hover or circle) or land.

VTOL vehicles use the [Mission Landing/Rally Point](../flight_modes/return.md#rtl_type_1) return type by default.
In this return type a vehicle ascends to a minimum safe altitude above obstructions (if needed), and then flies directly to a rally point or the start of a mission landing point (whichever is nearest), or the home position if neither rally points or mission landing pattern is defined.
If the destination is a mission landing pattern, the vehicle will then follow the pattern to land.
If the destination is a rally point or the home location, the vehicle will fly to that destination and land.

The vehicle will return using the flying mode (MC or FW) it was using at the point when return mode was triggered.
Generally it will follow the same return mode behaviour of the corresponding vehicle type, but will always transition to MC mode (if needed) before landing.

VTOL supports the [other PX4 return types](../flight_modes/return.md#return_types), including home/rally point return, mission path and closest safe destination.
The default type is recommended.

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires a global 3d position estimate (from GPS or inferred from a [local position](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Flying vehicles can't switch to this mode without global position.
  - Flying vehicles will failsafe if they lose the position estimate.
- Mode requires home position is set.
- Mode prevents arming (vehicle cannot be armed while this mode is selected).
- RC control switches can be used to change flight modes on any vehicle.
- RC stick movement is ignored.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Technical Summary

VTOL vehicles use the [Mission Landing/Rally Point](../flight_modes/return.md#rtl_type_1) return type by default, and return using the flying mode (MC or FW) it was using at the point when return mode was triggered.

### Fixed-wing Mode (FW) Return

If returning as a fixed-wing, the vehicle:

- Ascends to a safe minimum return altitude defined by [RTL_RETURN_ALT](#RTL_RETURN_ALT) (safely above any expected obstacles).
  The vehicle maintains its initial altitude if that is higher than the minimum return altitude.
  <!-- Note that return altitude cannot be configured using the "cone" parameter in fixed-wing vehicles. -->
- Flies via a constant-altitude path to the destination, which will be the closest of the start of a _mission landing pattern_ and any rally point, or the home location if no mission landing pattern or rally points are defined.
  The path is chosen to be the shortest horizontal [geofence-aware path](../flight_modes/return.md#geofence_awareness).
- If the destination is a mission landing pattern it will follow the pattern to land.

  A mission landing pattern for a VTOL vehicle consists of a [MAV_CMD_DO_LAND_START](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START), one or more position waypoints, and a [MAV_CMD_NAV_VTOL_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND).

- If the destination is a rally point or home it will:
  - Fly to the selected [VTOL approach loiter](#vtol-rally-point-approach-loiters) associated with that landing location (if any are defined) and use it to descend to the approach altitude.
    If several approach loiters are defined for that location, PX4 chooses the one that best matches the estimated wind at the landing point.

    See [VTOL Rally Point Approach Loiter](#vtol-rally-point-approach-loiters) below for information on how to define approach loiters (using [MAV_CMD_NAV_LOITER_TO_ALT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TO_ALT) items in your _rally plan_).

  - Loiter/spiral down to the approach altitude, or to [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) above the destination if no approach altitude is defined.
  - Circle for a short time, as defined by [RTL_LAND_DELAY](#RTL_LAND_DELAY).
  - Fly from the approach loiter to the return destination (the rally point or home location).
  - Transition to MC mode at the destination and land.

    Note that [NAV_FORCE_VT](../advanced_config/parameter_reference.md#NAV_FORCE_VT) is ignored: the vehicle will always land as a multicopter for these destinations.

#### VTOL Rally Point Approach Loiters

VTOL _rally point approach loiters_ are [MAV_CMD_NAV_LOITER_TO_ALT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TO_ALT) items associated with a particular rally point ([MAV_CMD_NAV_RALLY_POINT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RALLY_POINT)) in a [Rally plan](../flying/plan_safety_points.md).
They define options for where the VTOL can descend to the rally point approach altitude, and how it will approach the rally point.
Several approach loiters can be defined for a rally point, and PX4 will choose the one that best matches the estimated wind at the landing point.

::: tip
The `MAV_CMD_NAV_LOITER_TO_ALT` items that define approach loiters are associated with a Rally/safety point, and are hence part of the rally point plan when used in this way — _not_ the mission plan.
This behaviour is not defined in the MAVLink rally point plan specification.
:::

When uploading VTOL approach loiters through MAVLink, [upload them as rally/safe-point](https://mavlink.io/en/services/mission.html#mission_types) mission items (`MAV_MISSION_TYPE_RALLY`).
The `MAV_CMD_NAV_RALLY_POINT` item must come first, followed by one or more `MAV_CMD_NAV_LOITER_TO_ALT` items that define the approach loiters for that rally point.
The next `MAV_CMD_NAV_RALLY_POINT` starts a new landing-location block.

For each `MAV_CMD_NAV_LOITER_TO_ALT` item, `x/y/z` define the loiter center and approach altitude, and `param2` defines the loiter radius used by RTL.
If `param2` is unset or zero, PX4 falls back to [RTL_LOITER_RAD](#RTL_LOITER_RAD).

For example, a rally upload with one rally point and two possible approach loiters would use:

| Sequence | Command                                                                                           | Purpose                     | Key fields                                                             |
| -------- | ------------------------------------------------------------------------------------------------- | --------------------------- | ---------------------------------------------------------------------- |
| 0        | [MAV_CMD_NAV_RALLY_POINT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RALLY_POINT)     | Landing location            | `x/y/z`: rally latitude, longitude, altitude                           |
| 1        | [MAV_CMD_NAV_LOITER_TO_ALT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TO_ALT) | First VTOL approach loiter  | `x/y/z`: loiter latitude, longitude, altitude; `param2`: loiter radius |
| 2        | [MAV_CMD_NAV_LOITER_TO_ALT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TO_ALT) | Second VTOL approach loiter | `x/y/z`: loiter latitude, longitude, altitude; `param2`: loiter radius |

Note that the approach loiter is not the back-transition point.
If the selected approach loiter is far from the rally point or home location, the vehicle remains in fixed-wing mode after the loiter, flies to the destination at the approach altitude, and only then back-transitions for landing.

## Multicopter Mode (MC) Return

If returning as a multicopter:

- The behaviour is the same except that the vehicle flies as a multicopter and respects multicopter settings.
- In particular, if landing on rally point or the home position the vehicle uses the [RTL_CONE_ANG](#RTL_CONE_ANG) instead of just the [RTL_RETURN_ALT](#RTL_RETURN_ALT) for defining the minimum safe return altitude.
  For more information see the explanation of the "cone" in [Return mode (Generic Vehicle) > Minimum Return Altitude](../flight_modes/return.md#minimum-return-altitude).

## Parameters

The RTL parameters are listed in [Parameter Reference > Return Mode](../advanced_config/parameter_reference.md#return-mode).
If using a mission landing, only the [RTL_RETURN_ALT](#RTL_RETURN_ALT) and [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) are relevant.
The others are relevant if the destination is a rally point or the home location.

| Parameter                                                                                                   | Description                                                                                                                                                                                                                                                                                                                                                      |
| ----------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RTL_TYPE"></a>[RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE)                         | Return type.                                                                                                                                                                                                                                                                                                                                                     |
| <a id="RTL_APPR_FORCE"></a>[RTL_APPR_FORCE](../advanced_config/parameter_reference.md#RTL_APPR_FORCE)       | [VTOL FW only] If set, PX4 only considers home or rally-point RTL destinations when a valid VTOL approach loiter is defined for that landing location. Mission landing patterns are unaffected.                                                                                                                                                                  |
| <a id="RTL_RETURN_ALT"></a>[RTL_RETURN_ALT](../advanced_config/parameter_reference.md#RTL_RETURN_ALT)       | Return altitude in meters (default: 60m)If already above this value the vehicle will return at its current altitude.                                                                                                                                                                                                                                             |
| <a id="RTL_CONE_ANG"></a>[RTL_CONE_ANG](../advanced_config/parameter_reference.md#RTL_CONE_ANG)             | Half-angle of the cone that defines the vehicle RTL return altitude. Values (in degrees): 0, 25, 45, 65, 80, 90. Note that 0 is "no cone" (always return at `RTL_RETURN_ALT` or higher), while 90 indicates that the vehicle must return at the current altitude or `RTL_DESCEND_ALT` (whichever is higher).                                                     |
| <a id="RTL_DESCEND_ALT"></a>[RTL_DESCEND_ALT](../advanced_config/parameter_reference.md#RTL_DESCEND_ALT)    | Minimum return altitude and altitude at which the vehicle will slow or stop its initial descent from a higher return altitude (default: 30m)                                                                                                                                                                                                                     |
| <a id="RTL_LAND_DELAY"></a>[RTL_LAND_DELAY](../advanced_config/parameter_reference.md#RTL_LAND_DELAY)       | Time to hover at `RTL_DESCEND_ALT` before landing (default: 0.5s) -by default this period is short so that the vehicle will simply slow and then land immediately. If set to -1 the system will loiter at `RTL_DESCEND_ALT` rather than landing. The delay is provided to allow you to configure time for landing gear to be deployed (triggered automatically). |
| <a id="RTL_LOITER_RAD"></a>[RTL_LOITER_RAD](../advanced_config/parameter_reference.md#RTL_LOITER_RAD)       | [Fixed-wing Only] The radius of the loiter circle (at [RTL_LAND_DELAY](#RTL_LAND_DELAY).                                                                                                                                                                                                                                                                         |
| <a id="MIS_TKO_LAND_REQ"></a>[MIS_TKO_LAND_REQ](../advanced_config/parameter_reference.md#MIS_TKO_LAND_REQ) | Specify whether a mission landing or takeoff pattern is _required_. Generally fixed-wing vehicles set this to require a landing pattern but VTOL do not.                                                                                                                                                                                                         |

## See Also

- [Return Mode (Generic)](../flight_modes/return.md)
- [Return Mode (Multicopter)](../flight_modes_mc/return.md)
- [Return Mode (Fixed-Wing)](../flight_modes_fw/return.md)
