# Return Mode (Generic Vehicle)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Return_ flight mode is used to _fly a vehicle to safety_ on an unobstructed path to a safe destination, where it should land.

The following topics should be read first if you're using these vehicle types:

- [Multicopter](../flight_modes_mc/return.md)
- [Fixed-wing (Plane)](../flight_modes_fw/return.md)
- [VTOL](../flight_modes_vtol/return.md)

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires a global 3d position estimate (from GPS or inferred from a [local position](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Flying vehicles can't switch to this mode without global position.
  - Flying vehicles will failsafe if they lose the position estimate.
- Mode requires home position is set.
- Mode prevents arming (vehicle cannot be armed while this mode is selected).
- RC switches can be used to change flight modes on any vehicle.
- Stick movement in a multicopter (or VTOL in hover) will [by default](#MAN_OVERRIDE_SPD) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless prevented by the active failsafe state.
- A VTOL will return as MC or FW based on its mode at the point the return mode was triggered.
  In MC mode it will respect multicopter parameters, such as the landing "cone".
  In FW mode it will respect fixed-wing parameters (ignore the cone), but unless using a mission landing, will transition to MC mode and land at the destination after loitering at the descent altitude.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Overview

PX4 provides several mechanisms for choosing a safe return path, destination and landing, including using home location, rally ("safe") points, mission paths, and landing sequences defined in a mission.

All vehicles _nominally_ support all of these mechanisms, but not all of them make as much sense for particular vehicles.
For example, a multicopter can land virtually anywhere, so it doesn't make sense for it to use a landing sequence except in rare cases.
Similarly, a fixed-wing vehicle needs to fly a safe landing path: it can use the home location as a return point, but it won't try and land on it by default.

This topic covers all the possible return types that any vehicle _might_ be configured to use — the vehicle-specific return mode topics cover the default/recommended return type and configuration for each vehicle.

The following sections explain how to configure the [return type](#return_types), [minimum return altitude](#minimum-return-altitude) and [landing/arrival behaviour](#loiter-landing-at-destination).

## Return Types (RTL_TYPE) {#return_types}

PX4 provides four alternative approaches for finding an unobstructed path to a safe destination and/or landing, which are set using the [RTL_TYPE](#RTL_TYPE) parameter.

At high level these are:

- [Home/rally point return](#rtl_type_0) (`RTL_TYPE=0`): Ascend to safe altitude and return via a direct path to the closest rally point or home location.
- [Mission landing/rally point return](#rtl_type_1) (`RTL_TYPE=1`): Ascend to a safe altitude, fly direct to the closest destination _other than home_: rally point or start of mission landing.
  If no mission landing or rally points are defined, return home via direct path.
- [Mission path return](#rtl_type_2) (`RTL_TYPE=2`): Use mission path and fast-continue to mission landing (if defined).
  If no mission _landing_ defined, fast-reverse mission to home.
  If no _mission_ defined, return direct to home (rally points are ignored).
- [Closest safe destination return](#closest-safe-destination-return-type-rtl-type-3) (`RTL_TYPE=3`): Ascend to a safe altitude and return via direct path to closest destination: home, start of mission landing pattern, or rally point.
  If the destination is a mission landing pattern, follow the pattern to land.

More detailed explanations for each of the types are provided in the following sections.

### Home/Rally Point Return Type (RTL_TYPE=0) {#rtl_type_0}

This is the default return type for a [multicopter](../flight_modes_mc/return.md) (see topic for more information).

In this return type the vehicle:

- Ascends to a safe [minimum return altitude](#minimum-return-altitude) (above any expected obstacles).
- Flies to the home position or a rally point (whichever is closest), preferring a [geofence-aware](#geofence_awareness) horizontal path over a direct path where possible.
- On [arrival](#loiter-landing-at-destination) descends to "descent altitude" and waits for a configurable time.
  This time may be used to deploy landing gear.
- Lands or waits (this depends on landing parameters),
  By default an MC or VTOL in MC mode will land and a fixed-wing vehicle circles at the descent altitude.
  A VTOL in FW mode aligns its heading to the destination point, transitions to MC mode, and then lands.

::: info
If no rally points are defined, this is the same as a _Return to Launch_ (RTL)/_Return to Home_ (RTH).
:::

### Mission Landing/Rally Point Return Type (RTL_TYPE=1) {#rtl_type_1}

This is the default return type for a [fixed-wing](../flight_modes_fw/return.md) or [VTOL](../flight_modes_vtol/return.md) vehicle (see topics for more information).

In this return type the vehicle:

- Ascends to a safe [minimum return altitude](#minimum-return-altitude) (above any expected obstacles) if needed.
  The vehicle maintains its initial altitude if that is higher than the minimum return altitude.
- Flies at constant-altitude to a rally point or the start of a [mission landing pattern](#mission-landing-pattern) (whichever is closest), preferring a [geofence-aware](#geofence_awareness) horizontal path over a direct path where possible.
  If no mission landing or rally points are defined the vehicle instead returns home via a direct path.
- If the destination is a mission landing pattern it will follow the pattern to land.
- If the destination is a rally point or home it will [land or wait](#loiter-landing-at-destination) at descent altitude (depending on landing parameters).
  By default an MC or VTOL in MC mode will land, and a fixed-wing vehicle circles at the descent altitude.

  A VTOL in FW mode can first fly a _VTOL approach loiter_ associated with that destination.
  A _VTOL approach loiter_ is a [MAV_CMD_NAV_LOITER_TO_ALT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LOITER_TO_ALT) item that the vehicle uses to descend to the approach altitude.
  For upload details, see [VTOL Fixed-wing Mode Return](../flight_modes_vtol/return.md#fixed-wing-mode-fw-return).
  If several approach loiters are defined for the destination, PX4 chooses the most wind-aligned one. It then flies from the approach loiter to the destination point, transitions to MC mode at the destination, and lands.

::: info
Fixed wing vehicles commonly also set [MIS_TKO_LAND_REQ](#MIS_TKO_LAND_REQ) to _require_ a mission landing pattern.
:::

### Mission Path Return Type (RTL_TYPE=2) {#rtl_type_2}

This return type uses the mission (if defined) to provide a safe return _path_, and the [mission landing pattern](#mission-landing-pattern) (if defined) to provide landing behaviour.
If there is a mission but no mission landing pattern, the mission is flown _in reverse_.
Rally points, if any, are ignored.

::: info
The behaviour is fairly complex because it depends on the flight mode, and whether a mission and mission landing are defined.
:::

::: warning
This return type does have [geofence awareness](#geofence_awareness) (at any stage).
:::

Mission _with_ landing pattern:

- **Mission mode:**
  - Mission is continued in "fast-forward mode" and then lands.
    - DO_JUMP commands, delays and other non-position mission items are ignored, and loiter and other position waypoints are converted to simple waypoints.
- **Auto mode other than mission mode:**
  - Ascend to a safe [minimum return altitude](#minimum-return-altitude) above any expected obstacles.
  - Fly directly to closest waypoint (for FW not a landing WP) and descend to waypoint altitude.
  - Continue mission in fast forward mode from that waypoint, using the same traversal rules as above.
- **Manual modes:**
  - Ascend to a safe [minimum return altitude](#minimum-return-altitude) above any expected obstacles.
  - Fly directly to landing sequence position and descend to waypoint altitude
  - Land using mission landing pattern

Mission _without_ landing pattern defined:

- **Mission mode:**
  - Mission flown "fast-backward" (in reverse) starting from the previous waypoint
    - DO_JUMP commands, delays and other non-position mission items are ignored, and loiter and other position waypoints are converted to simple waypoints.
    - VTOL vehicles transition to FW mode (if needed) before flying the mission in reverse.
  - On reaching waypoint 1, the vehicle ascends to the [minimum return altitude](#minimum-return-altitude) and flies to the home position (where it [lands or waits](#loiter-landing-at-destination)).
- **Auto mode other than mission mode:**
  - Fly directly to closest waypoint (for FW not a landing WP) and descend to waypoint altitude.
  - Continue the mission in reverse, exactly as though Return mode was triggered in mission mode (above)
- **Manual modes:** Fly directly to home location and land.

If no mission is defined PX4 will fly directly to home location and land (rally points are ignored).

If the mission changes during return mode, then the behaviour is re-evaluated based on the new mission following the same rules as above (e.g. if the new mission has no landing sequence and you're in a mission, the mission is reversed).

::: info
For `RTL_TYPE=4`, PX4 currently chooses between continuing to a mission landing and reversing toward home by comparing raw mission item indices.
This is only an approximation of the flown path length, because the number if mission items is not indicative of the distance remaining and non-position items are also counted.
:::

### Closest Safe Destination Return Type (RTL_TYPE=3) {#rtl_type_3}

In this return type the vehicle:

- Ascends to a safe [minimum return altitude](#minimum-return-altitude) (above any expected obstacles).
- Flies a direct path to the closest destination of: home location, mission landing pattern or rally point.
- If the destination is a [mission landing pattern](#mission-landing-pattern) the vehicle will follow the pattern to land.
- If the destination is a home location or rally point, the vehicle will descend to the descent altitude ([RTL_DESCEND_ALT](#RTL_DESCEND_ALT)) and then [lands or waits](#loiter-landing-at-destination).
  By default an MC or VTOL in MC mode will land, and a fixed-wing vehicle circles at the descent altitude.
  A VTOL in FW mode aligns its heading to the destination point, transitions to MC mode, and then lands.

## Geofence Awareness {#geofence_awareness}

<Badge type="tip" text="main (PX4 v1.19)" />

For most of the return types (including the default home/rally point return type) the return path is chosen to avoid breaching any geofence.
Planning is purely horizontal: the altitude profile is unaffected, and only the lateral path is adjusted to avoid the fence.
If no geofence is set, the vehicle flies a direct path to the destination.

While the return mode is inactive, the autopilot constantly recalculates a [shortest horizontal return path](#shortest-path-calculation) that does not enter any exclusion zones and does not exit any inclusion zones.

If the return mode is triggered while the vehicle is violating any geofence, then the vehicle will first fly directly to the most recent recorded location at which it was not violating the geofence.
If no such point exists, or if the autopilot fails to plan a feasible path (e.g. the destination is located in an exclusion zone), then the vehicle falls back to flying directly to the destination.

::: info
The estimated time for return is based on the current shortest horizontal path to the destination and may change if the geofence is updated.
:::

::: warning
There is no absolute guarantee that the vehicle will not breach a geofence on the return path.
Things like path tracking error, wind and other disturbances may cause temporary violation of the geofence, while some of the [geofence limitations](#geofence-limitations) below may disable geofence-aware return altogether.
It is therefore very important to consider this possibility and especially to review the geofence breach action (e.g. [GF_ACTION](../advanced_config/parameter_reference.md#GF_ACTION)).
:::

### RTL-types with Geofence-Awareness

The following table shows which return types currently support geofence awareness:

| Return Type (RTL_TYPE) | Geofence Awareness |
| ---------------------- | ------------------ |
| 0 (home/rally point)   | Yes                |
| 1 (mission landing)    | Yes                |
| 2 (mission path)       | No                 |
| 3 (closest safe dest.) | Yes                |
| 4 (mission path)       | No                 |
| 5 (rally point only)   | Yes                |

### Shortest-Path Calculation

For the construction of the shortest path between the starting location and the destination, the autopilot uses the vertices of the geofence polygons as intermediate waypoints.
In order to avoid the path being too close to the polygon boundaries, the autopilot constructs a corresponding set of polygons, which are either enlarged (for exclusion zones) or shrunk (for inclusion zones).
The margin in both images below is 10m.
The figures below show an exclusion zone and an inclusion zone.

![Exclusion Zone](../../assets/flight_modes/rtl_geofence_exclusion.jpg)

![Inclusion Zone](../../assets/flight_modes/rtl_geofence_inclusion.jpg)

### Geofence Limitations

- Geofence awareness currently supports a maximum of 99 polygon vertices in total (circles count as 8 vertices each).
  If this limit is exceeded, the autopilot falls back to a direct path as described above.
- The autopilot does not check at upload time whether every part of the allowed area is still reachable once the [safety margin](#shortest-path-calculation) has been applied.
  A region can be "cut off" by the margin — for example where several exclusion zones overlap — without any error being reported when the geofence is uploaded.
  The error is only reported when a return is triggered and no feasible path can be found, at which point the vehicle falls back to flying directly to the destination.
  As a result you may believe that every region is reachable when in fact some are not.
- If the autopilot falls back to a direct path (for any of the reasons described above), a configured geofence breach action ([GF_ACTION](../advanced_config/parameter_reference.md#GF_ACTION)) still applies.
  For example, if the direct path breaches a geofence, the configured action (such as flight termination) will be carried out.
- The safety margin in the [Shortest-path Calculation](#shortest-path-calculation) above may not be appropriate for all vehicles.
  It is derived from [NAV_LOITER_RAD](../advanced_config/parameter_reference.md#NAV_LOITER_RAD) for fixed-wing and VTOL vehicles, and from twice [NAV_ACC_RAD](../advanced_config/parameter_reference.md#NAV_ACC_RAD) for multicopters.
  It is not directly configurable.

## Minimum Return Altitude

For most [return types](#return_types) a vehicle will ascend to a _minimum safe altitude_ before returning (unless already above that altitude), in order to avoid any obstacles between it and the destination.

::: info
The exception is when executing a [mission path return](#rtl_type_2) from _within a mission_.
In this case the vehicle follows mission waypoints, which we assume are planned to avoid any obstacles.
:::

The return altitude for a fixed-wing vehicle or a VTOL in fixed-wing mode is configured using the parameter [RTL_RETURN_ALT](#RTL_RETURN_ALT) (does not use the code described in the next paragraph).

The return altitude for a multicopter (or VTOL vehicle in MC mode) is configured using the parameters [RTL_RETURN_ALT](#RTL_RETURN_ALT), [RTL_CONE_ANG](#RTL_CONE_ANG), and [RTL_MIN_DIST](#RTL_MIN_DIST), which together define a half cone centered around the destination (home location or safety point).

![Return mode cone](../../assets/flying/rtl_cone.jpg)

<!-- Original draw.io diagram can be found here: https://drive.google.com/file/d/1W72XeZYSOkRlBSbPXCCiam9NMAyAWSg-/view?usp=sharing -->

If the vehicle is:

- Above [RTL_RETURN_ALT](#RTL_RETURN_ALT) (1) it will return at its current altitude.
- Outside of the radius defined [RTL_MIN_DIST](#RTL_MIN_DIST) (3) it will first climb until it reaches [RTL_RETURN_ALT](#RTL_RETURN_ALT).
- Below the cone and within [RTL_MIN_DIST](#RTL_MIN_DIST) it will climb to return at the cone intersection altitude (2), up to [RTL_RETURN_ALT](#RTL_RETURN_ALT).
- Inside the cone (4, 5) it will return at its current altitude.

Note:

- If [RTL_CONE_ANG](#RTL_CONE_ANG) is 0 degrees there is no "cone":
  - the vehicle returns at `RTL_RETURN_ALT` (or above).
- If [RTL_CONE_ANG](#RTL_CONE_ANG) is 90 degrees the vehicle will generally return at its current altitude when close to the destination. The return altitude may still be constrained to avoid flying too low while approaching the destination.

## Loiter/Landing at Destination

Unless executing a [mission landing pattern](#mission-landing-pattern) as part of the return mode, the vehicle will arrive at its destination, and rapidly descend to the [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) altitude (if above that altitude), where it will loiter for [RTL_LAND_DELAY](#RTL_LAND_DELAY) before landing.
If `RTL_LAND_DELAY=-1` it will loiter indefinitely.

The default landing configuration is vehicle dependent:

- Multicopters are configured to hover for a short while, deploying landing gear if needed, and then land.
- Fixed-wing vehicles use a return mode with a [mission landing pattern](#mission-landing-pattern), as this enables automated landing.
  If not using a mission landing, the default configuration is to loiter indefinitely, so the user can take over and manually land.
- VTOLs in MC mode fly and land exactly as a multicopter.
- VTOLs in FW mode head towards the landing point, transition to MC mode, and then land on the destination.
  If a VTOL approach loiter is defined for a rally point or home location, the vehicle uses that loiter to reach the approach altitude, then flies to the destination before back-transitioning.

## Mission Landing Pattern

A mission landing pattern is a landing pattern defined as part of a mission plan.
This consists of a [MAV_CMD_DO_LAND_START](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START), one or more position waypoints, and a [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) (or [MAV_CMD_NAV_VTOL_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND) for a VTOL Vehicle).

Landing patterns defined in missions are the safest way to automatically land a _fixed-wing_ vehicle on PX4.
For this reason fixed-wing vehicles are configured to use [Mission landing/really point return](#rtl_type_1) by default.

## Parameters

The RTL parameters are listed in [Parameter Reference > Return Mode](../advanced_config/parameter_reference.md#return-mode) (and summarised below).

| Parameter                                                                                                   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| ----------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RTL_TYPE"></a>[RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE)                         | Return mechanism (path and destination).<br>`0`: Return to a rally point or home (whichever is closest) via direct path.<br>`1`: Return to a rally point or the mission landing pattern start point (whichever is closest), via direct path. If neither mission landing or rally points are defined return home via a direct path. If the destination is a mission landing pattern, follow the pattern to land.<br>`2`: Use the mission path to landing while skipping DO_JUMP and other non-position mission items if a landing pattern is defined, otherwise fast-reverse to home with the same traversal rules. Ignore rally points. Fly direct to home if no mission plan is defined.<br>`3`: Return via direct path to closest destination: home, start of mission landing pattern or safe point. If the destination is a mission landing pattern, follow the pattern to land. |
| <a id="RTL_RETURN_ALT"></a>[RTL_RETURN_ALT](../advanced_config/parameter_reference.md#RTL_RETURN_ALT)       | Return altitude in meters (default: 60m) when [RTL_CONE_ANG](../advanced_config/parameter_reference.md#RTL_CONE_ANG) is 0. If already above this value the vehicle will return at its current altitude.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| <a id="RTL_DESCEND_ALT"></a>[RTL_DESCEND_ALT](../advanced_config/parameter_reference.md#RTL_DESCEND_ALT)    | Altitude above the destination used for the final descent before landing or loitering (default: 30m).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| <a id="RTL_LAND_DELAY"></a>[RTL_LAND_DELAY](../advanced_config/parameter_reference.md#RTL_LAND_DELAY)       | Time to wait at `RTL_DESCEND_ALT` before landing (default: 0.5s) - by default this period is short so that the vehicle will simply slow and then land immediately. If set to -1 the system will loiter at `RTL_DESCEND_ALT` rather than landing. The delay is provided to allow you to configure time for landing gear to be deployed (triggered automatically).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| <a id="RTL_MIN_DIST"></a>[RTL_MIN_DIST](../advanced_config/parameter_reference.md#RTL_MIN_DIST)             | Within this distance from the return destination, the return altitude is calculated from the "cone" rather than directly from `RTL_RETURN_ALT`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| <a id="RTL_CONE_ANG"></a>[RTL_CONE_ANG](../advanced_config/parameter_reference.md#RTL_CONE_ANG)             | Half-angle of the cone that defines the vehicle RTL return altitude. Values (in degrees): `0`, `25`, `45`, `65`, `80`, `90`. Note that `0` is "no cone" (always return at `RTL_RETURN_ALT` or higher), while `90` indicates an almost vertical cone, so the vehicle generally returns at its current altitude when close to the destination. The return altitude may still be constrained to avoid flying too low while approaching the destination.                                                                                                                                                                                                                                                                                                                                                                                                                                |
| <a id="RTL_APPR_FORCE"></a>[RTL_APPR_FORCE](../advanced_config/parameter_reference.md#RTL_APPR_FORCE)       | [VTOL FW only] If set, home or rally-point RTL destinations are only considered when a valid VTOL approach loiter is defined for that landing location. Mission landing patterns are unaffected.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| <a id="MAN_OVERRIDE_SPD"></a>[MAN_OVERRIDE_SPD](../advanced_config/parameter_reference.md#MAN_OVERRIDE_SPD) | Speed (normalized stick travel per second) above which moving the sticks controlling a multicopter (or VTOL in hover) gives control back to the pilot by switching to [Position mode](../flight_modes_mc/position.md) (or Altitude mode if position is unavailable). At the default 1 a half-stick movement in ~0.5 s triggers it; lower is more sensitive. A stick held statically has zero speed and will not trigger. Set to -1 to disable. <Badge type="tip" text="PX4 v1.18" />                                                                                                                                                                                                                                                                                                                                                                                                |
| <a id="RTL_LOITER_RAD"></a>[RTL_LOITER_RAD](../advanced_config/parameter_reference.md#RTL_LOITER_RAD)       | [Fixed-wing Only] The radius of the loiter circle (at [RTL_LAND_DELAY](#RTL_LAND_DELAY)).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| <a id="MIS_TKO_LAND_REQ"></a>[MIS_TKO_LAND_REQ](../advanced_config/parameter_reference.md#MIS_TKO_LAND_REQ) | Specify whether a mission landing or takeoff pattern is _required_. Generally fixed-wing vehicles set this to require a landing pattern but VTOL do not.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
