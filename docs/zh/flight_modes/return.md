# 返航模式

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Return_ flight mode is used to _fly a vehicle to safety_ on an unobstructed path to a safe destination, where it should land.

PX4 提供了几种机制来选择安全的返航路径，返航目的地和着陆，包括使用其实位置，集结（“安全”）点，任务路径和任务着陆顺序。

- [Multicopter](../flight_modes_mc/return.md)
- [Fixed-wing (Plane)](../flight_modes_fw/return.md)
- [VTOL](../flight_modes_vtol/return.md)

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires a global 3d position estimate (from GPS or inferred from a [local position](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Flying vehicles can't switch to this mode without global position.
  - Flying vehicles will failsafe if they lose the position estimate.
- Mode requires home position is set.
- Mode prevents arming (vehicle must be armed when switching to this mode).
- 遥控开关可以在任何无人机上更改飞行模式。
- RC stick movement in a multicopter (or VTOL in multicopter mode) will [by default](#COM_RC_OVERRIDE) change the vehicle to [Position mode](../flight_modes_mc/position.md) unless handling a critical battery failsafe.
- A VTOL will return as MC or FW based on its mode at the point the return mode was triggered.
  In MC mode it will respect multicopter parameters, such as the landing "cone".
  In FW mode it will respect fixed-wing parameters (ignore the cone), but unless using a mission landing, will transition to MC mode and land at the destination after loitering at the descent altitude.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## 综述

PX4 provides several mechanisms for choosing a safe return path, destination and landing, including using home location, rally ("safe") points, mission paths, and landing sequences defined in a mission.

All vehicles _nominally_ support all of these mechanisms, but not all of them make as much sense for particular vehicles.
For example, a multicopter can land virtually anywhere, so it doesn't make sense for it to use a landing sequence except in rare cases.
Similarly, a fixed-wing vehicle needs to fly a safe landing path: it can use the home location as a return point, but it won't try and land on it by default.

This topic covers all the possible return types that any vehicle _might_ be configured to use — the vehicle-specific return mode topics cover the default/recommended return type and configuration for each vehicle.

The following sections explain how to configure the [return type](#return_types), [minimum return altitude](#minimum-return-altitude) and [landing/arrival behaviour](#loiter-landing-at-destination).

<a id="return_types"></a>

## 返航类型（RTL_TYPE）

PX4 provides four alternative approaches for finding an unobstructed path to a safe destination and/or landing, which are set using the [RTL_TYPE](#RTL_TYPE) parameter.

At high level these are:

- [Home/rally point return](#home_return) (`RTL_TYPE=0`): Ascend to safe altitude and return via a direct path to the closest rally point or home location.
- [Mission landing/rally point return](#mission-landing-rally-point-return-type-rtl-type-1) (`RTL_TYPE=1`): Ascend to a safe altitude, fly direct to the closest destination _other than home_: rally point or start of mission landing.
  如果未定义任务着陆点或集结点，请通过直接路径返回起始位置。
- [Mission path return](#mission-path-return-type-rtl-type-2) (`RTL_TYPE=2`): Use mission path and fast-continue to mission landing (if defined).
  If no mission _landing_ defined, fast-reverse mission to home.
  If no _mission_ defined, return direct to home (rally points are ignored).
- [Closest safe destination return](#closest-safe-destination-return-type-rtl-type-3) (`RTL_TYPE=3`): Ascend to a safe altitude and return via direct path to closest destination: home, start of mission landing pattern, or rally point.
  如果目的地是飞行任务着陆模式，则按照该模式降落。

以下各节提供了每种类型的更详细说明。

<a id="home_return"></a>

### 起始位置/集结点返航类型（RTL_TYPE=0）

This is the default return type for a [multicopter](../flight_modes_mc/return.md) (see topic for more information).

无人机在该返航类型中：

- Ascends to a safe [minimum return altitude](#minimum-return-altitude) (above any expected obstacles).
- 通过直接路径飞往起始位置或集结点（以最近者为准）。
- On [arrival](#loiter-landing-at-destination) descends to "descent altitude" and waits for a configurable time.
  This time may be used to deploy landing gear.
- Lands or waits (this depends on landing parameters),
  By default an MC or VTOL in MC mode will land and a fixed-wing vehicle circles at the descent altitude.
  A VTOL in FW mode aligns its heading to the destination point, transitions to MC mode, and then lands.

:::info
If no rally points are defined, this is the same as a _Return to Launch_ (RTL)/_Return to Home_ (RTH).
:::

### 任务着陆/集结点返航类型 (RTL_TYPE=1)

This is the default return type for a [fixed-wing](../flight_modes_fw/return.md) or [VTOL](../flight_modes_vtol/return.md) vehicle (see topics for more information).

无人机在该返航类型中：

- Ascends to a safe [minimum return altitude](#minimum-return-altitude) (above any expected obstacles) if needed.
  The vehicle maintains its initial altitude if that is higher than the minimum return altitude.
- Flies via direct constant-altitude path to a rally point or the start of a [mission landing pattern](#mission-landing-pattern) (whichever is closest).
  如果未定义任务降落或集结点，无人机通过直接路径返回到起始位置。
- 如果目的地是飞行任务着陆模式，则按照该模式降落。
- If the destination is a rally point or home it will [land or wait](#loiter-landing-at-destination) at descent altitude (depending on landing parameters).
  By default an MC or VTOL in MC mode will land, and a fixed-wing vehicle circles at the descent altitude.
  A VTOL in FW mode aligns its heading to the destination point, transitions to MC mode, and then lands.

:::info
Fixed wing vehicles commonly also set [MIS_TKO_LAND_REQ](#MIS_TKO_LAND_REQ) to _require_ a mission landing pattern.
:::

### 任务路径返航类型（RTL_TYPE=2）

This return type uses the mission (if defined) to provide a safe return _path_, and the [mission landing pattern](#mission-landing-pattern) (if defined) to provide landing behaviour.
If there is a mission but no mission landing pattern, the mission is flown _in reverse_.
集结点，如果有的话，将被忽略。

:::info
The behaviour is fairly complex because it depends on the flight mode, and whether a mission and mission landing are defined.
:::

Mission _with_ landing pattern:

- **Mission mode:** Mission is continued in "fast-forward mode" (jumps, delay and any other non-position commands ignored, loiter and other position waypoints converted to simple waypoints) and then lands.
- **Auto mode other than mission mode:**
  - Ascend to a safe [minimum return altitude](#minimum-return-altitude) above any expected obstacles.
  - 直接飞到最近的航点（对固定翼而言，不是着陆航点），然后降落到航点高度。
  - 从该航点以快速模式继续执行任务。
- **Manual modes:**
  - Ascend to a safe [minimum return altitude](#minimum-return-altitude) above any expected obstacles.
  - 直接飞到降落序列位置并下降到航点高度。
  - 使用任务降落模式着陆。

Mission _without_ landing pattern defined:

- **Mission mode:**
  - 从上一个航点开始以“快退”（反向）飞行的任务
    - 跳，延迟和其他任何非定位命令都会被忽略，悬停和其他位置航点将转换为简单航点。
    - VTOL 无人机在反向飞行任务之前切换到固定翼模式（如果需要）。
  - On reaching waypoint 1, the vehicle ascends to the [minimum return altitude](#minimum-return-altitude) and flies to the home position (where it [lands or waits](#loiter-landing-at-destination)).
- **Auto mode other than mission mode:**
  - 直接飞到最近的航点（对固定翼而言，不是着陆航点），然后降落到航点高度。
  - 反向执行任务，就像在任务模式中触发返航模式一样（上图）。
- **Manual modes:** Fly directly to home location and land.

如果未定义任务，PX4 将直接飞回起始位置并着陆（集结点将被忽略）。

如果任务在返航模式期间发生更改，则将按照与上述相同的规则根据新任务重新评估行为（例如，如果新任务没有降落顺序并且你在一个任务中，则任务将被逆转）。

### 最近的安全目的地返回类型（RTL_TYPE=3）

无人机在该返航类型中：

- Ascends to a safe [minimum return altitude](#minimum-return-altitude) (above any expected obstacles).
- 飞到最近目的地的直接路径：起始位置，任务着陆模式或集结点。
- If the destination is a [mission landing pattern](#mission-landing-pattern) the vehicle will follow the pattern to land.
- If the destination is a home location or rally point, the vehicle will descend to the descent altitude ([RTL_DESCEND_ALT](#RTL_DESCEND_ALT)) and then [lands or waits](#loiter-landing-at-destination).
  By default an MC or VTOL in MC mode will land, and a fixed-wing vehicle circles at the descent altitude.
  A VTOL in FW mode aligns its heading to the destination point, transitions to MC mode, and then lands.

## 悬停/降落在目的地

For most [return types](#return_types) a vehicle will ascend to a _minimum safe altitude_ before returning (unless already above that altitude), in order to avoid any obstacles between it and the destination.

:::info
The exception is when executing a [mission path return](#mission-path-return-type-rtl-type-2) from _within a mission_.
In this case the vehicle follows mission waypoints, which we assume are planned to avoid any obstacles.
:::

The return altitude for a fixed-wing vehicle or a VTOL in fixed-wing mode is configured using the parameter [RTL_RETURN_ALT](#RTL_RETURN_ALT) (does not use the code described in the next paragraph).

The return altitude for a multicopter or a VTOL vehicles in MC mode is configured using the parameters [RTL_RETURN_ALT](#RTL_RETURN_ALT) and [RTL_CONE_ANG](#RTL_CONE_ANG), which define a half cone centered around the destination (home location or safety point).

![Return mode cone](../../assets/flying/rtl_cone.jpg)

<!-- Original draw.io diagram can be found here: https://drive.google.com/file/d/1W72XeZYSOkRlBSbPXCCiam9NMAyAWSg-/view?usp=sharing -->

如果无人机是：

- Above [RTL_RETURN_ALT](#RTL_RETURN_ALT) (1) it will return at its current altitude.
- Below the cone it will return where it intersects the cone (2) or [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) (whichever is higher).
- Outside the cone (3) it will first climb until it reaches [RTL_RETURN_ALT](#RTL_RETURN_ALT).
- 在圆锥内：
  - Above [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) (4) it will return at its current altitude.
  - Below [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) (5) it will first ascend to `RTL_DESCEND_ALT`.

注意：

- If [RTL_CONE_ANG](#RTL_CONE_ANG) is 0 degrees there is no "cone":
  - the vehicle returns at `RTL_RETURN_ALT` (or above).
- If [RTL_CONE_ANG](#RTL_CONE_ANG) is 90 degrees the vehicle will return at the greater of `RTL_DESCEND_ALT` and the current altitude.
- The vehicle will always ascend at least [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) for the return.

## 无人机默认行为

Unless executing a [mission landing pattern](#mission-landing-pattern) as part of the return mode, the vehicle will arrive at its destination, and rapidly descend to the [RTL_DESCEND_ALT](#RTL_DESCEND_ALT) altitude, where it will loiter for [RTL_LAND_DELAY](#RTL_LAND_DELAY) before landing.
If `RTL_LAND_DELAY=-1` it will loiter indefinitely.

The default landing configuration is vehicle dependent:

- 如果定义了任务降落，直接飞往任务降落起点，然后着陆。
- Fixed-wing vehicles use a return mode with a [mission landing pattern](#mission-landing-pattern), as this enables automated landing.
  If not using a mission landing, the default configuration is to loiter indefinitely, so the user can take over and manually land.
- VTOLs in MC mode fly and land exactly as a multicopter.
- VTOLS in FW mode head towards the landing point, transition to MC mode, and then land on the destination.

## Mission Landing Pattern

A mission landing pattern is a landing pattern defined as part of a mission plan.
This consists of a [MAV_CMD_DO_LAND_START](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_LAND_START), one or more position waypoints, and a [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) (or [MAV_CMD_NAV_VTOL_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND) for a VTOL Vehicle).

Landing patterns defined in missions are the safest way to automatically land a _fixed-wing_ vehicle on PX4.
For this reason fixed-wing vehicles are configured to use [Mission landing/really point return](#mission-landing-rally-point-return-type-rtl-type-1) by default.

## 参数

The RTL parameters are listed in [Parameter Reference > Return Mode](../advanced_config/parameter_reference.md#return-mode) (and summarised below).

| 参数                                                                                                                                                                         | 描述                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RTL_TYPE"></a>[RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE)                                                                   | Return mechanism (path and destination).<br>`0`: Return to a rally point or home (whichever is closest) via direct path.<br>`1`: Return to a rally point or the mission landing pattern start point (whichever is closest), via direct path. 如果未定义任务着陆点或集结点，通过直接路径返回起始位置。 If the destination is a mission landing pattern, follow the pattern to land.<br>`2`: Use mission path fast-forward to landing if a landing pattern is defined, otherwise fast-reverse to home. 忽略集结点。 Fly direct to home if no mission plan is defined.<br>`3`: Return via direct path to closest destination: home, start of mission landing pattern or safe point. 如果目的地是飞行任务着陆模式，则按照该模式降落。 |
| <a id="RTL_RETURN_ALT"></a>[RTL_RETURN_ALT](../advanced_config/parameter_reference.md#RTL_RETURN_ALT)                            | Return altitude in meters (default: 60m) when [RTL_CONE_ANG](../advanced_config/parameter_reference.md#RTL_CONE_ANG) is 0. 如果已经超过这个值, 飞机将返回当前的高度。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| <a id="RTL_DESCEND_ALT"></a>[RTL_DESCEND_ALT](../advanced_config/parameter_reference.md#RTL_DESCEND_ALT)                         | 最小返航高度和无人机从较高的返航高度到减速或者停止的初始下降高度（默认： 30米）。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| <a id="RTL_LAND_DELAY"></a>[RTL_LAND_DELAY](../advanced_config/parameter_reference.md#RTL_LAND_DELAY)                            | Time to wait at `RTL_DESCEND_ALT` before landing (default: 0.5s) -by default this period is short so that the vehicle will simply slow and then land immediately. If set to -1 the system will loiter at `RTL_DESCEND_ALT` rather than landing. 延迟能够使你为起落架的展开部署配置时间（自动触发）。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| <a id="RTL_MIN_DIST"></a>[RTL_MIN_DIST](../advanced_config/parameter_reference.md#RTL_MIN_DIST)                                  | 能够触发无人机上升到返航高度，距离起始位置的最小水平距离由那个"锥形"指定。 If the vehicle is horizontally closer than this distance to home, it will return at its current altitude or `RTL_DESCEND_ALT` (whichever is higher) instead of first ascending to RTL_RETURN_ALT.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| <a id="RTL_CONE_ANG"></a>[RTL_CONE_ANG](../advanced_config/parameter_reference.md#RTL_CONE_ANG)                                  | 圆锥半角决定无人机的 RTL 返航高度。 数值(度数)：0、25、45、65、80、90。 Note that 0 is "no cone" (always return at `RTL_RETURN_ALT` or higher), while 90 indicates that the vehicle must return at the current altitude or `RTL_DESCEND_ALT` (whichever is higher).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| <a id="COM_RC_OVERRIDE"></a>[COM_RC_OVERRIDE](../advanced_config/parameter_reference.md#COM_RC_OVERRIDE)                         | Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to [Position mode](../flight_modes_mc/position.md) (except when vehicle is handling a critical battery failsafe). 可以分别为自动模式和 offboard 模式启用此功能，默认情况下在自动模式下启用此功能。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| <a id="COM_RC_STICK_OV"></a>[COM_RC_STICK_OV](../advanced_config/parameter_reference.md#COM_RC_STICK_OV)    | The amount of stick movement that causes a transition to [Position mode](../flight_modes_mc/position.md) (if [COM_RC_OVERRIDE](#COM_RC_OVERRIDE) is enabled).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| <a id="RTL_LOITER_RAD"></a>[RTL_LOITER_RAD](../advanced_config/parameter_reference.md#RTL_LOITER_RAD)                            | [Fixed-wing Only] The radius of the loiter circle (at [RTL_LAND_DELAY](#RTL_LAND_DELAY)).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| <a id="MIS_TKO_LAND_REQ"></a>[MIS_TKO_LAND_REQ](../advanced_config/parameter_reference.md#MIS_TKO_LAND_REQ) | Specify whether a mission landing or takeoff pattern is _required_. Generally fixed-wing vehicles set this to require a landing pattern but VTOL do not.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
