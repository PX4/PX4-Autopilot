# Guided Course Mode (Fixed-Wing)

<img src="../../assets/site/position_fixed.svg" title="Position required (e.g. GPS)" width="30px" />

_Guided Course mode_ maintains a constant ground track (course), altitude, and airspeed without any manual stick input.
The operator controls the vehicle entirely via [GCS commands](#in-flight-commands), making it the guided equivalent of [Position mode](../flight_modes_fw/position.md).

:::tip
This mode is suited to situations where an operator wants to guide a fixed-wing vehicle from a GCS without manual control.
:::

::: info

- Requires a horizontal velocity estimate (e.g. GPS/dead-reckoning).
  Course commands will be rejected if the velocity estimate is unavailable.
- Manual control input is ignored.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Overview

On activation, the vehicle captures its current velocity over ground vector as the initial course bearing and holds altitude and airspeed from the moment of activation.
The vehicle then flies that course indefinitely until the operator issues a new command.

There is no waypoint sequencing or autonomous path planning: the GCS guides the vehicle in real time by sending individual commands.

## Supported Commands

The following commands are accepted while in Guided Course mode:

| Command                         | Effect                                                                                                                          |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| [MAV_CMD_GUIDED_CHANGE_HEADING] | Set a new course bearing (degrees, 0 = north) with `HEADING_TYPE = 0`. Rejected if horizontal velocity estimate is unavailable. |
| [MAV_CMD_DO_CHANGE_ALTITUDE]    | Set a new target altitude (AMSL, metres).                                                                                       |
| [MAV_CMD_DO_CHANGE_SPEED]       | Set a new equivalent airspeed via param2 (m/s). If param2 ≤ 0, the default cruise speed is restored.                            |

[MAV_CMD_GUIDED_CHANGE_HEADING]: https://mavlink.io/en/messages/common.html#MAV_CMD_GUIDED_CHANGE_HEADING
[MAV_CMD_DO_CHANGE_ALTITUDE]: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_ALTITUDE
[MAV_CMD_DO_CHANGE_SPEED]: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED

## Technical Description

The navigator mode (`course.cpp`) sets a position setpoint with `course` (ground track bearing) and `alt` fields populated, and `yaw = NAN`.

The fixed-wing mode manager (`FixedWingModeManager`) detects the finite `course` field and bypasses normal waypoint sequencing, calling `navigateBearing()` from the directional guidance library to compute lateral acceleration and course setpoints.
Longitudinal control targets the altitude and airspeed from the setpoint.

## Failsafe Behaviour

Guided Course is classified as an `AUTO` mode for failsafe purposes.
The following failsafe exception parameters apply:

| Parameter                                                                  | Bit            | Effect when set                                   |
| -------------------------------------------------------------------------- | -------------- | ------------------------------------------------- |
| [COM_RCL_EXCEPT](../advanced_config/parameter_reference.md#COM_RCL_EXCEPT) | 1 (Auto modes) | RC loss does not trigger a failsafe in this mode. |
| [COM_DLL_EXCEPT](../advanced_config/parameter_reference.md#COM_DLL_EXCEPT) | 1 (Auto modes) | GCS connection loss does not trigger a failsafe.  |

:::warning
Since Guided Course is driven entirely by GCS commands, operators should carefully consider the datalink loss failsafe setting (`COM_DLL_EXCEPT` bit 1).
If the GCS link drops, the vehicle will continue on its last commanded course indefinitely.
It is strongly recommended to either leave the datalink failsafe active or ensure a secondary safety mechanism (e.g. geofence, battery failsafe) is in place.
:::

## Parameters

| Parameter                                                                  | Description                                                                                           |
| -------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| [FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM) | Default cruise airspeed used on activation and when `MAV_CMD_DO_CHANGE_SPEED` is sent with value ≤ 0. |
| [FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN)   | Minimum airspeed. Commanded airspeed is clamped to this value.                                        |
| [FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX)   | Maximum airspeed. Commanded airspeed is clamped to this value.                                        |
| [COM_RCL_EXCEPT](../advanced_config/parameter_reference.md#COM_RCL_EXCEPT) | RC loss failsafe exceptions bitmask. Bit 1 covers all auto modes including Guided Course.             |
| [COM_DLL_EXCEPT](../advanced_config/parameter_reference.md#COM_DLL_EXCEPT) | Datalink loss failsafe exceptions bitmask. Bit 1 covers all auto modes including Guided Course.       |
