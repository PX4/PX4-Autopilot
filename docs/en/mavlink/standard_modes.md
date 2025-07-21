# Standard Modes Protocol (MAVLink)

<Badge type="tip" text="PX4 v1.15" />

PX4 implements the MAVLink [Standard Modes Protocol](https://mavlink.io/en/services/standard_modes.md) from PX4 v1.15, with a corresponding implementation in QGroundControl Daily builds (and future release builds).

The protocol allows you to discover all flight modes available to the vehicle, including PX4 External modes created using the [PX4 ROS 2 Control Interface](../ros2/px4_ros2_control_interface.md), and get or set the current mode.

## Messages & Command Overview

[Standard modes](https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE) are those that are common to most flight stacks with broadly the same behaviour, whereas custom modes are flight-stack specific.

This protocol allows:

- Discovery of all modes supported by a system from both PX4 and ROS 2 ([MAV_CMD_REQUEST_MESSAGE](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE) and [AVAILABLE_MODES](https://mavlink.io/en/messages/common.html#AVAILABLE_MODES)).
- Discovery of the current mode ([CURRENT_MODE](https://mavlink.io/en/messages/common.html#CURRENT_MODE)).
- Setting of standard modes ([MAV_STANDARD_MODE](https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE)) using [MAV_CMD_DO_SET_STANDARD_MODE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_STANDARD_MODE) (recommended).
- Notification when the set of modes changes ([AVAILABLE_MODES_MONITOR](https://mavlink.io/en/messages/common.html#AVAILABLE_MODES_MONITOR))

You can also set custom modes using [SET_MODE](https://mavlink.io/en/messages/common.html#SET_MODE) and custom mode information from `AVAILABLE_MODES`.
At time of writing [MAV_CMD_DO_SET_MODE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE) is not supported.

## Supported Standard Modes

PX4 advertises support for the following standard flight modes, which means that you can start them by calling [MAV_CMD_DO_SET_STANDARD_MODE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_STANDARD_MODE) with the appropriate [MAV_STANDARD_MODE](https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE) enum value.

### Multicopter Standard Modes

MC vehicles support the following standard modes (some of these are supported by all vehicles):

| Standard Mode                                                      | PX4 Mode                                           | Internal Mode |
| ------------------------------------------------------------------ | -------------------------------------------------- | ------------- |
| [MAV_STANDARD_MODE_POSITION_HOLD][MAV_STANDARD_MODE_POSITION_HOLD] | [MC Position mode](../flight_modes_mc/position.md) | POSCTL        |
| [MAV_STANDARD_MODE_ALTITUDE_HOLD][MAV_STANDARD_MODE_ALTITUDE_HOLD] | [MC Altitude mode](../flight_modes_mc/altitude.md) | ALTCTL        |
| [MAV_STANDARD_MODE_ORBIT][MAV_STANDARD_MODE_ORBIT]                 | [FW Orbit mode](../flight_modes_mc/orbit.md)       | POSCTL        |
| [MAV_STANDARD_MODE_SAFE_RECOVERY][MAV_STANDARD_MODE_SAFE_RECOVERY] | [Return mode](../flight_modes/return.md)           | AUTO_RTL      |
| [MAV_STANDARD_MODE_MISSION][MAV_STANDARD_MODE_MISSION]             | [Mission mode](../flight_modes_mc/mission.md)      | AUTO_MISSION  |
| [MAV_STANDARD_MODE_LAND][MAV_STANDARD_MODE_LAND]                   | [Land mode](../flight_modes_mc/land.md)            | AUTO_LAND     |
| [MAV_STANDARD_MODE_TAKEOFF][MAV_STANDARD_MODE_TAKEOFF]             | [Takeoff mode](../flight_modes_mc/takeoff.md)      | AUTO_TAKEOFF  |

[MAV_STANDARD_MODE_POSITION_HOLD]: https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE_POSITION_HOLD
[MAV_STANDARD_MODE_ORBIT]: https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE_ORBIT
[MAV_STANDARD_MODE_ALTITUDE_HOLD]: https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE_ALTITUDE_HOLD

### Fixed Wing Standard Modes

FW vehicles support the following standard modes (some of these are supported by all vehicles):

| Standard Mode                                                      | PX4 Mode                                           | Internal Mode |
| ------------------------------------------------------------------ | -------------------------------------------------- | ------------- |
| [MAV_STANDARD_MODE_CRUISE][MAV_STANDARD_MODE_CRUISE]               | [FW Position mode](../flight_modes_fw/position.md) | POSCTL        |
| [MAV_STANDARD_MODE_ALTITUDE_HOLD][MAV_STANDARD_MODE_ALTITUDE_HOLD] | [FW Altitude mode](../flight_modes_fw/altitude.md) | ALTCTL        |
| [MAV_STANDARD_MODE_ORBIT][MAV_STANDARD_MODE_ORBIT]                 | [FW Hold mode](../flight_modes_fw/hold.md)         | AUTO_LOITER   |
| [MAV_STANDARD_MODE_SAFE_RECOVERY][MAV_STANDARD_MODE_SAFE_RECOVERY] | [Return mode](../flight_modes/return.md)           | AUTO_RTL      |
| [MAV_STANDARD_MODE_MISSION][MAV_STANDARD_MODE_MISSION]             | [Mission mode](../flight_modes_fw/mission.md)      | AUTO_MISSION  |
| [MAV_STANDARD_MODE_LAND][MAV_STANDARD_MODE_LAND]                   | [Land mode](../flight_modes_fw/land.md)            | AUTO_LAND     |
| [MAV_STANDARD_MODE_TAKEOFF][MAV_STANDARD_MODE_TAKEOFF]             | [Takeoff mode](../flight_modes_fw/takeoff.md)      | AUTO_TAKEOFF  |

[MAV_STANDARD_MODE_CRUISE]: https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE_CRUISE

### VTOL Standard Modes

VTOL vehicles support the following standard modes (some of these are supported by all vehicles).
Note that the behaviour of the vehicle depends on whether it is flying as a multicopter or a fixed-wing vehicle.

| Standard Mode                                                      | PX4 Mode                                           | Internal Mode |
| ------------------------------------------------------------------ | -------------------------------------------------- | ------------- |
| [MAV_STANDARD_MODE_ALTITUDE_HOLD][MAV_STANDARD_MODE_ALTITUDE_HOLD] | [FW Altitude mode](../flight_modes_fw/altitude.md) | ALTCTL        |
| [MAV_STANDARD_MODE_SAFE_RECOVERY][MAV_STANDARD_MODE_SAFE_RECOVERY] | [Return mode](../flight_modes/return.md)           | AUTO_RTL      |
| [MAV_STANDARD_MODE_MISSION][MAV_STANDARD_MODE_MISSION]             | [Mission mode](../flight_modes_mc/mission.md)      | AUTO_MISSION  |
| [MAV_STANDARD_MODE_LAND][MAV_STANDARD_MODE_LAND]                   | [Land mode](../flight_modes_mc/land.md)            | AUTO_LAND     |
| [MAV_STANDARD_MODE_TAKEOFF][MAV_STANDARD_MODE_TAKEOFF]             | [Takeoff mode](../flight_modes_mc/takeoff.md)      | AUTO_TAKEOFF  |

::: info
Note that VTOL vehicles could also support `MAV_STANDARD_MODE_CRUISE` (FW) or `MAV_STANDARD_MODE_POSITION_HOLD` (MC) and `MAV_STANDARD_MODE_ORBIT` in respective modes, but this has not been implemented.
:::

### All Vehicles Modes

These standard modes are mapped for all vehicle types.

| Standard Mode                                                      | PX4 Mode                                      | Internal Mode |
| ------------------------------------------------------------------ | --------------------------------------------- | ------------- |
| [MAV_STANDARD_MODE_SAFE_RECOVERY][MAV_STANDARD_MODE_SAFE_RECOVERY] | [Return mode](../flight_modes/return.md)      | AUTO_RTL      |
| [MAV_STANDARD_MODE_MISSION][MAV_STANDARD_MODE_MISSION]             | [Mission mode](../flight_modes_mc/mission.md) | AUTO_MISSION  |
| [MAV_STANDARD_MODE_LAND][MAV_STANDARD_MODE_LAND]                   | [Land mode](../flight_modes_mc/land.md)       | AUTO_LAND     |
| [MAV_STANDARD_MODE_TAKEOFF][MAV_STANDARD_MODE_TAKEOFF]             | [Takeoff mode](../flight_modes_mc/takeoff.md) | AUTO_TAKEOFF  |

[MAV_STANDARD_MODE_SAFE_RECOVERY]: https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE_SAFE_RECOVERY
[MAV_STANDARD_MODE_MISSION]: https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE_MISSION
[MAV_STANDARD_MODE_LAND]: https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE_LAND
[MAV_STANDARD_MODE_TAKEOFF]: https://mavlink.io/en/messages/common.html#MAV_STANDARD_MODE_TAKEOFF

## Standard Mode Implementation

PX4 presents its own custom modes as standard modes, when appropriate.

When implementing a mapping to a standard mode, see [src/lib/modes/standard_modes.hpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/modes/standard_modes.hpp), and in particular the implementation of `getNavStateFromStandardMode()`.

Note that PX4 External modes created using the [PX4 ROS 2 Control Interface](../ros2/px4_ros2_control_interface.md) are also exposed by this interface.

<!--
- How are modes added to available modes - does a developer need to do anything particular when defining a new mode?
- How are their characteristics set?
- How do I notify when the set of modes changes? Do I need to do anything when I create a new mode?

- [PX4-Autopilot#24011: standard_modes: add vehicle-type specific standard modes](https://github.com/PX4/PX4-Autopilot/pull/24011)

-->

## Other MAVLink Mode-changing Commands

Some modes, both standard and custom, can also be set using specific commands and messages.
This can be more convenient that just starting the mode, in particular when the message allows additional settings to be configured.

PX4 supports these commands for changing modes on some vehicle types:

- [MAV_CMD_NAV_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF) — Takeoff mode
- [MAV_CMD_NAV_RETURN_TO_LAUNCH](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RETURN_TO_LAUNCH) — Return mode
- [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) — Land mode
- [MAV_CMD_DO_FOLLOW](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FOLLOW) — MC Follow mode
- [MAV_CMD_DO_FOLLOW_REPOSITION](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FOLLOW_REPOSITION) — Follow mode (from new position)
- [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) — MC Orbit mode
- [MAV_CMD_NAV_VTOL_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_TAKEOFF) — VTOL specific takeoff mode
- [MAV_CMD_DO_REPOSITION](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION)
- [MAV_CMD_DO_PAUSE_CONTINUE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE) — Pauses a mission by putting the vehicle into Hold/Loiter
- [MAV_CMD_MISSION_START](https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START) — Starts a mission.

Note that these commands predate "standard modes" and are mapped to vehicle-specific custom modes.
What that means is that even though the standard mode may be applicable to multiple vehicle types, the mode may only work for particular vehicles.
For example, the Orbit standard mode maps to Orbit mode on MC and Hold/Loiter on FW: but at time of writing the `MAV_CMD_DO_ORBIT` would start orbit mode in MC and be ignored on FW.
