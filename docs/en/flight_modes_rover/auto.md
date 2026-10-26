# Auto Modes

In auto modes the autopilot takes over control of the vehicle to run missions, return to launch, or perform other autonomous navigation tasks.

To use auto modes **all** the configuration/tuning steps in [Rover Configuration/Tuning](../config_rover/index.md) must be followed (from [Basic Setup](../config_rover/basic_setup.md) to [Position tuning](../config_rover/position_tuning.md)).

## Mission Mode

_Mission mode_ is an automatic mode that causes the vehicle to execute a predefined autonomous [mission plan](../flying/missions.md) that has been uploaded to the flight controller.
The mission is typically created and uploaded with a Ground Control Station (GCS) application, such as [QGroundControl](https://docs.qgroundcontrol.com/master/en/).

### Mission commands

The following commands can be used in missions at time of writing (PX4 v1.16):

| QGC mission item    | Command                                     | Description                                       |
| ------------------- | ------------------------------------------- | ------------------------------------------------- |
| Mission start       | [MAV_CMD_MISSION_START]                     | Starts the mission.                               |
| Waypoint            | [MAV_CMD_NAV_WAYPOINT]                      | Navigate to waypoint.                             |
| Return to launch    | [MAV_CMD_NAV_RETURN_TO_LAUNCH]              | Return to the launch location.                    |
| Change speed        | [MAV_CMD_DO_CHANGE_SPEED]                   | Change the speed setpoint                         |
| Set launch location | [MAV_CMD_DO_SET_HOME]                       | Changes launch location to specified coordinates. |
| Jump to item (all)  | [MAV_CMD_DO_JUMP] (and other jump commands) | Jump to specified mission item.                   |

[MAV_CMD_MISSION_START]: https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START
[MAV_CMD_NAV_WAYPOINT]: https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
[MAV_CMD_NAV_RETURN_TO_LAUNCH]: https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_RETURN_TO_LAUNCH
[MAV_CMD_DO_CHANGE_SPEED]: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED
[MAV_CMD_DO_SET_HOME]: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME
[MAV_CMD_DO_JUMP]: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_JUMP

## Guided Course Mode

_Guided Course mode_ holds a constant course over ground and speed without any manual stick input, like the [fixed-wing Guided Course mode](../flight_modes_fw/guided_course.md).
The operator guides the rover from a GCS by sending a new course or speed.

On activation the rover takes its current course over ground if it drives forwards fast enough for the velocity direction to be meaningful (at least 0.5 m/s), and the direction it points in otherwise.
It then drives at the cruising speed until the operator issues a new command.

The course itself is controlled: the heading setpoint is the course corrected by the estimated drift angle between heading and course over ground (from sideslip), the way a fixed-wing vehicle corrects its heading for the wind.
A sideways offset is not corrected: after one, the rover continues on a parallel track.
To follow a line instead, use [Mission mode](#mission-mode) or send a [RoverPositionSetpoint](../msg_docs/RoverPositionSetpoint.md).

| Command                         | Effect                                                                                                       |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| [MAV_CMD_GUIDED_CHANGE_HEADING] | Set a new course over ground (degrees, 0 = north) with `HEADING_TYPE = 0`. Other heading types are rejected. |
| [MAV_CMD_DO_CHANGE_SPEED]       | Set a new ground speed via param2 (m/s), applied immediately. If param2 ≤ 0, the default speed is restored.  |

[MAV_CMD_GUIDED_CHANGE_HEADING]: https://mavlink.io/en/messages/common.html#MAV_CMD_GUIDED_CHANGE_HEADING

Internally the mode publishes a [UgvUsvCourseSetpoint](../msg_docs/UgvUsvCourseSetpoint.md), which [external modes](../ros2/px4_ros2_control_interface.md) can also send directly.

## Return Mode

The vehicle will return to the launch position.
Return mode can be activated through the respective [mission command](#mission-commands) or through the ground station UI.
