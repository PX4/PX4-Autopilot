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

_Guided Course mode_ makes the rover drive in a fixed direction at a fixed speed without any manual stick input, like the [Fixed-Wing Guided Course mode](../flight_modes_fw/guided_course.md).
The direction is a course over ground: the direction the rover moves over the ground, which can differ from its heading.

The mode is selected like any other mode, for example in the GCS.
On entry, the rover maintains its current velocity vector:

- **Course:** If the rover is driving forwards faster than 0.5 m/s it keeps its current course over ground.
  When slower (or reversing) its direction of travel is not reliable, so it uses the current heading as a course setpoint instead.
- **Speed:** The rover keeps its current forward speed.
  If it is entered at rest or while reversing, the rover stops and waits for a speed command.

While the mode is active, the course and speed can be changed with these MAVLink commands, sent from a GCS or companion computer:

- [MAV_CMD_GUIDED_CHANGE_HEADING](https://mavlink.io/en/messages/common.html#MAV_CMD_GUIDED_CHANGE_HEADING): Set a new course over ground (degrees, 0 = north) with `HEADING_TYPE = 0` (Course over Ground). Other heading types are rejected.
- [MAV_CMD_DO_CHANGE_SPEED](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED): Set a new ground speed via param2 (m/s), applied immediately and limited to [RO_SPEED_LIM](../advanced_config/parameter_reference.md#RO_SPEED_LIM).
  `0` stops the rover, negative values keep the current speed setpoint. |

The rover tries to track the commanded course over ground.
Ackermann and differential rovers correct their heading by the estimated drift angle so that their course over ground matches the course setpoint, the way a fixed-wing vehicle crabs into the wind.
Mecanum rovers drive along the course setpoint directly without adjusting their heading.
There is no desired track: after a sideways offset, the rover continues on a parallel track.
To follow a desired track instead, use [Mission mode](#mission-mode) or send a [RoverPositionSetpoint](../msg_docs/RoverPositionSetpoint.md).

Internally the mode publishes a [SurfaceVehicleCourseSetpoint](../msg_docs/SurfaceVehicleCourseSetpoint.md), which [external modes](../ros2/px4_ros2_control_interface.md) can also send directly.

## Return Mode

The vehicle will return to the launch position.
Return mode can be activated through the respective [mission command](#mission-commands) or through the ground station UI.
