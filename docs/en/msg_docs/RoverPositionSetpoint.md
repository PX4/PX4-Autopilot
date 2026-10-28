---
pageClass: is-wide-page
---

# RoverPositionSetpoint (UORB message)

Rover Position Setpoint.

Position setpoint consumed by the rover position controllers (AckermannPosControl,
DifferentialPosControl, MecanumPosControl), which turn it into rover_speed_setpoint
and rover_attitude_setpoint.
Published by the manual (position), auto and offboard drive modes of rover_ackermann,
rover_differential and rover_mecanum, and can be sent from ROS 2 over uXRCE-DDS
(/fmu/in/rover_position_setpoint).
The rover follows the desired track from start_ned to target_ned. To control the course over ground
instead, publish surface_vehicle_course_setpoint; the position controller follows whichever
of the two was published most recently.

**TOPICS:** rover_position_setpoint

## Fields

| Name                                          | Type         | Unit [Frame] | Range/Enum   | Description                                                                                                                           |
| --------------------------------------------- | ------------ | ------------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp           | `uint64`     | us           |              | Time since system start                                                                                                               |
| <a id="fld_target_ned"></a>target_ned         | `float32[2]` | m [NED]      | [-inf : inf] | Target position (Invalid: NaN No target; the position controller stays idle)                                                          |
| <a id="fld_start_ned"></a>start_ned           | `float32[2]` | m [NED]      | [-inf : inf] | Start position of the desired track towards target_ned (Invalid: NaN Defaults to the vehicle position when the setpoint is received)  |
| <a id="fld_cruising_speed"></a>cruising_speed | `float32`    | m/s          | [-inf : inf] | Cruising speed; a negative value drives the rover in reverse (rear first) towards target_ned (Invalid: NaN Defaults to maximum speed) |
| <a id="fld_arrival_speed"></a>arrival_speed   | `float32`    | m/s          | [0 : inf]    | Speed magnitude to arrive at the target with, independent of the direction of travel (Invalid: NaN Defaults to 0)                     |
| <a id="fld_yaw"></a>yaw                       | `float32`    | rad [NED]    | [-pi : pi]   | Currently unused. Only for vehicles that can move sideways (e.g. mecanum): vehicle yaw during travel. (Invalid: NaN Not considered)   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverPositionSetpoint.msg)

::: details Click here to see original file

```c
# Rover Position Setpoint
#
# Position setpoint consumed by the rover position controllers (AckermannPosControl,
# DifferentialPosControl, MecanumPosControl), which turn it into rover_speed_setpoint
# and rover_attitude_setpoint.
# Published by the manual (position), auto and offboard drive modes of rover_ackermann,
# rover_differential and rover_mecanum, and can be sent from ROS 2 over uXRCE-DDS
# (/fmu/in/rover_position_setpoint).
# The rover follows the desired track from start_ned to target_ned. To control the course over ground
# instead, publish surface_vehicle_course_setpoint; the position controller follows whichever
# of the two was published most recently.

uint64 timestamp # [us] Time since system start
float32[2] target_ned # [m] [@range -inf, inf] [@frame NED] [@invalid NaN No target; the position controller stays idle] Target position
float32[2] start_ned # [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to the vehicle position when the setpoint is received] Start position of the desired track towards target_ned
float32 cruising_speed # [m/s] [@range -inf, inf] [@invalid NaN Defaults to maximum speed] Cruising speed; a negative value drives the rover in reverse (rear first) towards target_ned
float32 arrival_speed # [m/s] [@range 0, inf] [@invalid NaN Defaults to 0] Speed magnitude to arrive at the target with, independent of the direction of travel
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Not considered] Currently unused. Only for vehicles that can move sideways (e.g. mecanum): vehicle yaw during travel.
```

:::
