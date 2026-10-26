---
pageClass: is-wide-page
---

# UgvUsvCourseSetpoint (UORB message)

UGV/USV Course Setpoint.

Course setpoint for unmanned ground and surface vehicles (UGVs such as rovers, USVs such as boats).
Consumed by the position controllers of these vehicles (currently AckermannPosControl,
DifferentialPosControl and MecanumPosControl), which turn it into speed and attitude
setpoints.
Published by the Guided Course mode of rover_ackermann, rover_differential and rover_mecanum,
and can be sent from ROS 2 over uXRCE-DDS (/fmu/in/ugv_usv_course_setpoint).
Like the fixed wing course hold, the course over ground itself is controlled: the heading
setpoint is the course corrected by the estimated drift angle, and a cross-track error is not
corrected. To follow a line, publish rover_position_setpoint instead.
The position controller follows whichever of the two was published most recently.

**TOPICS:** ugv_usv_course_setpoint

## Fields

| Name                                | Type      | Unit [Frame] | Range/Enum   | Description                                                                                                                                                   |
| ----------------------------------- | --------- | ------------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`  | us           |              | Time since system start                                                                                                                                       |
| <a id="fld_course"></a>course       | `float32` | rad [NED]    | [-pi : pi]   | Course over ground to hold (Invalid: NaN No course; the position controller stays idle)                                                                       |
| <a id="fld_speed"></a>speed         | `float32` | m/s          | [-inf : inf] | Cruising speed; a negative value drives the vehicle in reverse (rear first) along the course (Invalid: NaN Defaults to maximum speed)                         |
| <a id="fld_yaw"></a>yaw             | `float32` | rad [NED]    | [-pi : pi]   | Currently unused. Only for vehicles that can move sideways (e.g. mecanum rovers): vehicle yaw while holding the course (Invalid: NaN Defaults to vehicle yaw) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/UgvUsvCourseSetpoint.msg)

::: details Click here to see original file

```c
# UGV/USV Course Setpoint
#
# Course setpoint for unmanned ground and surface vehicles (UGVs such as rovers, USVs such as boats).
# Consumed by the position controllers of these vehicles (currently AckermannPosControl,
# DifferentialPosControl and MecanumPosControl), which turn it into speed and attitude
# setpoints.
# Published by the Guided Course mode of rover_ackermann, rover_differential and rover_mecanum,
# and can be sent from ROS 2 over uXRCE-DDS (/fmu/in/ugv_usv_course_setpoint).
# Like the fixed wing course hold, the course over ground itself is controlled: the heading
# setpoint is the course corrected by the estimated drift angle, and a cross-track error is not
# corrected. To follow a line, publish rover_position_setpoint instead.
# The position controller follows whichever of the two was published most recently.

uint64 timestamp # [us] Time since system start
float32 course # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN No course; the position controller stays idle] Course over ground to hold
float32 speed # [m/s] [@range -inf, inf] [@invalid NaN Defaults to maximum speed] Cruising speed; a negative value drives the vehicle in reverse (rear first) along the course
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw] Currently unused. Only for vehicles that can move sideways (e.g. mecanum rovers): vehicle yaw while holding the course
```

:::
