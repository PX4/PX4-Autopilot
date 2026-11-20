---
pageClass: is-wide-page
---

# SurfaceVehicleCourseSetpoint (UORB message)

Surface Vehicle Course Setpoint.

Course setpoint for surface vehicles, on land or on water (rovers, boats).
Published by the position controllers of rover_ackermann, rover_differential and rover_mecanum (pure
pursuit towards the target) and by their Guided Course mode. Can be sent from ROS 2 over uXRCE-DDS
(/fmu/in/surface_vehicle_course_setpoint).
Consumed by the course controllers of these modules, which turn it into attitude and speed setpoints.
The vehicle tries to track the course over ground, as with the fixed-wing course hold: vehicles
that steer by heading correct their heading by the estimated drift angle; omnidirectional vehicles drive
along the course setpoint directly. There is no desired track, so a sideways offset is not
corrected. To follow a desired track, publish surface_vehicle_position_setpoint instead.

**TOPICS:** surface_vehicle_course_setpoint

## Fields

| Name                                | Type      | Unit [Frame] | Range/Enum   | Description                                                                                                                                                                                                                                                                                                                                                                                                  |
| ----------------------------------- | --------- | ------------ | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp | `uint64`  | us           |              | Time since system start                                                                                                                                                                                                                                                                                                                                                                                      |
| <a id="fld_course"></a>course       | `float32` | rad [NED]    | [-pi : pi]   | Course over ground to hold (Invalid: NaN No course. The last valid course is controlled until a NaN course is received. With a speed, the speed is still commanded (rover_ackermann and rover_differential keep the current heading, rover_mecanum drives along its body x axis); with NaN speed too, the course controller stops)                                                                           |
| <a id="fld_speed"></a>speed         | `float32` | m/s          | [-inf : inf] | Cruising speed; a negative value drives the vehicle in reverse (rear first) along the course (Invalid: NaN Speed not controlled: the speed controller is deactivated, and the heading is turned so that an external thrust commanded (surface_vehicle_thrust_setpoint) drives along the course: the sign of x gives forwards or reverse, rover_mecanum also aligns any x/y thrust direction with the course) |
| <a id="fld_yaw"></a>yaw             | `float32` | rad [NED]    | [-pi : pi]   | Currently unused. Only for omnidirectional vehicles: vehicle yaw while holding the course. (Invalid: NaN Not considered)                                                                                                                                                                                                                                                                                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SurfaceVehicleCourseSetpoint.msg)

::: details Click here to see original file

```c
# Surface Vehicle Course Setpoint
#
# Course setpoint for surface vehicles, on land or on water (rovers, boats).
# Published by the position controllers of rover_ackermann, rover_differential and rover_mecanum (pure
# pursuit towards the target) and by their Guided Course mode. Can be sent from ROS 2 over uXRCE-DDS
# (/fmu/in/surface_vehicle_course_setpoint).
# Consumed by the course controllers of these modules, which turn it into attitude and speed setpoints.
# The vehicle tries to track the course over ground, as with the fixed-wing course hold: vehicles
# that steer by heading correct their heading by the estimated drift angle; omnidirectional vehicles drive
# along the course setpoint directly. There is no desired track, so a sideways offset is not
# corrected. To follow a desired track, publish surface_vehicle_position_setpoint instead.

uint64 timestamp # [us] Time since system start
float32 course # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN No course. The last valid course is controlled until a NaN course is received. With a speed, the speed is still commanded (rover_ackermann and rover_differential keep the current heading, rover_mecanum drives along its body x axis); with NaN speed too, the course controller stops] Course over ground to hold
float32 speed # [m/s] [@range -inf, inf] [@invalid NaN Speed not controlled: the speed controller is deactivated, and the heading is turned so that an external thrust commanded (surface_vehicle_thrust_setpoint) drives along the course: the sign of x gives forwards or reverse, rover_mecanum also aligns any x/y thrust direction with the course] Cruising speed; a negative value drives the vehicle in reverse (rear first) along the course
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Not considered] Currently unused. Only for omnidirectional vehicles: vehicle yaw while holding the course.
```

:::
