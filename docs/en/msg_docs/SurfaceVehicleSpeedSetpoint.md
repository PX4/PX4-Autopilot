---
pageClass: is-wide-page
---

# SurfaceVehicleSpeedSetpoint (UORB message)

Surface Vehicle Speed Setpoint.

Speed setpoint for surface vehicles, on land or on water (rovers, boats).
Published by the position controllers of rover_ackermann, rover_differential and rover_mecanum,
which turn surface_vehicle_position_setpoint and surface_vehicle_course_setpoint into it, by
their position mode while steering or at rest, and by their offboard mode. Can be sent from
ROS 2 over uXRCE-DDS (/fmu/in/surface_vehicle_speed_setpoint).
The speed controllers of these modules turn it into surface_vehicle_thrust_setpoint.

**TOPICS:** surface_vehicle_speed_setpoint

## Fields

| Name                                | Type      | Unit [Frame] | Range/Enum                          | Description                                                                                               |
| ----------------------------------- | --------- | ------------ | ----------------------------------- | --------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`  | us           |                                     | Time since system start                                                                                   |
| <a id="fld_x"></a>x                 | `float32` | m/s [Body]   | [-inf (Backwards) : inf (Forwards)] | Speed setpoint along the body X axis                                                                      |
| <a id="fld_y"></a>y                 | `float32` | m/s [Body]   | [-inf (Left) : inf (Right)]         | Omnidirectional vehicles only: Speed setpoint along the body Y axis (Invalid: NaN If not omnidirectional) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SurfaceVehicleSpeedSetpoint.msg)

::: details Click here to see original file

```c
# Surface Vehicle Speed Setpoint
#
# Speed setpoint for surface vehicles, on land or on water (rovers, boats).
# Published by the position controllers of rover_ackermann, rover_differential and rover_mecanum,
# which turn surface_vehicle_position_setpoint and surface_vehicle_course_setpoint into it, by
# their position mode while steering or at rest, and by their offboard mode. Can be sent from
# ROS 2 over uXRCE-DDS (/fmu/in/surface_vehicle_speed_setpoint).
# The speed controllers of these modules turn it into surface_vehicle_thrust_setpoint.

uint64 timestamp # [us] Time since system start
float32 x # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint along the body X axis
float32 y # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not omnidirectional] Omnidirectional vehicles only: Speed setpoint along the body Y axis
```

:::
