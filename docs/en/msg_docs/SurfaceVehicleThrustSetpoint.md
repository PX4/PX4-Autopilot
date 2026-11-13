---
pageClass: is-wide-page
---

# SurfaceVehicleThrustSetpoint (UORB message)

Surface Vehicle Thrust Setpoint.

Normalized thrust setpoint for surface vehicles, on land or on water (rovers, boats).
Published by the speed controllers of rover_ackermann, rover_differential and rover_mecanum,
which turn surface_vehicle_speed_setpoint into it, and by their manual, acro, stabilized and
manual parking modes directly from the sticks. Can be sent from ROS 2 over uXRCE-DDS
(/fmu/in/surface_vehicle_thrust_setpoint).
The actuator controllers of these modules turn it into motor commands (actuator_motors).

**TOPICS:** surface_vehicle_thrust_setpoint

## Fields

| Name                                | Type      | Unit [Frame] | Range/Enum                      | Description                                                                                                           |
| ----------------------------------- | --------- | ------------ | ------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`  | us           |                                 | Time since system start                                                                                               |
| <a id="fld_x"></a>x                 | `float32` | [Body]       | [-1 (Backwards) : 1 (Forwards)] | Normalized thrust setpoint along the body X axis                                                                      |
| <a id="fld_y"></a>y                 | `float32` | [Body]       | [-1 (Left) : 1 (Right)]         | Omnidirectional vehicles only: Normalized thrust setpoint along the body Y axis (Invalid: NaN If not omnidirectional) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SurfaceVehicleThrustSetpoint.msg)

::: details Click here to see original file

```c
# Surface Vehicle Thrust Setpoint
#
# Normalized thrust setpoint for surface vehicles, on land or on water (rovers, boats).
# Published by the speed controllers of rover_ackermann, rover_differential and rover_mecanum,
# which turn surface_vehicle_speed_setpoint into it, and by their manual, acro, stabilized and
# manual parking modes directly from the sticks. Can be sent from ROS 2 over uXRCE-DDS
# (/fmu/in/surface_vehicle_thrust_setpoint).
# The actuator controllers of these modules turn it into motor commands (actuator_motors).

uint64 timestamp # [us] Time since system start
float32 x # [-] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Normalized thrust setpoint along the body X axis
float32 y # [-] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not omnidirectional] Omnidirectional vehicles only: Normalized thrust setpoint along the body Y axis
```

:::
