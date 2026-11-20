---
pageClass: is-wide-page
---

# SurfaceVehicleSpeedStatus (UORB message)

Surface Vehicle Speed Status.

Status of the speed controller of surface vehicles, on land or on water (rovers, boats).
Published by the speed controllers of rover_ackermann, rover_differential and rover_mecanum
while they track surface_vehicle_speed_setpoint, for logging and tuning.
Not consumed by other modules.

**TOPICS:** surface_vehicle_speed_status

## Fields

| Name                                                                          | Type      | Unit [Frame] | Range/Enum                          | Description                                                                                                                                                |
| ----------------------------------------------------------------------------- | --------- | ------------ | ----------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                           | `uint64`  | us           |                                     | Time since system start                                                                                                                                    |
| <a id="fld_measured_speed_body_x"></a>measured_speed_body_x                   | `float32` | m/s [Body]   | [-inf (Backwards) : inf (Forwards)] | Measured speed along the body X axis                                                                                                                       |
| <a id="fld_adjusted_speed_body_x_setpoint"></a>adjusted_speed_body_x_setpoint | `float32` | m/s [Body]   | [-inf (Backwards) : inf (Forwards)] | Speed setpoint along the body X axis that is being tracked (Applied slew rates)                                                                            |
| <a id="fld_pid_thrust_body_x_integral"></a>pid_thrust_body_x_integral         | `float32` |              | [-1 : 1]                            | Integral of the PID for the closed loop controller of the speed along the body X axis                                                                      |
| <a id="fld_measured_speed_body_y"></a>measured_speed_body_y                   | `float32` | m/s [Body]   | [-inf (Left) : inf (Right)]         | Omnidirectional vehicles only: Measured speed along the body Y axis (Invalid: NaN If not omnidirectional)                                                  |
| <a id="fld_adjusted_speed_body_y_setpoint"></a>adjusted_speed_body_y_setpoint | `float32` | m/s [Body]   | [-inf (Left) : inf (Right)]         | Omnidirectional vehicles only: Speed setpoint along the body Y axis that is being tracked (Applied slew rates) (Invalid: NaN If not omnidirectional)       |
| <a id="fld_pid_thrust_body_y_integral"></a>pid_thrust_body_y_integral         | `float32` |              | [-1 : 1]                            | Omnidirectional vehicles only: Integral of the PID for the closed loop controller of the speed along the body Y axis (Invalid: NaN If not omnidirectional) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SurfaceVehicleSpeedStatus.msg)

::: details Click here to see original file

```c
# Surface Vehicle Speed Status
#
# Status of the speed controller of surface vehicles, on land or on water (rovers, boats).
# Published by the speed controllers of rover_ackermann, rover_differential and rover_mecanum
# while they track surface_vehicle_speed_setpoint, for logging and tuning.
# Not consumed by other modules.

uint64 timestamp # [us] Time since system start
float32 measured_speed_body_x # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Measured speed along the body X axis
float32 adjusted_speed_body_x_setpoint # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint along the body X axis that is being tracked (Applied slew rates)
float32 pid_thrust_body_x_integral # [-] [@range -1, 1] Integral of the PID for the closed loop controller of the speed along the body X axis
float32 measured_speed_body_y # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not omnidirectional] Omnidirectional vehicles only: Measured speed along the body Y axis
float32 adjusted_speed_body_y_setpoint # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not omnidirectional] Omnidirectional vehicles only: Speed setpoint along the body Y axis that is being tracked (Applied slew rates)
float32 pid_thrust_body_y_integral # [-] [@range -1, 1] [@invalid NaN If not omnidirectional] Omnidirectional vehicles only: Integral of the PID for the closed loop controller of the speed along the body Y axis
```

:::
