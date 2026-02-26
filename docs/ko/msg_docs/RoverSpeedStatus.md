---
pageClass: is-wide-page
---

# RoverSpeedStatus (UORB message)

Rover Velocity Status.

**TOPICS:** rover_speedstatus

## Fields

| 명칭                                                                                                                 | 형식        | Unit [Frame] | Range/Enum                                                                                                                                    | 설명                                                                                                                                                                                              |
| ------------------------------------------------------------------------------------------------------------------ | --------- | ---------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                                          | `uint64`  | us                                                               |                                                                                                                                               | Time since system start                                                                                                                                                                         |
| measured_speed_body_x                               | `float32` | m/s [Body]   | [-inf (Backwards) : inf (Forwards)] | Measured speed in body x direction                                                                                                                                                              |
| adjusted_speed_body_x_setpoint | `float32` | m/s [Body]   | [-inf (Backwards) : inf (Forwards)] | Speed setpoint in body x direction that is being tracked (Applied slew rates)                                                                                                |
| pid_throttle_body_x_integral   | `float32` |                                                                  | [-1 : 1]                                                                  | Integral of the PID for the closed loop controller of the speed in body x direction                                                                                                             |
| measured_speed_body_y                               | `float32` | m/s [Body]   | [-inf (Left) : inf (Right)]         | Mecanum only: Measured speed in body y direction (Invalid: NaN If not mecanum)                                                               |
| adjusted_speed_body_y_setpoint | `float32` | m/s [Body]   | [-inf (Left) : inf (Right)]         | Mecanum only: Speed setpoint in body y direction that is being tracked (Applied slew rates) (Invalid: NaN If not mecanum) |
| pid_throttle_body_y_integral   | `float32` |                                                                  | [-1 : 1]                                                                  | Mecanum only: Integral of the PID for the closed loop controller of the speed in body y direction (Invalid: NaN If not mecanum)              |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverSpeedStatus.msg)

:::details
Click here to see original file

```c
# Rover Velocity Status

uint64 timestamp                        # [us] Time since system start
float32 measured_speed_body_x           # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Measured speed in body x direction
float32 adjusted_speed_body_x_setpoint  # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint in body x direction that is being tracked (Applied slew rates)
float32 pid_throttle_body_x_integral    # [-] [@range -1, 1] Integral of the PID for the closed loop controller of the speed in body x direction
float32 measured_speed_body_y           # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Measured speed in body y direction
float32 adjusted_speed_body_y_setpoint  # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint in body y direction that is being tracked (Applied slew rates)
float32 pid_throttle_body_y_integral    # [-] [@range -1, 1] [@invalid NaN If not mecanum] Mecanum only: Integral of the PID for the closed loop controller of the speed in body y direction
```

:::
