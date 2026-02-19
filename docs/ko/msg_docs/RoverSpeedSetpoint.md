---
pageClass: is-wide-page
---

# RoverSpeedSetpoint (UORB message)

Rover Speed Setpoint.

**TOPICS:** rover_speedsetpoint

## Fields

| 명칭                                                     | 형식        | Unit [Frame] | Range/Enum                                                                                                                                    | 설명                                                                                                                                |
| ------------------------------------------------------ | --------- | ---------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                              | `uint64`  | us                                                               |                                                                                                                                               | Time since system start                                                                                                           |
| speed_body_x | `float32` | m/s [Body]   | [-inf (Backwards) : inf (Forwards)] | Speed setpoint in body x direction                                                                                                |
| speed_body_y | `float32` | m/s [Body]   | [-inf (Left) : inf (Right)]         | Mecanum only: Speed setpoint in body y direction (Invalid: NaN If not mecanum) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverSpeedSetpoint.msg)

:::details
Click here to see original file

```c
# Rover Speed Setpoint

uint64 timestamp      # [us] Time since system start
float32 speed_body_x  # [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint in body x direction
float32 speed_body_y  # [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint in body y direction
```

:::
