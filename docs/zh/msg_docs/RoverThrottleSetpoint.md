---
pageClass: is-wide-page
---

# RoverThrottleSetpoint (UORB message)

Rover Throttle setpoint.

**TOPICS:** rover_throttlesetpoint

## Fields

| 参数名                                                       | 类型        | Unit [Frame] | Range/Enum                                                                                                                                | 描述                                                                                                                                 |
| --------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                 | `uint64`  | us                                                               |                                                                                                                                           | Time since system start                                                                                                            |
| throttle_body_x | `float32` | [Body]       | [-1 (Backwards) : 1 (Forwards)] | Throttle setpoint along body X axis                                                                                                |
| throttle_body_y | `float32` | [Body]       | [-1 (Left) : 1 (Right)]         | Mecanum only: Throttle setpoint along body Y axis (Invalid: NaN If not mecanum) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverThrottleSetpoint.msg)

:::details
Click here to see original file

```c
# Rover Throttle setpoint

uint64 timestamp         # [us] Time since system start
float32 throttle_body_x  # [-] [@range -1 (Backwards), 1 (Forwards)] [@frame Body] Throttle setpoint along body X axis
float32 throttle_body_y  # [-] [@range -1 (Left), 1 (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Throttle setpoint along body Y axis
```

:::
