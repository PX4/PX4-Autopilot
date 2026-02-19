---
pageClass: is-wide-page
---

# LandingGearWheel (UORB message)

**TOPICS:** landing_gearwheel

## Fields

| 参数名                                                                 | 类型        | Unit [Frame] | Range/Enum | 描述                                                                                                           |
| ------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------ |
| timestamp                                                           | `uint64`  |                                                                  |            | time since system start (microseconds)                                                    |
| normalized_wheel_setpoint | `float32` |                                                                  |            | negative is turning left, positive turning right [-1, 1] |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LandingGearWheel.msg)

:::details
Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)

float32 normalized_wheel_setpoint	# negative is turning left, positive turning right [-1, 1]
```

:::
