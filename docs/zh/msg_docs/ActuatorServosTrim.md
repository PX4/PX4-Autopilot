---
pageClass: is-wide-page
---

# ActuatorServosTrim (UORB message)

Servo trims, added as offset to servo outputs.

**TOPICS:** actuator_servostrim

## Fields

| 参数名       | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                 |
| --------- | ------------ | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------- |
| timestamp | `uint64`     |                                                                  |            | time since system start (microseconds)                          |
| trim      | `float32[8]` |                                                                  |            | range: [-1, 1] |

## Constants

| 参数名                                                            | 类型      | 值 | 描述 |
| -------------------------------------------------------------- | ------- | - | -- |
| <a href="#NUM_CONTROLS"></a> NUM_CONTROLS | `uint8` | 8 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorServosTrim.msg)

:::details
Click here to see original file

```c
# Servo trims, added as offset to servo outputs
uint64 timestamp			# time since system start (microseconds)

uint8 NUM_CONTROLS = 8
float32[8] trim    # range: [-1, 1]
```

:::
