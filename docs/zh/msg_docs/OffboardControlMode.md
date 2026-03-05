---
pageClass: is-wide-page
---

# OffboardControlMode (UORB消息)

Off-board control mode.

**TOPICS:** offboard_controlmode

## Fields

| 参数名                                                         | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| ----------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                   | `uint64` |                                                                  |            | time since system start (microseconds) |
| 位置                                                          | `bool`   |                                                                  |            |                                                           |
| 速度                                                          | `bool`   |                                                                  |            |                                                           |
| acceleration                                                | `bool`   |                                                                  |            |                                                           |
| attitude                                                    | `bool`   |                                                                  |            |                                                           |
| body_rate                              | `bool`   |                                                                  |            |                                                           |
| thrust_and_torque | `bool`   |                                                                  |            |                                                           |
| direct_actuator                        | `bool`   |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OffboardControlMode.msg)

:::details
Click here to see original file

```c
# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

bool position
bool velocity
bool acceleration
bool attitude
bool body_rate
bool thrust_and_torque
bool direct_actuator
```

:::
