---
pageClass: is-wide-page
---

# OffboardControlMode (UORB message)

Off-board control mode.

**TOPICS:** offboard_control_mode

## Fields

| Name              | Type     | Unit [Frame] | Range/Enum | Description                            |
| ----------------- | -------- | ------------ | ---------- | -------------------------------------- |
| timestamp         | `uint64` |              |            | time since system start (microseconds) |
| position          | `bool`   |              |            |
| velocity          | `bool`   |              |            |
| acceleration      | `bool`   |              |            |
| attitude          | `bool`   |              |            |
| body_rate         | `bool`   |              |            |
| thrust_and_torque | `bool`   |              |            |
| direct_actuator   | `bool`   |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OffboardControlMode.msg)

::: details Click here to see original file

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
