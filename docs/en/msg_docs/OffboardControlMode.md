---
pageClass: is-wide-page
---

# OffboardControlMode (UORB message)

Off-board control mode.

**TOPICS:** offboard_control_mode

## Fields

| Name                                                | Type     | Unit [Frame] | Range/Enum | Description                            |
| --------------------------------------------------- | -------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                 | `uint64` |              |            | time since system start (microseconds) |
| <a id="fld_position"></a>position                   | `bool`   |              |            |
| <a id="fld_velocity"></a>velocity                   | `bool`   |              |            |
| <a id="fld_acceleration"></a>acceleration           | `bool`   |              |            |
| <a id="fld_attitude"></a>attitude                   | `bool`   |              |            |
| <a id="fld_body_rate"></a>body_rate                 | `bool`   |              |            |
| <a id="fld_thrust_and_torque"></a>thrust_and_torque | `bool`   |              |            |
| <a id="fld_direct_actuator"></a>direct_actuator     | `bool`   |              |            |

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
