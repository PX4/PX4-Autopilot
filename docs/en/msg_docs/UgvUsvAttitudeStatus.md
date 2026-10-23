---
pageClass: is-wide-page
---

# UgvUsvAttitudeStatus (UORB message)

UGV/USV Attitude Status.

**TOPICS:** ugv_usv_attitude_status

## Fields

| Name                                                        | Type      | Unit [Frame] | Range/Enum | Description                                             |
| ----------------------------------------------------------- | --------- | ------------ | ---------- | ------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                         | `uint64`  | us           |            | Time since system start                                 |
| <a id="fld_measured_yaw"></a>measured_yaw                   | `float32` | rad [NED]    | [-pi : pi] | Measured yaw                                            |
| <a id="fld_adjusted_yaw_setpoint"></a>adjusted_yaw_setpoint | `float32` | rad [NED]    | [-pi : pi] | Yaw setpoint that is being tracked (Applied slew rates) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/UgvUsvAttitudeStatus.msg)

::: details Click here to see original file

```c
# UGV/USV Attitude Status

uint64 timestamp               # [us] Time since system start
float32 measured_yaw           # [rad] [@range -pi, pi] [@frame NED]Measured yaw
float32 adjusted_yaw_setpoint  # [rad] [@range -pi, pi] [@frame NED] Yaw setpoint that is being tracked (Applied slew rates)
```

:::
