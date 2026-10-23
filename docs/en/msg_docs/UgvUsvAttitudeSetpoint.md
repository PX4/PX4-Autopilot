---
pageClass: is-wide-page
---

# UgvUsvAttitudeSetpoint (UORB message)

UGV/USV Attitude Setpoint.

**TOPICS:** ugv_usv_attitude_setpoint

## Fields

| Name                                      | Type      | Unit [Frame] | Range/Enum   | Description             |
| ----------------------------------------- | --------- | ------------ | ------------ | ----------------------- |
| <a id="fld_timestamp"></a>timestamp       | `uint64`  | us           |              | Time since system start |
| <a id="fld_yaw_setpoint"></a>yaw_setpoint | `float32` | rad [NED]    | [-inf : inf] | Yaw setpoint            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/UgvUsvAttitudeSetpoint.msg)

::: details Click here to see original file

```c
# UGV/USV Attitude Setpoint

uint64 timestamp      # [us] Time since system start
float32 yaw_setpoint  # [rad] [@range -inf, inf] [@frame NED] Yaw setpoint
```

:::
