---
pageClass: is-wide-page
---

# RoverRateStatus (UORB message)

Rover Rate Status.

**TOPICS:** rover_rate_status

## Fields

| Name                                                                  | Type      | Unit [Frame] | Range/Enum   | Description                                                  |
| --------------------------------------------------------------------- | --------- | ------------ | ------------ | ------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                                   | `uint64`  | us           |              | Time since system start                                      |
| <a id="fld_measured_yaw_rate"></a>measured_yaw_rate                   | `float32` | rad/s [NED]  | [-inf : inf] | Measured yaw rate                                            |
| <a id="fld_adjusted_yaw_rate_setpoint"></a>adjusted_yaw_rate_setpoint | `float32` | rad/s [NED]  | [-inf : inf] | Yaw rate setpoint that is being tracked (Applied slew rates) |
| <a id="fld_pid_yaw_rate_integral"></a>pid_yaw_rate_integral           | `float32` |              | [-1 : 1]     | Integral of the PID for the closed loop yaw rate controller  |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverRateStatus.msg)

::: details Click here to see original file

```c
# Rover Rate Status

uint64 timestamp                    # [us] Time since system start
float32 measured_yaw_rate           # [rad/s] [@range -inf, inf] [@frame NED] Measured yaw rate
float32 adjusted_yaw_rate_setpoint  # [rad/s] [@range -inf, inf] [@frame NED] Yaw rate setpoint that is being tracked (Applied slew rates)
float32 pid_yaw_rate_integral       # [-] [@range -1, 1] Integral of the PID for the closed loop yaw rate controller
```

:::
