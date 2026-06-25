---
pageClass: is-wide-page
---

# VelocityLimits (UORB message)

Velocity and yaw rate limits for a multicopter position slow mode only.

**TOPICS:** velocity_limits

## Fields

| Name                                                    | Type      | Unit [Frame] | Range/Enum | Description                            |
| ------------------------------------------------------- | --------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                     | `uint64`  |              |            | time since system start (microseconds) |
| <a id="fld_horizontal_velocity"></a>horizontal_velocity | `float32` | m/s          |            |
| <a id="fld_vertical_velocity"></a>vertical_velocity     | `float32` | m/s          |            |
| <a id="fld_yaw_rate"></a>yaw_rate                       | `float32` | rad/s        |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VelocityLimits.msg)

::: details Click here to see original file

```c
# Velocity and yaw rate limits for a multicopter position slow mode only

uint64 timestamp # time since system start (microseconds)

# absolute speeds, NAN means use default limit
float32 horizontal_velocity # [m/s]
float32 vertical_velocity # [m/s]
float32 yaw_rate # [rad/s]
```

:::
