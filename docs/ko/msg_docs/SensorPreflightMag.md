---
pageClass: is-wide-page
---

# SensorPreflightMag (UORB message)

Pre-flight sensor check metrics. The topic will not be updated when the vehicle is armed.

**TOPICS:** sensor_preflightmag

## Fields

| 명칭                                                                | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                    |
| ----------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------- |
| timestamp                                                         | `uint64`  |                                                                  |            | time since system start (microseconds)                             |
| mag_inconsistency_angle | `float32` |                                                                  |            | maximum angle between magnetometer instance field vectors in radians. |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorPreflightMag.msg)

:::details
Click here to see original file

```c
#
# Pre-flight sensor check metrics.
# The topic will not be updated when the vehicle is armed
#
uint64 timestamp # time since system start (microseconds)

float32 mag_inconsistency_angle # maximum angle between magnetometer instance field vectors in radians.
```

:::
