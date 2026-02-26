---
pageClass: is-wide-page
---

# VehicleAcceleration (UORB message)

**TOPICS:** vehicle_acceleration

## Fields

| 명칭                                    | 형식           | Unit [Frame] | Range/Enum | 설명                                                                                                         |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`     |                                                                  |            | time since system start (microseconds)                                                  |
| timestamp_sample | `uint64`     |                                                                  |            | the timestamp of the raw data (microseconds)                                            |
| xyz                                   | `float32[3]` |                                                                  |            | Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2 |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleAcceleration.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint64 timestamp_sample		# the timestamp of the raw data (microseconds)

float32[3] xyz			# Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2
```

:::
