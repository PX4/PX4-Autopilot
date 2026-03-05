---
pageClass: is-wide-page
---

# MagnetometerBiasEstimate (UORB message)

**TOPICS:** magnetometer_biasestimate

## Fields

| 参数名                         | 类型           | Unit [Frame] | Range/Enum | 描述                                                        |
| --------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                   | `uint64`     |                                                                  |            | time since system start (microseconds) |
| bias_x | `float32[4]` |                                                                  |            | estimated X-bias of all the sensors                       |
| bias_y | `float32[4]` |                                                                  |            | estimated Y-bias of all the sensors                       |
| bias_z | `float32[4]` |                                                                  |            | estimated Z-bias of all the sensors                       |
| valid                       | `bool[4]`    |                                                                  |            | true if the estimator has converged                       |
| stable                      | `bool[4]`    |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MagnetometerBiasEstimate.msg)

:::details
Click here to see original file

```c
uint64 timestamp                # time since system start (microseconds)

float32[4] bias_x		# estimated X-bias of all the sensors
float32[4] bias_y		# estimated Y-bias of all the sensors
float32[4] bias_z		# estimated Z-bias of all the sensors

bool[4] valid			# true if the estimator has converged
bool[4] stable
```

:::
