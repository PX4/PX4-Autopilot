---
pageClass: is-wide-page
---

# YawEstimatorStatus (UORB message)

**TOPICS:** yaw_estimatorstatus

## Fields

| 명칭                                                            | 형식           | Unit [Frame] | Range/Enum | 설명                                                                                   |
| ------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------ |
| timestamp                                                     | `uint64`     |                                                                  |            | time since system start (microseconds)                            |
| timestamp_sample                         | `uint64`     |                                                                  |            | the timestamp of the raw data (microseconds)                      |
| yaw_composite                            | `float32`    |                                                                  |            | composite yaw from GSF (rad)                                      |
| yaw_variance                             | `float32`    |                                                                  |            | composite yaw variance from GSF (rad^2)                           |
| yaw_composite_valid | `bool`       |                                                                  |            |                                                                                      |
| yaw                                                           | `float32[5]` |                                                                  |            | yaw estimate for each model in the filter bank (rad)              |
| innov_vn                                 | `float32[5]` |                                                                  |            | North velocity innovation for each model in the filter bank (m/s) |
| innov_ve                                 | `float32[5]` |                                                                  |            | East velocity innovation for each model in the filter bank (m/s)  |
| weight                                                        | `float32[5]` |                                                                  |            | weighting for each model in the filter bank                                          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/YawEstimatorStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32 yaw_composite	# composite yaw from GSF (rad)
float32 yaw_variance	# composite yaw variance from GSF (rad^2)
bool yaw_composite_valid

float32[5] yaw		# yaw estimate for each model in the filter bank (rad)
float32[5] innov_vn	# North velocity innovation for each model in the filter bank (m/s)
float32[5] innov_ve	# East velocity innovation for each model in the filter bank (m/s)
float32[5] weight	# weighting for each model in the filter bank
```

:::
