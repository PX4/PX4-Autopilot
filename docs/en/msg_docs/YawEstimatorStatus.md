---
pageClass: is-wide-page
---

# YawEstimatorStatus (UORB message)

**TOPICS:** yaw_estimator_status

## Fields

| Name                                                    | Type         | Unit [Frame] | Range/Enum | Description                                                       |
| ------------------------------------------------------- | ------------ | ------------ | ---------- | ----------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                     | `uint64`     |              |            | time since system start (microseconds)                            |
| <a id="fld_timestamp_sample"></a>timestamp_sample       | `uint64`     |              |            | the timestamp of the raw data (microseconds)                      |
| <a id="fld_yaw_composite"></a>yaw_composite             | `float32`    |              |            | composite yaw from GSF (rad)                                      |
| <a id="fld_yaw_variance"></a>yaw_variance               | `float32`    |              |            | composite yaw variance from GSF (rad^2)                           |
| <a id="fld_yaw_composite_valid"></a>yaw_composite_valid | `bool`       |              |            |
| <a id="fld_yaw"></a>yaw                                 | `float32[5]` |              |            | yaw estimate for each model in the filter bank (rad)              |
| <a id="fld_innov_vn"></a>innov_vn                       | `float32[5]` |              |            | North velocity innovation for each model in the filter bank (m/s) |
| <a id="fld_innov_ve"></a>innov_ve                       | `float32[5]` |              |            | East velocity innovation for each model in the filter bank (m/s)  |
| <a id="fld_weight"></a>weight                           | `float32[5]` |              |            | weighting for each model in the filter bank                       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/YawEstimatorStatus.msg)

::: details Click here to see original file

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
