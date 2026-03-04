---
pageClass: is-wide-page
---

# EstimatorBias (UORB message)

**TOPICS:** estimator_baro_bias estimator_gnss_hgt_bias

## Fields

| 参数名                                                        | 类型        | Unit [Frame] | Range/Enum | 描述                                                                          |
| ---------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------- |
| timestamp                                                  | `uint64`  |                                                                  |            | time since system start (microseconds)                   |
| timestamp_sample                      | `uint64`  |                                                                  |            | the timestamp of the raw data (microseconds)             |
| device_id                             | `uint32`  |                                                                  |            | unique device ID for the sensor that does not change between power cycles   |
| bias                                                       | `float32` |                                                                  |            | estimated barometric altitude bias (m)                   |
| bias_var                              | `float32` |                                                                  |            | estimated barometric altitude bias variance (m^2)        |
| innov                                                      | `float32` |                                                                  |            | innovation of the last measurement fusion (m)            |
| innov_var                             | `float32` |                                                                  |            | innovation variance of the last measurement fusion (m^2) |
| innov_test_ratio | `float32` |                                                                  |            | normalized innovation squared test ratio                                    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorBias.msg)

:::details
Click here to see original file

```c
uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

uint32 device_id		# unique device ID for the sensor that does not change between power cycles
float32 bias			# estimated barometric altitude bias (m)
float32 bias_var		# estimated barometric altitude bias variance (m^2)

float32 innov			# innovation of the last measurement fusion (m)
float32 innov_var		# innovation variance of the last measurement fusion (m^2)
float32 innov_test_ratio	# normalized innovation squared test ratio

# TOPICS estimator_baro_bias estimator_gnss_hgt_bias
```

:::
