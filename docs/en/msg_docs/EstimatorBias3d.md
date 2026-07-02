---
pageClass: is-wide-page
---

# EstimatorBias3d (UORB message)

**TOPICS:** estimator_bias3d estimator_ev_pos_bias

## Fields

| Name                                              | Type         | Unit [Frame] | Range/Enum | Description                                                               |
| ------------------------------------------------- | ------------ | ------------ | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`     |              |            | time since system start (microseconds)                                    |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`     |              |            | the timestamp of the raw data (microseconds)                              |
| <a id="fld_device_id"></a>device_id               | `uint32`     |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_bias"></a>bias                         | `float32[3]` |              |            | estimated barometric altitude bias (m)                                    |
| <a id="fld_bias_var"></a>bias_var                 | `float32[3]` |              |            | estimated barometric altitude bias variance (m^2)                         |
| <a id="fld_innov"></a>innov                       | `float32[3]` |              |            | innovation of the last measurement fusion (m)                             |
| <a id="fld_innov_var"></a>innov_var               | `float32[3]` |              |            | innovation variance of the last measurement fusion (m^2)                  |
| <a id="fld_innov_test_ratio"></a>innov_test_ratio | `float32[3]` |              |            | normalized innovation squared test ratio                                  |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorBias3d.msg)

::: details Click here to see original file

```c
uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

uint32 device_id                # unique device ID for the sensor that does not change between power cycles

float32[3] bias                 # estimated barometric altitude bias (m)
float32[3] bias_var             # estimated barometric altitude bias variance (m^2)

float32[3] innov                # innovation of the last measurement fusion (m)
float32[3] innov_var            # innovation variance of the last measurement fusion (m^2)
float32[3] innov_test_ratio     # normalized innovation squared test ratio

# TOPICS estimator_bias3d
# TOPICS estimator_ev_pos_bias
```

:::
