---
pageClass: is-wide-page
---

# EstimatorAidSource2d (UORB message)

**TOPICS:** estimator_aid_src_ev_pos estimator_aid_src_fake_pos estimator_aid_src_gnss_pos estimator_aid_src_aux_global_position estimator_aid_src_aux_vel estimator_aid_src_optical_flow estimator_aid_src_drag

## Fields

| Name                                                      | Type         | Unit [Frame] | Range/Enum | Description                                  |
| --------------------------------------------------------- | ------------ | ------------ | ---------- | -------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                       | `uint64`     |              |            | time since system start (microseconds)       |
| <a id="fld_timestamp_sample"></a>timestamp_sample         | `uint64`     |              |            | the timestamp of the raw data (microseconds) |
| <a id="fld_estimator_instance"></a>estimator_instance     | `uint8`      |              |            |
| <a id="fld_device_id"></a>device_id                       | `uint32`     |              |            |
| <a id="fld_time_last_fuse"></a>time_last_fuse             | `uint64`     |              |            |
| <a id="fld_observation"></a>observation                   | `float64[2]` |              |            |
| <a id="fld_observation_variance"></a>observation_variance | `float32[2]` |              |            |
| <a id="fld_innovation"></a>innovation                     | `float32[2]` |              |            |
| <a id="fld_innovation_filtered"></a>innovation_filtered   | `float32[2]` |              |            |
| <a id="fld_innovation_variance"></a>innovation_variance   | `float32[2]` |              |            |
| <a id="fld_test_ratio"></a>test_ratio                     | `float32[2]` |              |            | normalized innovation squared                |
| <a id="fld_test_ratio_filtered"></a>test_ratio_filtered   | `float32[2]` |              |            | signed filtered test ratio                   |
| <a id="fld_innovation_rejected"></a>innovation_rejected   | `bool`       |              |            | true if the observation has been rejected    |
| <a id="fld_fused"></a>fused                               | `bool`       |              |            | true if the sample was successfully fused    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorAidSource2d.msg)

::: details Click here to see original file

```c
uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

uint8 estimator_instance

uint32 device_id

uint64 time_last_fuse

float64[2] observation
float32[2] observation_variance

float32[2] innovation
float32[2] innovation_filtered

float32[2] innovation_variance

float32[2] test_ratio           # normalized innovation squared
float32[2] test_ratio_filtered  # signed filtered test ratio

bool innovation_rejected        # true if the observation has been rejected
bool fused                      # true if the sample was successfully fused

# TOPICS estimator_aid_src_ev_pos estimator_aid_src_fake_pos estimator_aid_src_gnss_pos estimator_aid_src_aux_global_position
# TOPICS estimator_aid_src_aux_vel estimator_aid_src_optical_flow
# TOPICS estimator_aid_src_drag
```

:::
