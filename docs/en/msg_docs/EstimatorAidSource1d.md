---
pageClass: is-wide-page
---

# EstimatorAidSource1d (UORB message)

**TOPICS:** estimator_aid_src_baro_hgt estimator_aid_src_ev_hgt estimator_aid_src_gnss_hgt estimator_aid_src_rng_hgt estimator_aid_src_ranging_beacon estimator_aid_src_airspeed estimator_aid_src_sideslip estimator_aid_src_fake_hgt estimator_aid_src_gnss_yaw estimator_aid_src_ev_yaw

## Fields

| Name                                                      | Type      | Unit [Frame] | Range/Enum | Description                                  |
| --------------------------------------------------------- | --------- | ------------ | ---------- | -------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                       | `uint64`  |              |            | time since system start (microseconds)       |
| <a id="fld_timestamp_sample"></a>timestamp_sample         | `uint64`  |              |            | the timestamp of the raw data (microseconds) |
| <a id="fld_estimator_instance"></a>estimator_instance     | `uint8`   |              |            |
| <a id="fld_device_id"></a>device_id                       | `uint32`  |              |            |
| <a id="fld_time_last_fuse"></a>time_last_fuse             | `uint64`  |              |            |
| <a id="fld_observation"></a>observation                   | `float32` |              |            |
| <a id="fld_observation_variance"></a>observation_variance | `float32` |              |            |
| <a id="fld_innovation"></a>innovation                     | `float32` |              |            |
| <a id="fld_innovation_filtered"></a>innovation_filtered   | `float32` |              |            |
| <a id="fld_innovation_variance"></a>innovation_variance   | `float32` |              |            |
| <a id="fld_test_ratio"></a>test_ratio                     | `float32` |              |            | normalized innovation squared                |
| <a id="fld_test_ratio_filtered"></a>test_ratio_filtered   | `float32` |              |            | signed filtered test ratio                   |
| <a id="fld_innovation_rejected"></a>innovation_rejected   | `bool`    |              |            | true if the observation has been rejected    |
| <a id="fld_fused"></a>fused                               | `bool`    |              |            | true if the sample was successfully fused    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorAidSource1d.msg)

::: details Click here to see original file

```c
uint64 timestamp             # time since system start (microseconds)
uint64 timestamp_sample      # the timestamp of the raw data (microseconds)

uint8 estimator_instance

uint32 device_id

uint64 time_last_fuse

float32 observation
float32 observation_variance

float32 innovation
float32 innovation_filtered

float32 innovation_variance

float32 test_ratio           # normalized innovation squared
float32 test_ratio_filtered  # signed filtered test ratio

bool innovation_rejected     # true if the observation has been rejected
bool fused                   # true if the sample was successfully fused

# TOPICS estimator_aid_src_baro_hgt estimator_aid_src_ev_hgt estimator_aid_src_gnss_hgt estimator_aid_src_rng_hgt estimator_aid_src_ranging_beacon
# TOPICS estimator_aid_src_airspeed estimator_aid_src_sideslip
# TOPICS estimator_aid_src_fake_hgt
# TOPICS estimator_aid_src_gnss_yaw estimator_aid_src_ev_yaw
```

:::
