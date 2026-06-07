---
pageClass: is-wide-page
---

# VehicleOpticalFlowVel (UORB message)

**TOPICS:** estimator_optical_flow_vel vehicle_optical_flow_vel

## Fields

| Name                                                            | Type         | Unit [Frame] | Range/Enum | Description                                                                                                           |
| --------------------------------------------------------------- | ------------ | ------------ | ---------- | --------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                             | `uint64`     |              |            | time since system start (microseconds)                                                                                |
| <a id="fld_timestamp_sample"></a>timestamp_sample               | `uint64`     |              |            | the timestamp of the raw data (microseconds)                                                                          |
| <a id="fld_vel_body"></a>vel_body                               | `float32[2]` |              |            | velocity obtained from gyro-compensated and distance-scaled optical flow raw measurements in body frame(m/s)          |
| <a id="fld_vel_ne"></a>vel_ne                                   | `float32[2]` |              |            | same as vel_body but in local frame (m/s)                                                                             |
| <a id="fld_vel_body_filtered"></a>vel_body_filtered             | `float32[2]` |              |            | filtered velocity obtained from gyro-compensated and distance-scaled optical flow raw measurements in body frame(m/s) |
| <a id="fld_vel_ne_filtered"></a>vel_ne_filtered                 | `float32[2]` |              |            | filtered same as vel_body_filtered but in local frame (m/s)                                                           |
| <a id="fld_flow_rate_uncompensated"></a>flow_rate_uncompensated | `float32[2]` |              |            | integrated optical flow measurement (rad/s)                                                                           |
| <a id="fld_flow_rate_compensated"></a>flow_rate_compensated     | `float32[2]` |              |            | integrated optical flow measurement compensated for angular motion (rad/s)                                            |
| <a id="fld_gyro_rate"></a>gyro_rate                             | `float32[3]` |              |            | gyro measurement synchronized with flow measurements (rad/s)                                                          |
| <a id="fld_gyro_bias"></a>gyro_bias                             | `float32[3]` |              |            |
| <a id="fld_ref_gyro"></a>ref_gyro                               | `float32[3]` |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleOpticalFlowVel.msg)

::: details Click here to see original file

```c
uint64 timestamp                       # time since system start (microseconds)
uint64 timestamp_sample                # the timestamp of the raw data (microseconds)

float32[2] vel_body                    # velocity obtained from gyro-compensated and distance-scaled optical flow raw measurements in body frame(m/s)
float32[2] vel_ne                      # same as vel_body but in local frame (m/s)

float32[2] vel_body_filtered           # filtered velocity obtained from gyro-compensated and distance-scaled optical flow raw measurements in body frame(m/s)
float32[2] vel_ne_filtered             # filtered same as vel_body_filtered but in local frame (m/s)

float32[2] flow_rate_uncompensated     # integrated optical flow measurement (rad/s)
float32[2] flow_rate_compensated       # integrated optical flow measurement compensated for angular motion (rad/s)

float32[3] gyro_rate                   # gyro measurement synchronized with flow measurements (rad/s)

float32[3] gyro_bias
float32[3] ref_gyro

# TOPICS estimator_optical_flow_vel vehicle_optical_flow_vel
```

:::
