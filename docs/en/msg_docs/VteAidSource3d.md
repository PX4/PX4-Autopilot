---
pageClass: is-wide-page
---

# VteAidSource3d (UORB message)

Vision Target Estimator 3D fusion aid-source diagnostics, one fusion_status per NED axis.

Published by: vision_target_estimator (VTEPosition) on every fusion attempt, including rejected ones.
Subscribed by: logger only. Inspect observation, innovation, test_ratio, and per-axis fusion_status to debug why a measurement was or was not fused.

**TOPICS:** vte_aid_gps_pos_target vte_aid_gps_pos_mission vte_aid_gps_vel_target vte_aid_gps_vel_uav vte_aid_fiducial_marker

## Fields

| Name                                                      | Type         | Unit [Frame] | Range/Enum                              | Description                                               |
| --------------------------------------------------------- | ------------ | ------------ | --------------------------------------- | --------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                       | `uint64`     | us           |                                         | Time since system start                                   |
| <a id="fld_timestamp_sample"></a>timestamp_sample         | `uint64`     | us           |                                         | Timestamp of the raw observation                          |
| <a id="fld_time_last_predict"></a>time_last_predict       | `uint64`     | us           |                                         | Timestamp of last filter prediction                       |
| <a id="fld_observation"></a>observation                   | `float32[3]` | [NED]        |                                         | Sensor observation attempted to be fused                  |
| <a id="fld_observation_variance"></a>observation_variance | `float32[3]` | [NED]        |                                         | Variance of the observation attempted to be fused         |
| <a id="fld_innovation"></a>innovation                     | `float32[3]` | [NED]        |                                         | Kalman Filter innovation (y = z - Hx)                     |
| <a id="fld_innovation_variance"></a>innovation_variance   | `float32[3]` | [NED]        |                                         | Kalman Filter variance of the innovation                  |
| <a id="fld_test_ratio"></a>test_ratio                     | `float32[3]` |              |                                         | Normalized innovation squared (NIS)                       |
| <a id="fld_fusion_status"></a>fusion_status               | `uint8[3]`   |              | [VTE_FUSION_STATUS](#VTE_FUSION_STATUS) | Fusion status code per axis                               |
| <a id="fld_time_since_meas_ms"></a>time_since_meas_ms     | `float32`    | ms           |                                         | (now - timestamp_sample)                                  |
| <a id="fld_history_steps"></a>history_steps               | `uint8`      |              |                                         | Number of steps replayed in OOSM (0 if current or failed) |

## Enums

### VTE_FUSION_STATUS {#VTE_FUSION_STATUS}

Used in field(s): [fusion_status](#fld_fusion_status)

| Name | Type | Value | Description |
| ---- | ---- | ----- | ----------- |

## Constants

| Name                                                      | Type    | Value | Description                                                         |
| --------------------------------------------------------- | ------- | ----- | ------------------------------------------------------------------- |
| <a id="#STATUS_IDLE"></a> STATUS_IDLE                     | `uint8` | 0     | No fusion attempted yet                                             |
| <a id="#STATUS_FUSED_CURRENT"></a> STATUS_FUSED_CURRENT   | `uint8` | 1     | Fused immediately (low latency)                                     |
| <a id="#STATUS_FUSED_OOSM"></a> STATUS_FUSED_OOSM         | `uint8` | 2     | Fused via history buffer                                            |
| <a id="#STATUS_REJECT_NIS"></a> STATUS_REJECT_NIS         | `uint8` | 3     | Rejected by Normalized Innovation Squared check                     |
| <a id="#STATUS_REJECT_COV"></a> STATUS_REJECT_COV         | `uint8` | 4     | Rejected due to invalid/infinite covariance or numerical error      |
| <a id="#STATUS_REJECT_TOO_OLD"></a> STATUS_REJECT_TOO_OLD | `uint8` | 5     | Rejected: older than buffer limit (kOosmMaxTimeUs) or oldest sample |
| <a id="#STATUS_REJECT_TOO_NEW"></a> STATUS_REJECT_TOO_NEW | `uint8` | 6     | Rejected: timestamp in the future (beyond tolerance)                |
| <a id="#STATUS_REJECT_STALE"></a> STATUS_REJECT_STALE     | `uint8` | 7     | Rejected: history was reset due to staleness/discontinuity          |
| <a id="#STATUS_REJECT_EMPTY"></a> STATUS_REJECT_EMPTY     | `uint8` | 8     | Rejected: history buffer not yet populated                          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VteAidSource3d.msg)

::: details Click here to see original file

```c
# Vision Target Estimator 3D fusion aid-source diagnostics, one fusion_status per NED axis.
#
# Published by: vision_target_estimator (VTEPosition) on every fusion attempt, including rejected ones.
# Subscribed by: logger only. Inspect observation, innovation, test_ratio, and per-axis fusion_status to debug why a measurement was or was not fused.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw observation
uint64 time_last_predict # [us] Timestamp of last filter prediction

# Observation & Innovation
float32[3] observation # [-] [@frame NED] Sensor observation attempted to be fused
float32[3] observation_variance # [-] [@frame NED] Variance of the observation attempted to be fused

float32[3] innovation # [-] [@frame NED] Kalman Filter innovation (y = z - Hx)
float32[3] innovation_variance # [-] [@frame NED] Kalman Filter variance of the innovation

float32[3] test_ratio # [-] Normalized innovation squared (NIS)

uint8[3] fusion_status # [@enum VTE_FUSION_STATUS] Fusion status code per axis
uint8 STATUS_IDLE = 0 # No fusion attempted yet
uint8 STATUS_FUSED_CURRENT = 1 # Fused immediately (low latency)
uint8 STATUS_FUSED_OOSM = 2 # Fused via history buffer
uint8 STATUS_REJECT_NIS = 3 # Rejected by Normalized Innovation Squared check
uint8 STATUS_REJECT_COV = 4 # Rejected due to invalid/infinite covariance or numerical error
uint8 STATUS_REJECT_TOO_OLD = 5 # Rejected: older than buffer limit (kOosmMaxTimeUs) or oldest sample
uint8 STATUS_REJECT_TOO_NEW = 6 # Rejected: timestamp in the future (beyond tolerance)
uint8 STATUS_REJECT_STALE = 7 # Rejected: history was reset due to staleness/discontinuity
uint8 STATUS_REJECT_EMPTY = 8 # Rejected: history buffer not yet populated

# OOSM Diagnostics (Shared across axes)
float32 time_since_meas_ms # [ms] (now - timestamp_sample)
uint8 history_steps # [-] Number of steps replayed in OOSM (0 if current or failed)

# TOPICS vte_aid_gps_pos_target vte_aid_gps_pos_mission vte_aid_gps_vel_target vte_aid_gps_vel_uav
# TOPICS vte_aid_fiducial_marker
```

:::
