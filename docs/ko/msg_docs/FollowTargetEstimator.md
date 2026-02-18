---
pageClass: is-wide-page
---

# FollowTargetEstimator (UORB message)

**TOPICS:** follow_targetestimator

## Fields

| 명칭                                                                                         | 형식           | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                                             |
| ------------------------------------------------------------------------------------------ | ------------ | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                  | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                                                      |
| last_filter_reset_timestamp | `uint64`     |                                                                  |            | time of last filter reset (microseconds)                                                                                                                                    |
| valid                                                                                      | `bool`       |                                                                  |            | True if estimator states are okay to be used                                                                                                                                                   |
| stale                                                                                      | `bool`       |                                                                  |            | True if estimator stopped receiving follow_target messages for some time. The estimate can still be valid, though it might be inaccurate. |
| lat_est                                                               | `float64`    |                                                                  |            | Estimated target latitude                                                                                                                                                                      |
| lon_est                                                               | `float64`    |                                                                  |            | Estimated target longitude                                                                                                                                                                     |
| alt_est                                                               | `float32`    |                                                                  |            | Estimated target altitude                                                                                                                                                                      |
| pos_est                                                               | `float32[3]` |                                                                  |            | Estimated target NED position (m)                                                                                                                                           |
| vel_est                                                               | `float32[3]` |                                                                  |            | Estimated target NED velocity (m/s)                                                                                                                                         |
| acc_est                                                               | `float32[3]` |                                                                  |            | Estimated target NED acceleration (m^2/s)                                                                                                                                   |
| prediction_count                                                      | `uint64`     |                                                                  |            |                                                                                                                                                                                                |
| fusion_count                                                          | `uint64`     |                                                                  |            |                                                                                                                                                                                                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FollowTargetEstimator.msg)

:::details
Click here to see original file

```c
uint64 timestamp                     # time since system start (microseconds)
uint64 last_filter_reset_timestamp   # time of last filter reset (microseconds)

bool valid              # True if estimator states are okay to be used
bool stale              # True if estimator stopped receiving follow_target messages for some time. The estimate can still be valid, though it might be inaccurate.

float64 lat_est         # Estimated target latitude
float64 lon_est         # Estimated target longitude
float32 alt_est         # Estimated target altitude

float32[3] pos_est      # Estimated target NED position (m)
float32[3] vel_est      # Estimated target NED velocity (m/s)
float32[3] acc_est      # Estimated target NED acceleration (m^2/s)

uint64 prediction_count
uint64 fusion_count
```

:::
