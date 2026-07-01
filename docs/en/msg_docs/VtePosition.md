---
pageClass: is-wide-page
---

# VtePosition (UORB message)

Vision Target Estimator position state, exposing the full per-axis Kalman filter state with covariances for logging and tuning.

Published by: vision_target_estimator (VTEPosition).
Subscribed by: logger only. The position-related fields consumed elsewhere (precision landing, EKF2 aiding) are exposed on landing_target_pose.

vel_target and acc_target are only populated when the firmware is built with CONFIG_VTEST_MOVING=y; otherwise they stay at zero.

**TOPICS:** vte_position

## Fields

| Name                                          | Type         | Unit [Frame]    | Range/Enum | Description                                    |
| --------------------------------------------- | ------------ | --------------- | ---------- | ---------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp           | `uint64`     | us              |            | Time since system start                        |
| <a id="fld_rel_pos_valid"></a>rel_pos_valid   | `bool`       |                 |            | Relative position estimate valid               |
| <a id="fld_rel_vel_valid"></a>rel_vel_valid   | `bool`       |                 |            | Relative velocity estimate valid               |
| <a id="fld_rel_pos"></a>rel_pos               | `float32[3]` | m [NED]         |            | Target position relative to vehicle            |
| <a id="fld_vel_uav"></a>vel_uav               | `float32[3]` | m/s [NED]       |            | Vehicle velocity                               |
| <a id="fld_vel_target"></a>vel_target         | `float32[3]` | m/s [NED]       |            | Target velocity                                |
| <a id="fld_bias"></a>bias                     | `float32[3]` | m [NED]         |            | GNSS bias between vehicle and target receivers |
| <a id="fld_acc_target"></a>acc_target         | `float32[3]` | m/s^2 [NED]     |            | Target acceleration                            |
| <a id="fld_cov_rel_pos"></a>cov_rel_pos       | `float32[3]` | m^2 [NED]       |            | Variance of rel_pos                            |
| <a id="fld_cov_vel_uav"></a>cov_vel_uav       | `float32[3]` | (m/s)^2 [NED]   |            | Variance of vel_uav                            |
| <a id="fld_cov_bias"></a>cov_bias             | `float32[3]` | m^2 [NED]       |            | Variance of bias                               |
| <a id="fld_cov_vel_target"></a>cov_vel_target | `float32[3]` | (m/s)^2 [NED]   |            | Variance of vel_target                         |
| <a id="fld_cov_acc_target"></a>cov_acc_target | `float32[3]` | (m/s^2)^2 [NED] |            | Variance of acc_target                         |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VtePosition.msg)

::: details Click here to see original file

```c
# Vision Target Estimator position state, exposing the full per-axis Kalman filter state with covariances for logging and tuning.
#
# Published by: vision_target_estimator (VTEPosition).
# Subscribed by: logger only. The position-related fields consumed elsewhere (precision landing, EKF2 aiding) are exposed on landing_target_pose.
#
# vel_target and acc_target are only populated when the firmware is built with CONFIG_VTEST_MOVING=y; otherwise they stay at zero.

uint64 timestamp # [us] Time since system start

bool rel_pos_valid # [-] Relative position estimate valid
bool rel_vel_valid # [-] Relative velocity estimate valid

float32[3] rel_pos # [m] [@frame NED] Target position relative to vehicle
float32[3] vel_uav # [m/s] [@frame NED] Vehicle velocity
float32[3] vel_target # [m/s] [@frame NED] Target velocity
float32[3] bias # [m] [@frame NED] GNSS bias between vehicle and target receivers
float32[3] acc_target # [m/s^2] [@frame NED] Target acceleration

float32[3] cov_rel_pos # [m^2] [@frame NED] Variance of rel_pos
float32[3] cov_vel_uav # [(m/s)^2] [@frame NED] Variance of vel_uav
float32[3] cov_bias # [m^2] [@frame NED] Variance of bias
float32[3] cov_vel_target # [(m/s)^2] [@frame NED] Variance of vel_target
float32[3] cov_acc_target # [(m/s^2)^2] [@frame NED] Variance of acc_target
```

:::
