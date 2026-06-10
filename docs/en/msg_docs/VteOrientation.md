---
pageClass: is-wide-page
---

# VteOrientation (UORB message)

Vision Target Estimator orientation state, exposing the full yaw filter output with covariances for logging and tuning.

Published by: vision_target_estimator (VTEOrientation).
Subscribed by: logger only. The orientation-related fields consumed elsewhere (precision landing) are exposed on landing_target_pose.

**TOPICS:** vte_orientation

## Fields

| Name                                                | Type      | Unit [Frame] | Range/Enum | Description                         |
| --------------------------------------------------- | --------- | ------------ | ---------- | ----------------------------------- |
| <a id="fld_timestamp"></a>timestamp                 | `uint64`  | us           |            | Time since system start             |
| <a id="fld_orientation_valid"></a>orientation_valid | `bool`    |              |            | Relative orientation estimate valid |
| <a id="fld_yaw"></a>yaw                             | `float32` | rad [NED]    |            | Target yaw angle                    |
| <a id="fld_cov_yaw"></a>cov_yaw                     | `float32` | rad^2        |            | Variance of yaw                     |
| <a id="fld_yaw_rate"></a>yaw_rate                   | `float32` | rad/s [NED]  |            | Target yaw rate                     |
| <a id="fld_cov_yaw_rate"></a>cov_yaw_rate           | `float32` | (rad/s)^2    |            | Variance of yaw_rate                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VteOrientation.msg)

::: details Click here to see original file

```c
# Vision Target Estimator orientation state, exposing the full yaw filter output with covariances for logging and tuning.
#
# Published by: vision_target_estimator (VTEOrientation).
# Subscribed by: logger only. The orientation-related fields consumed elsewhere (precision landing) are exposed on landing_target_pose.

uint64 timestamp # [us] Time since system start

bool orientation_valid # [-] Relative orientation estimate valid

float32 yaw # [rad] [@frame NED] Target yaw angle
float32 cov_yaw # [rad^2] Variance of yaw

float32 yaw_rate # [rad/s] [@frame NED] Target yaw rate
float32 cov_yaw_rate # [(rad/s)^2] Variance of yaw_rate
```

:::
