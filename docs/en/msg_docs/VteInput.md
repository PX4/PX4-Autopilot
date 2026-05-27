---
pageClass: is-wide-page
---

# VteInput (UORB message)

Vehicle inputs fed into the Vision Target Estimator position prediction step, logged for tuning.

Published by: vision_target_estimator (VisionTargetEst work item).
Subscribed by: logger only.

**TOPICS:** vte_input

## Fields

| Name                                              | Type         | Unit [Frame] | Range/Enum | Description                                                     |
| ------------------------------------------------- | ------------ | ------------ | ---------- | --------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`     | us           |            | Time since system start                                         |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`     | us           |            | Timestamp of the raw input data                                 |
| <a id="fld_acc_xyz"></a>acc_xyz                   | `float32[3]` | m/s^2 [NED]  |            | Downsampled UAV bias-corrected acceleration (including gravity) |
| <a id="fld_q_att"></a>q_att                       | `float32[4]` |              |            | Downsampled UAV attitude quaternion (FRD body -> NED earth)     |
| <a id="fld_acc_sample_count"></a>acc_sample_count | `uint32`     |              |            | Number of raw samples averaged into acc_xyz this cycle          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VteInput.msg)

::: details Click here to see original file

```c
# Vehicle inputs fed into the Vision Target Estimator position prediction step, logged for tuning.
#
# Published by: vision_target_estimator (VisionTargetEst work item).
# Subscribed by: logger only.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw input data

float32[3] acc_xyz # [m/s^2] [@frame NED] Downsampled UAV bias-corrected acceleration (including gravity)
float32[4] q_att # [-] Downsampled UAV attitude quaternion (FRD body -> NED earth)
uint32 acc_sample_count # [-] Number of raw samples averaged into acc_xyz this cycle
```

:::
