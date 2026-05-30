---
pageClass: is-wide-page
---

# FiducialMarkerYawReport (UORB message)

Yaw of a precision-landing target relative to the NED (North, East, Down) frame, reported by a vision pipeline.

Published by: vision pipelines (on-board or off-board over MAVLink TARGET_RELATIVE), decoded in mavlink_receiver.
Subscribed by: vision_target_estimator (VTEOrientation).

**TOPICS:** fiducial_marker_yaw_report

## Fields

| Name                                              | Type      | Unit [Frame] | Range/Enum | Description                                                    |
| ------------------------------------------------- | --------- | ------------ | ---------- | -------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`  | us           |            | Time since system start                                        |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`  | us           |            | Timestamp of the raw observation                               |
| <a id="fld_yaw_ned"></a>yaw_ned                   | `float32` | rad [NED]    |            | Orientation of the target relative to the NED frame [-Pi ; Pi] |
| <a id="fld_yaw_var_ned"></a>yaw_var_ned           | `float32` | rad^2        |            | Orientation uncertainty                                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FiducialMarkerYawReport.msg)

::: details Click here to see original file

```c
# Yaw of a precision-landing target relative to the NED (North, East, Down) frame, reported by a vision pipeline.
#
# Published by: vision pipelines (on-board or off-board over MAVLink TARGET_RELATIVE), decoded in mavlink_receiver.
# Subscribed by: vision_target_estimator (VTEOrientation).

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw observation

float32 yaw_ned # [rad] [@frame NED] Orientation of the target relative to the NED frame [-Pi ; Pi]
float32 yaw_var_ned # [rad^2] Orientation uncertainty
```

:::
