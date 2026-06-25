---
pageClass: is-wide-page
---

# IrlockReport (UORB message)

IRLOCK_REPORT message data.

**TOPICS:** irlock_report

## Fields

| Name                                | Type      | Unit [Frame] | Range/Enum | Description                                                                                                  |
| ----------------------------------- | --------- | ------------ | ---------- | ------------------------------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp | `uint64`  |              |            | time since system start (microseconds)                                                                       |
| <a id="fld_signature"></a>signature | `uint16`  |              |            |
| <a id="fld_pos_x"></a>pos_x         | `float32` |              |            | tan(theta), where theta is the angle between the target and the camera center of projection in camera x-axis |
| <a id="fld_pos_y"></a>pos_y         | `float32` |              |            | tan(theta), where theta is the angle between the target and the camera center of projection in camera y-axis |
| <a id="fld_size_x"></a>size_x       | `float32` |              |            | /** size of target along camera x-axis in units of tan(theta) **/                                            |
| <a id="fld_size_y"></a>size_y       | `float32` |              |            | /** size of target along camera y-axis in units of tan(theta) **/                                            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/IrlockReport.msg)

::: details Click here to see original file

```c
# IRLOCK_REPORT message data

uint64 timestamp		# time since system start (microseconds)

uint16 signature

# When looking along the optical axis of the camera, x points right, y points down, and z points along the optical axis.
float32 pos_x # tan(theta), where theta is the angle between the target and the camera center of projection in camera x-axis
float32 pos_y # tan(theta), where theta is the angle between the target and the camera center of projection in camera y-axis
float32 size_x #/** size of target along camera x-axis in units of tan(theta) **/
float32 size_y #/** size of target along camera y-axis in units of tan(theta) **/
```

:::
