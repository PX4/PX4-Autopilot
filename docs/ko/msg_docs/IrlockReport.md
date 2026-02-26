---
pageClass: is-wide-page
---

# IrlockReport (UORB message)

IRLOCK_REPORT message data.

**TOPICS:** irlock_report

## Fields

| 명칭                          | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                                                              |
| --------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                   | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                       |
| signature                   | `uint16`  |                                                                  |            |                                                                                                                                 |
| pos_x  | `float32` |                                                                  |            | tan(theta), where theta is the angle between the target and the camera center of projection in camera x-axis |
| pos_y  | `float32` |                                                                  |            | tan(theta), where theta is the angle between the target and the camera center of projection in camera y-axis |
| size_x | `float32` |                                                                  |            | /\*\* size of target along camera x-axis in units of tan(theta) \*\*/                                        |
| size_y | `float32` |                                                                  |            | /\*\* size of target along camera y-axis in units of tan(theta) \*\*/                                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/IrlockReport.msg)

:::details
Click here to see original file

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
