---
pageClass: is-wide-page
---

# CameraStatus (UORB message)

**TOPICS:** camera_status

## Fields

| 参数名                                                      | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| -------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                | `uint64` |                                                                  |            | time since system start (microseconds) |
| active_sys_id  | `uint8`  |                                                                  |            | mavlink system id of the currently active camera          |
| active_comp_id | `uint8`  |                                                                  |            | mavlink component id of currently active camera           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CameraStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint8 active_sys_id		# mavlink system id of the currently active camera
uint8 active_comp_id 	# mavlink component id of currently active camera
```

:::
