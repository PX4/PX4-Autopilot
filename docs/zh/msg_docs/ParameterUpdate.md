---
pageClass: is-wide-page
---

# ParameterUpdate (UORB message)

This message is used to notify the system about one or more parameter changes.

**TOPICS:** parameter_update

## Fields

| 参数名                                 | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| ----------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                           | `uint64` |                                                                  |            | time since system start (microseconds) |
| instance                            | `uint32` |                                                                  |            | Instance count - constantly incrementing                  |
| get_count      | `uint32` |                                                                  |            |                                                           |
| set_count      | `uint32` |                                                                  |            |                                                           |
| find_count     | `uint32` |                                                                  |            |                                                           |
| export_count   | `uint32` |                                                                  |            |                                                           |
| active                              | `uint16` |                                                                  |            |                                                           |
| changed                             | `uint16` |                                                                  |            |                                                           |
| custom_default | `uint16` |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ParameterUpdate.msg)

:::details
Click here to see original file

```c
# This message is used to notify the system about one or more parameter changes

uint64 timestamp		# time since system start (microseconds)

uint32 instance		# Instance count - constantly incrementing

uint32 get_count
uint32 set_count
uint32 find_count
uint32 export_count

uint16 active
uint16 changed
uint16 custom_default
```

:::
