---
pageClass: is-wide-page
---

# GimbalManagerStatus (UORB message)

**TOPICS:** gimbal_managerstatus

## Fields

| 参数名                                                                | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| ------------------------------------------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                          | `uint64` |                                                                  |            | time since system start (microseconds) |
| flags                                                              | `uint32` |                                                                  |            |                                                           |
| gimbal_device_id         | `uint8`  |                                                                  |            |                                                           |
| primary_control_sysid    | `uint8`  |                                                                  |            |                                                           |
| primary_control_compid   | `uint8`  |                                                                  |            |                                                           |
| secondary_control_sysid  | `uint8`  |                                                                  |            |                                                           |
| secondary_control_compid | `uint8`  |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalManagerStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint32 flags
uint8 gimbal_device_id
uint8 primary_control_sysid
uint8 primary_control_compid
uint8 secondary_control_sysid
uint8 secondary_control_compid
```

:::
