---
pageClass: is-wide-page
---

# MavlinkLog (UORB message)

**TOPICS:** mavlink_log

## Fields

| 参数名       | 类型          | Unit [Frame] | Range/Enum | 描述                                                                          |
| --------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------- |
| timestamp | `uint64`    |                                                                  |            | time since system start (microseconds)                   |
| text      | `char[127]` |                                                                  |            |                                                                             |
| severity  | `uint8`     |                                                                  |            | log level (same as in the linux kernel, starting with 0) |

## Constants

| 参数名                                                                                         | 类型      | 值 | 描述 |
| ------------------------------------------------------------------------------------------- | ------- | - | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 8 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/MavlinkLog.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

char[127] text
uint8 severity # log level (same as in the linux kernel, starting with 0)

uint8 ORB_QUEUE_LENGTH = 8
```

:::
