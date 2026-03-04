---
pageClass: is-wide-page
---

# MavlinkLog (UORB message)

**TOPICS:** mavlink_log

## Fields

| 명칭        | 형식          | Unit [Frame] | Range/Enum | 설명                                                                          |
| --------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------- |
| timestamp | `uint64`    |                                                                  |            | time since system start (microseconds)                   |
| text      | `char[127]` |                                                                  |            |                                                                             |
| severity  | `uint8`     |                                                                  |            | log level (same as in the linux kernel, starting with 0) |

## Constants

| 명칭                                                                                          | 형식      | Value | 설명 |
| ------------------------------------------------------------------------------------------- | ------- | ----- | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 8     |    |

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
