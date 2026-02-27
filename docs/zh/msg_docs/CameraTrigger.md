---
pageClass: is-wide-page
---

# CameraTrigger (UORB message)

**TOPICS:** camera_trigger

## Fields

| 参数名                                | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| ---------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                          | `uint64` |                                                                  |            | time since system start (microseconds) |
| timestamp_utc | `uint64` |                                                                  |            | UTC timestamp                                             |
| seq                                | `uint32` |                                                                  |            | Image sequence number                                     |
| feedback                           | `bool`   |                                                                  |            | Trigger feedback from camera                              |

## Constants

| 参数名                                                                                         | 类型       | 值 | 描述 |
| ------------------------------------------------------------------------------------------- | -------- | - | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint32` | 2 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CameraTrigger.msg)

:::details
Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)
uint64 timestamp_utc # UTC timestamp

uint32 seq		# Image sequence number
bool feedback	# Trigger feedback from camera

uint32 ORB_QUEUE_LENGTH = 2
```

:::
