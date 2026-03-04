---
pageClass: is-wide-page
---

# OrbTestMedium (UORB message)

**TOPICS:** orb_test_medium orb_test_medium_multi orb_test_medium_wrap_around orb_test_medium_queue orb_test_medium_queue_poll

## Fields

| 参数名       | 类型          | Unit [Frame] | Range/Enum | 描述                                                        |
| --------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64`    |                                                                  |            | time since system start (microseconds) |
| val       | `int32`     |                                                                  |            |                                                           |
| junk      | `uint8[64]` |                                                                  |            |                                                           |

## Constants

| 参数名                                                                                         | 类型      | 值  | 描述 |
| ------------------------------------------------------------------------------------------- | ------- | -- | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 16 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OrbTestMedium.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

int32 val

uint8[64] junk

uint8 ORB_QUEUE_LENGTH = 16

# TOPICS orb_test_medium orb_test_medium_multi orb_test_medium_wrap_around orb_test_medium_queue orb_test_medium_queue_poll
```

:::
