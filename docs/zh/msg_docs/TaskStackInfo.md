---
pageClass: is-wide-page
---

# TaskStackInfo (UORB message)

stack information for a single running process.

**TOPICS:** task_stackinfo

## Fields

| 参数名                             | 类型         | Unit [Frame] | Range/Enum | 描述                                                        |
| ------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                       | `uint64`   |                                                                  |            | time since system start (microseconds) |
| stack_free | `uint16`   |                                                                  |            |                                                           |
| task_name  | `char[24]` |                                                                  |            |                                                           |

## Constants

| 参数名                                                                                         | 类型      | 值 | 描述 |
| ------------------------------------------------------------------------------------------- | ------- | - | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 2 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TaskStackInfo.msg)

:::details
Click here to see original file

```c
# stack information for a single running process

uint64 timestamp		# time since system start (microseconds)

uint16 stack_free
char[24] task_name

uint8 ORB_QUEUE_LENGTH = 2
```

:::
