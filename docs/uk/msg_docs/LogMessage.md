---
pageClass: is-wide-page
---

# LogMessage (повідомлення UORB)

A logging message, output with PX4_WARN, PX4_ERR, PX4_INFO.

**TOPICS:** log_message

## Fields

| Назва     | Тип         | Unit [Frame] | Range/Enum | Опис                                                                        |
| --------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------- |
| timestamp | `uint64`    |                                                                  |            | time since system start (microseconds)                   |
| severity  | `uint8`     |                                                                  |            | log level (same as in the linux kernel, starting with 0) |
| text      | `char[127]` |                                                                  |            |                                                                             |

## Constants

| Назва                                                                                       | Тип     | Значення | Опис |
| ------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LogMessage.msg)

:::details
Click here to see original file

```c
# A logging message, output with PX4_WARN, PX4_ERR, PX4_INFO

uint64 timestamp		# time since system start (microseconds)

uint8 severity # log level (same as in the linux kernel, starting with 0)
char[127] text

uint8 ORB_QUEUE_LENGTH = 4
```

:::
