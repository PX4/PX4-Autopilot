---
pageClass: is-wide-page
---

# ButtonEvent (UORB message)

**TOPICS:** button_event safety_button

## Fields

| 명칭        | 형식       | Unit [Frame] | Range/Enum | 설명                                                        |
| --------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64` |                                                                  |            | time since system start (microseconds) |
| triggered | `bool`   |                                                                  |            | Set to true if the event is triggered                     |

## Constants

| 명칭                                                                                          | 형식      | Value | 설명 |
| ------------------------------------------------------------------------------------------- | ------- | ----- | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 2     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ButtonEvent.msg)

:::details
Click here to see original file

```c
uint64 timestamp			# time since system start (microseconds)
bool triggered				# Set to true if the event is triggered

# TOPICS button_event safety_button

uint8 ORB_QUEUE_LENGTH = 2
```

:::
