---
pageClass: is-wide-page
---

# ButtonEvent (UORB message)

**TOPICS:** button_event safety_button

## Fields

| Name      | Type     | Unit [Frame] | Range/Enum | Description                            |
| --------- | -------- | ------------ | ---------- | -------------------------------------- |
| timestamp | `uint64` |              |            | time since system start (microseconds) |
| triggered | `bool`   |              |            | Set to true if the event is triggered  |

## Constants

| Name                                            | Type    | Value | Description |
| ----------------------------------------------- | ------- | ----- | ----------- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 2     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ButtonEvent.msg)

::: details Click here to see original file

```c
uint64 timestamp			# time since system start (microseconds)
bool triggered				# Set to true if the event is triggered

# TOPICS button_event safety_button

uint8 ORB_QUEUE_LENGTH = 2
```

:::
