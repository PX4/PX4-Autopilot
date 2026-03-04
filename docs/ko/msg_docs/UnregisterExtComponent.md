---
pageClass: is-wide-page
---

# UnregisterExtComponent (UORB message)

**TOPICS:** unregister_extcomponent

## Fields

| 명칭                                                         | 형식         | Unit [Frame] | Range/Enum | 설명                                                                     |
| ---------------------------------------------------------- | ---------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------- |
| timestamp                                                  | `uint64`   |                                                                  |            | time since system start (microseconds)              |
| name                                                       | `char[25]` |                                                                  |            | either the mode name, or component name                                |
| arming_check_id  | `int8`     |                                                                  |            | arming check registration ID (-1 if not registered) |
| mode_id                               | `int8`     |                                                                  |            | assigned mode ID (-1 if not registered)             |
| mode_executor_id | `int8`     |                                                                  |            | assigned mode executor ID (-1 if not registered)    |

## Constants

| 명칭                                                                   | 형식       | Value | 설명 |
| -------------------------------------------------------------------- | -------- | ----- | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/UnregisterExtComponent.msg)

:::details
Click here to see original file

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

char[25] name                      # either the mode name, or component name

int8 arming_check_id      # arming check registration ID (-1 if not registered)
int8 mode_id              # assigned mode ID (-1 if not registered)
int8 mode_executor_id     # assigned mode executor ID (-1 if not registered)
```

:::
