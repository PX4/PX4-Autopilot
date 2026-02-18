---
pageClass: is-wide-page
---

# QshellRetval (UORB message)

**TOPICS:** qshell_retval

## Fields

| 명칭                                   | 형식       | Unit [Frame] | Range/Enum | 설명                                                        |
| ------------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                            | `uint64` |                                                                  |            | time since system start (microseconds) |
| return_value    | `int32`  |                                                                  |            |                                                           |
| return_sequence | `uint32` |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/QshellRetval.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
int32 return_value
uint32 return_sequence
```

:::
