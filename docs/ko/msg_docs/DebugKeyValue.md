---
pageClass: is-wide-page
---

# DebugKeyValue (UORB message)

**TOPICS:** debug_keyvalue

## Fields

| 명칭        | 형식         | Unit [Frame] | Range/Enum | 설명                                                        |
| --------- | ---------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64`   |                                                                  |            | time since system start (microseconds) |
| 키         | `char[10]` |                                                                  |            | max. 10 characters as key / name          |
| value     | `float32`  |                                                                  |            | the value to send as debug output                         |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugKeyValue.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
char[10] key			# max. 10 characters as key / name
float32 value			# the value to send as debug output
```

:::
