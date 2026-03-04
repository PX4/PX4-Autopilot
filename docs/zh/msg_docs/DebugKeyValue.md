---
pageClass: is-wide-page
---

# DebugKeyValue (UORB message)

**TOPICS:** debug_keyvalue

## Fields

| 参数名       | 类型         | Unit [Frame] | Range/Enum | 描述                                                        |
| --------- | ---------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64`   |                                                                  |            | time since system start (microseconds) |
| 键         | `char[10]` |                                                                  |            | max. 10 characters as key / name          |
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
