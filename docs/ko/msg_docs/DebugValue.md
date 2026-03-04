---
pageClass: is-wide-page
---

# DebugValue (UORB message)

**TOPICS:** debug_value

## Fields

| 명칭        | 형식        | Unit [Frame] | Range/Enum | 설명                                                        |
| --------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64`  |                                                                  |            | time since system start (microseconds) |
| ind       | `int8`    |                                                                  |            | index of debug variable                                   |
| value     | `float32` |                                                                  |            | the value to send as debug output                         |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugValue.msg)

:::details
Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)
int8 ind                # index of debug variable
float32 value           # the value to send as debug output
```

:::
