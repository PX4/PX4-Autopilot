---
pageClass: is-wide-page
---

# DebugVect (UORB message)

**TOPICS:** debug_vect

## Fields

| 参数名       | 类型         | Unit [Frame] | Range/Enum | 描述                                                        |
| --------- | ---------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64`   |                                                                  |            | time since system start (microseconds) |
| name      | `char[10]` |                                                                  |            | max. 10 characters as key / name          |
| x         | `float32`  |                                                                  |            | x value                                                   |
| y         | `float32`  |                                                                  |            | y value                                                   |
| z         | `float32`  |                                                                  |            | z value                                                   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugVect.msg)

:::details
Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)
char[10] name           # max. 10 characters as key / name
float32 x               # x value
float32 y               # y value
float32 z               # z value
```

:::
