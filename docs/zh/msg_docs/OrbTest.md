---
pageClass: is-wide-page
---

# OrbTest (UORB message)

**TOPICS:** orb_test orb_multitest

## Fields

| 参数名       | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| --------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64` |                                                                  |            | time since system start (microseconds) |
| val       | `int32`  |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OrbTest.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

int32 val

# TOPICS orb_test orb_multitest
```

:::
