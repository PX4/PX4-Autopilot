---
pageClass: is-wide-page
---

# OrbTestLarge (UORB message)

**TOPICS:** orb_testlarge

## Fields

| 参数名       | 类型           | Unit [Frame] | Range/Enum | 描述                                                        |
| --------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp | `uint64`     |                                                                  |            | time since system start (microseconds) |
| val       | `int32`      |                                                                  |            |                                                           |
| junk      | `uint8[512]` |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OrbTestLarge.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

int32 val

uint8[512] junk
```

:::
