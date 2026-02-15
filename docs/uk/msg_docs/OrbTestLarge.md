---
pageClass: is-wide-page
---

# OrbTestLarge (повідомлення UORB)

**TOPICS:** orb_testlarge

## Fields

| Назва     | Тип          | Unit [Frame] | Range/Enum | Опис                                                      |
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
