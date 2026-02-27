---
pageClass: is-wide-page
---

# GpioOut (UORB message)

GPIO mask and state.

**TOPICS:** gpio_out

## Fields

| 参数名                            | 类型       | Unit [Frame] | Range/Enum | 描述                                                        |
| ------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                      | `uint64` |                                                                  |            | time since system start (microseconds) |
| device_id | `uint32` |                                                                  |            | Device id                                                 |
| mask                           | `uint32` |                                                                  |            | pin mask                                                  |
| state                          | `uint32` |                                                                  |            | pin state mask                                            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpioOut.msg)

:::details
Click here to see original file

```c
# GPIO mask and state

uint64 timestamp			# time since system start (microseconds)
uint32 device_id			# Device id

uint32 mask				# pin mask
uint32 state				# pin state mask
```

:::
