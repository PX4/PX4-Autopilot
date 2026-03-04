---
pageClass: is-wide-page
---

# GpioIn (UORB message)

GPIO mask and state.

**TOPICS:** gpio_in

## Fields

| 명칭                             | 형식       | Unit [Frame] | Range/Enum | 설명                                                        |
| ------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                      | `uint64` |                                                                  |            | time since system start (microseconds) |
| device_id | `uint32` |                                                                  |            | Device id                                                 |
| state                          | `uint32` |                                                                  |            | pin state mask                                            |

## Constants

| 명칭                                                               | 형식      | Value | 설명 |
| ---------------------------------------------------------------- | ------- | ----- | -- |
| <a href="#MAX_INSTANCES"></a> MAX_INSTANCES | `uint8` | 8     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpioIn.msg)

:::details
Click here to see original file

```c
# GPIO mask and state
uint8 MAX_INSTANCES = 8

uint64 timestamp			# time since system start (microseconds)
uint32 device_id			# Device id

uint32 state				# pin state mask
```

:::
