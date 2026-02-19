---
pageClass: is-wide-page
---

# GpioIn (повідомлення UORB)

GPIO mask and state.

**TOPICS:** gpio_in

## Fields

| Назва                          | Тип      | Unit [Frame] | Range/Enum | Опис                                                      |
| ------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                      | `uint64` |                                                                  |            | time since system start (microseconds) |
| device_id | `uint32` |                                                                  |            | Device id                                                 |
| state                          | `uint32` |                                                                  |            | pin state mask                                            |

## Constants

| Назва                                                            | Тип     | Значення | Опис |
| ---------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#MAX_INSTANCES"></a> MAX_INSTANCES | `uint8` | 8        |      |

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
