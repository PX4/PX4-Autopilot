---
pageClass: is-wide-page
---

# Rpm (UORB message)

**TOPICS:** rpm

## Fields

| Назва                             | Тип       | Unit [Frame] | Range/Enum | Опис                                                      |
| --------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                         | `uint64`  |                                                                  |            | time since system start (microseconds) |
| rpm_estimate | `float32` |                                                                  |            | filtered revolutions per minute                           |
| rpm_raw      | `float32` |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Rpm.msg)

:::details
Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)

# rpm values of 0.0 mean within a timeout there is no movement measured
float32 rpm_estimate # filtered revolutions per minute
float32 rpm_raw
```

:::
