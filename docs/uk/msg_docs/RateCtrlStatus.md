---
pageClass: is-wide-page
---

# RateCtrlStatus (UORB message)

**TOPICS:** rate_ctrlstatus

## Fields

| Назва                                 | Тип       | Unit [Frame] | Range/Enum | Опис                                                      |
| ------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                             | `uint64`  |                                                                  |            | time since system start (microseconds) |
| rollspeed_integ  | `float32` |                                                                  |            |                                                           |
| pitchspeed_integ | `float32` |                                                                  |            |                                                           |
| yawspeed_integ   | `float32` |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RateCtrlStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
```

:::
