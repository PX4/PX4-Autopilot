---
pageClass: is-wide-page
---

# PwmInput (UORB message)

**TOPICS:** pwm_input

## Fields

| 명칭                               | 형식       | Unit [Frame] | Range/Enum | 설명                                                              |
| -------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------- |
| timestamp                        | `uint64` |                                                                  |            | Time since system start (microseconds)       |
| error_count | `uint64` |                                                                  |            | Timer overcapture error flag (AUX5 or MAIN5) |
| pulse_width | `uint32` |                                                                  |            | Pulse width, timer counts (microseconds)     |
| period                           | `uint32` |                                                                  |            | Period, timer counts (microseconds)          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PwmInput.msg)

:::details
Click here to see original file

```c
uint64 timestamp   # Time since system start (microseconds)
uint64 error_count # Timer overcapture error flag (AUX5 or MAIN5)
uint32 pulse_width # Pulse width, timer counts (microseconds)
uint32 period      # Period, timer counts (microseconds)
```

:::
