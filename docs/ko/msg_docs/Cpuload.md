---
pageClass: is-wide-page
---

# Cpuload (UORB message)

**TOPICS:** cpuload

## Fields

| 명칭                             | 형식        | Unit [Frame] | Range/Enum | 설명                                                        |
| ------------------------------ | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                      | `uint64`  |                                                                  |            | time since system start (microseconds) |
| load                           | `float32` |                                                                  |            | processor load from 0 to 1                                |
| ram_usage | `float32` |                                                                  |            | RAM usage from 0 to 1                                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/Cpuload.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
float32 load                    # processor load from 0 to 1
float32 ram_usage		# RAM usage from 0 to 1
```

:::
