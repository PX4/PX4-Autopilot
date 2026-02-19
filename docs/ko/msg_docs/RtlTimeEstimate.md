---
pageClass: is-wide-page
---

# RtlTimeEstimate (UORB message)

**TOPICS:** rtl_timeestimate

## Fields

| 명칭                                                           | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                                                                    |
| ------------------------------------------------------------ | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                    | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                             |
| valid                                                        | `bool`    |                                                                  |            | Flag indicating whether the time estiamtes are valid                                                                                  |
| time_estimate                           | `float32` | s                                                                |            | Estimated time for RTL                                                                                                                |
| safe_time_estimate | `float32` | s                                                                |            | Same as time_estimate, but with safety factor and safety margin included (factor\*t + margin) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RtlTimeEstimate.msg)

:::details
Click here to see original file

```c
uint64 timestamp # time since system start (microseconds)

bool valid			# Flag indicating whether the time estiamtes are valid
float32 time_estimate		# [s] Estimated time for RTL
float32 safe_time_estimate	# [s] Same as time_estimate, but with safety factor and safety margin included (factor*t + margin)
```

:::
