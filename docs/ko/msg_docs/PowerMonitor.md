---
pageClass: is-wide-page
---

# PowerMonitor (UORB message)

power monitor message.

**TOPICS:** power_monitor

## Fields

| 명칭                             | 형식        | Unit [Frame] | Range/Enum | 설명                                                        |
| ------------------------------ | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                      | `uint64`  |                                                                  |            | Time since system start (microseconds) |
| voltage_v | `float32` |                                                                  |            | Voltage in volts, 0 if unknown                            |
| current_a | `float32` |                                                                  |            | Current in amperes, -1 if unknown                         |
| power_w   | `float32` |                                                                  |            | power in watts, -1 if unknown                             |
| rconf                          | `int16`   |                                                                  |            |                                                           |
| rsv                            | `int16`   |                                                                  |            |                                                           |
| rbv                            | `int16`   |                                                                  |            |                                                           |
| rp                             | `int16`   |                                                                  |            |                                                           |
| rc                             | `int16`   |                                                                  |            |                                                           |
| rcal                           | `int16`   |                                                                  |            |                                                           |
| me                             | `int16`   |                                                                  |            |                                                           |
| al                             | `int16`   |                                                                  |            |                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PowerMonitor.msg)

:::details
Click here to see original file

```c
# power monitor message

uint64 timestamp			# Time since system start (microseconds)

float32 voltage_v			# Voltage in volts, 0 if unknown
float32 current_a		    # Current in amperes, -1 if unknown
float32 power_w		        # power in watts, -1 if unknown
int16 rconf
int16 rsv
int16 rbv
int16 rp
int16 rc
int16 rcal
int16 me
int16 al
```

:::
