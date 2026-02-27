---
pageClass: is-wide-page
---

# DistanceSensorModeChangeRequest (UORB message)

**TOPICS:** distance_sensormode_changerequest

## Fields

| 명칭                                                       | 형식       | Unit [Frame] | Range/Enum | 설명                                                        |
| -------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                | `uint64` |                                                                  |            | time since system start (microseconds) |
| request_on_off | `uint8`  |                                                                  |            | request to disable/enable the distance sensor             |

## Constants

| 명칭                                                           | 형식      | Value | 설명 |
| ------------------------------------------------------------ | ------- | ----- | -- |
| <a href="#REQUEST_OFF"></a> REQUEST_OFF | `uint8` | 0     |    |
| <a href="#REQUEST_ON"></a> REQUEST_ON   | `uint8` | 1     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DistanceSensorModeChangeRequest.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint8 request_on_off 			# request to disable/enable the distance sensor
uint8 REQUEST_OFF = 0
uint8 REQUEST_ON  = 1
```

:::
