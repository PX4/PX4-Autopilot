---
pageClass: is-wide-page
---

# HeaterStatus (UORB message)

**TOPICS:** heater_status

## Fields

| 명칭                                                                                     | 형식        | Unit [Frame] | Range/Enum | 설명                                                        |
| -------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                                              | `uint64`  |                                                                  |            | time since system start (microseconds) |
| device_id                                                         | `uint32`  |                                                                  |            |                                                           |
| heater_on                                                         | `bool`    |                                                                  |            |                                                           |
| temperature_target_met                       | `bool`    |                                                                  |            |                                                           |
| temperature_sensor                                                | `float32` |                                                                  |            |                                                           |
| temperature_target                                                | `float32` |                                                                  |            |                                                           |
| controller_period_usec                       | `uint32`  |                                                                  |            |                                                           |
| controller_time_on_usec | `uint32`  |                                                                  |            |                                                           |
| proportional_value                                                | `float32` |                                                                  |            |                                                           |
| integrator_value                                                  | `float32` |                                                                  |            |                                                           |
| feed_forward_value                           | `float32` |                                                                  |            |                                                           |
| mode                                                                                   | `uint8`   |                                                                  |            |                                                           |

## Constants

| 명칭                                                         | 형식      | Value | 설명 |
| ---------------------------------------------------------- | ------- | ----- | -- |
| <a href="#MODE_GPIO"></a> MODE_GPIO   | `uint8` | 1     |    |
| <a href="#MODE_PX4IO"></a> MODE_PX4IO | `uint8` | 2     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/HeaterStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)

uint32 device_id

bool heater_on
bool temperature_target_met

float32 temperature_sensor
float32 temperature_target

uint32 controller_period_usec
uint32 controller_time_on_usec

float32 proportional_value
float32 integrator_value
float32 feed_forward_value

uint8 MODE_GPIO  = 1
uint8 MODE_PX4IO = 2
uint8 mode
```

:::
