---
pageClass: is-wide-page
---

# HoverThrustEstimate (повідомлення UORB)

**TOPICS:** hover_thrustestimate

## Fields

| Назва                                                                                 | Тип       | Unit [Frame] | Range/Enum | Опис                                                                                                                  |
| ------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                             | `uint64`  |                                                                  |            | time since system start (microseconds)                                                             |
| timestamp_sample                                                 | `uint64`  |                                                                  |            | time of corresponding sensor data last used for this estimate                                                         |
| hover_thrust                                                     | `float32` |                                                                  |            | estimated hover thrust [0.1, 0.9] |
| hover_thrust_var                            | `float32` |                                                                  |            | estimated hover thrust variance                                                                                       |
| accel_innov                                                      | `float32` |                                                                  |            | innovation of the last acceleration fusion                                                                            |
| accel_innov_var                             | `float32` |                                                                  |            | innovation variance of the last acceleration fusion                                                                   |
| accel_innov_test_ratio | `float32` |                                                                  |            | normalized innovation squared test ratio                                                                              |
| accel_noise_var                             | `float32` |                                                                  |            | vertical acceleration noise variance estimated form innovation residual                                               |
| valid                                                                                 | `bool`    |                                                                  |            |                                                                                                                       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/HoverThrustEstimate.msg)

:::details
Click here to see original file

```c
uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # time of corresponding sensor data last used for this estimate

float32 hover_thrust		# estimated hover thrust [0.1, 0.9]
float32 hover_thrust_var	# estimated hover thrust variance

float32 accel_innov		# innovation of the last acceleration fusion
float32 accel_innov_var		# innovation variance of the last acceleration fusion
float32 accel_innov_test_ratio	# normalized innovation squared test ratio

float32 accel_noise_var		# vertical acceleration noise variance estimated form innovation residual

bool valid
```

:::
