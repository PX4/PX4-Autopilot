---
pageClass: is-wide-page
---

# EstimatorStates (повідомлення UORB)

**TOPICS:** estimator_states

## Fields

| Назва                                 | Тип           | Unit [Frame] | Range/Enum | Опис                                                            |
| ------------------------------------- | ------------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------- |
| timestamp                             | `uint64`      |                                                                  |            | time since system start (microseconds)       |
| timestamp_sample | `uint64`      |                                                                  |            | the timestamp of the raw data (microseconds) |
| states                                | `float32[25]` |                                                                  |            | Internal filter states                                          |
| n_states         | `uint8`       |                                                                  |            | Number of states effectively used                               |
| covariances                           | `float32[24]` |                                                                  |            | Diagonal Elements of Covariance Matrix                          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorStates.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[25] states		# Internal filter states
uint8 n_states		# Number of states effectively used

float32[24] covariances	# Diagonal Elements of Covariance Matrix
```

:::
