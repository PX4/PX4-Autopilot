---
pageClass: is-wide-page
---

# GimbalControls (UORB message)

**TOPICS:** gimbal_controls

## Fields

| 명칭                                    | 형식           | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                                                               |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                                                                        |
| timestamp_sample | `uint64`     |                                                                  |            | the timestamp the data this control response is based on was sampled                                                                                                                                             |
| control                               | `float32[3]` |                                                                  |            | Normalized output. 1 means maximum positive position. -1 maximum negative position. 0 means no deflection. NaN maps to disarmed. |

## Constants

| 명칭                                                           | 형식      | Value | 설명 |
| ------------------------------------------------------------ | ------- | ----- | -- |
| <a href="#INDEX_ROLL"></a> INDEX_ROLL   | `uint8` | 0     |    |
| <a href="#INDEX_PITCH"></a> INDEX_PITCH | `uint8` | 1     |    |
| <a href="#INDEX_YAW"></a> INDEX_YAW     | `uint8` | 2     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalControls.msg)

:::details
Click here to see original file

```c
uint64 timestamp			# time since system start (microseconds)
uint8 INDEX_ROLL = 0
uint8 INDEX_PITCH = 1
uint8 INDEX_YAW = 2

uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled
float32[3] control	# Normalized output. 1 means maximum positive position. -1 maximum negative position. 0 means no deflection. NaN maps to disarmed.
```

:::
