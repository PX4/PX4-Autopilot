---
pageClass: is-wide-page
---

# ActuatorServos (UORB message)

Servo control message.

Normalised output setpoint for up to 8 servos.
Published by the vehicle's allocation and consumed by the actuator output drivers.

**TOPICS:** actuator_servos

## Fields

| 명칭                                    | 형식           | Unit [Frame] | Range/Enum                                                                   | 설명                                                                                                                                                                                                                                                                     |
| ------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`     | us                                                               |                                                                              | Time since system start                                                                                                                                                                                                                                                |
| timestamp_sample | `uint64`     | us                                                               |                                                                              | Sampling timestamp of the data this control response is based on                                                                                                                                                                                                       |
| control                               | `float32[8]` |                                                                  | [-1 : 1] | Normalized output. 1 means maximum positive position. -1 maximum negative position (if not supported by the output, <0 maps to NaN). NaN maps to disarmed. |

## Constants

| 명칭                                                                   | 형식       | Value | 설명 |
| -------------------------------------------------------------------- | -------- | ----- | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0     |    |
| <a href="#NUM_CONTROLS"></a> NUM_CONTROLS       | `uint8`  | 8     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ActuatorServos.msg)

:::details
Click here to see original file

```c
# Servo control message
#
# Normalised output setpoint for up to 8 servos.
# Published by the vehicle's allocation and consumed by the actuator output drivers.

uint32 MESSAGE_VERSION = 0

uint64 timestamp         # [us] Time since system start
uint64 timestamp_sample  # [us] Sampling timestamp of the data this control response is based on

uint8 NUM_CONTROLS = 8  #
float32[8] control      # [-] [@range -1, 1] Normalized output. 1 means maximum positive position. -1 maximum negative position (if not supported by the output, <0 maps to NaN). NaN maps to disarmed.
```

:::
