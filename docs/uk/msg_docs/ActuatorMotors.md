---
pageClass: is-wide-page
---

# ActuatorMotors (повідомлення UORB)

Motor control message.

Normalised thrust setpoint for up to 12 motors.
Published by the vehicle's allocation and consumed by the ESC protocol drivers e.g. PWM, DSHOT, UAVCAN.

**TOPICS:** actuator_motors

## Fields

| Назва                                 | Тип           | Unit [Frame] | Range/Enum                                                                   | Опис                                                                                                                                                                                                                                                                  |
| ------------------------------------- | ------------- | ---------------------------------------------------------------- | ---------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                             | `uint64`      | us                                                               |                                                                              | Time since system start                                                                                                                                                                                                                                               |
| timestamp_sample | `uint64`      | us                                                               |                                                                              | Sampling timestamp of the data this control response is based on                                                                                                                                                                                                      |
| reversible_flags | `uint16`      |                                                                  |                                                                              | Bitset indicating which motors are configured to be reversible                                                                                                                                                                                                        |
| control                               | `float32[12]` |                                                                  | [-1 : 1] | Normalized thrust. where 1 means maximum positive thrust, -1 maximum negative (if not supported by the output, <0 maps to NaN). NaN maps to disarmed (stop the motors) |

## Constants

| Назва                                                                                                       | Тип      | Значення | Опис |
| ----------------------------------------------------------------------------------------------------------- | -------- | -------- | ---- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                        | `uint32` | 0        |      |
| <a href="#ACTUATOR_FUNCTION_MOTOR1"></a> ACTUATOR_FUNCTION_MOTOR1 | `uint8`  | 101      |      |
| <a href="#NUM_CONTROLS"></a> NUM_CONTROLS                                              | `uint8`  | 12       |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ActuatorMotors.msg)

:::details
Click here to see original file

```c
# Motor control message
#
# Normalised thrust setpoint for up to 12 motors.
# Published by the vehicle's allocation and consumed by the ESC protocol drivers e.g. PWM, DSHOT, UAVCAN.

uint32 MESSAGE_VERSION = 0

uint64 timestamp         # [us] Time since system start
uint64 timestamp_sample  # [us] Sampling timestamp of the data this control response is based on

uint16 reversible_flags  # [-] Bitset indicating which motors are configured to be reversible

uint8 ACTUATOR_FUNCTION_MOTOR1 = 101 #

uint8 NUM_CONTROLS = 12  #
float32[12] control      # [@range -1, 1] Normalized thrust. where 1 means maximum positive thrust, -1 maximum negative (if not supported by the output, <0 maps to NaN). NaN maps to disarmed (stop the motors)
```

:::
