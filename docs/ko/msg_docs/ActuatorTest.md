---
pageClass: is-wide-page
---

# ActuatorTest (UORB message)

**TOPICS:** actuator_test

## Fields

| 명칭                              | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                                                         |
| ------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------- |
| timestamp                       | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                  |
| action                          | `uint8`   |                                                                  |            | one of ACTION\_\*                                                                                    |
| function                        | `uint16`  |                                                                  |            | actuator output function                                                                                                   |
| value                           | `float32` |                                                                  |            | range: [-1, 1], where 1 means maximum positive output, |
| timeout_ms | `uint32`  |                                                                  |            | timeout in ms after which to exit test mode (if 0, do not time out)                                     |

## Constants

| 명칭                                                                                                      | 형식      | Value | 설명                                                                                                                   |
| ------------------------------------------------------------------------------------------------------- | ------- | ----- | -------------------------------------------------------------------------------------------------------------------- |
| <a href="#ACTION_RELEASE_CONTROL"></a> ACTION_RELEASE_CONTROL | `uint8` | 0     | exit test mode for the given function                                                                                |
| <a href="#ACTION_DO_CONTROL"></a> ACTION_DO_CONTROL           | `uint8` | 1     | enable actuator test mode                                                                                            |
| <a href="#FUNCTION_MOTOR1"></a> FUNCTION_MOTOR1                                    | `uint8` | 101   |                                                                                                                      |
| <a href="#MAX_NUM_MOTORS"></a> MAX_NUM_MOTORS                 | `uint8` | 12    |                                                                                                                      |
| <a href="#FUNCTION_SERVO1"></a> FUNCTION_SERVO1                                    | `uint8` | 201   |                                                                                                                      |
| <a href="#MAX_NUM_SERVOS"></a> MAX_NUM_SERVOS                 | `uint8` | 8     |                                                                                                                      |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH             | `uint8` | 16    | > = MAX_NUM_MOTORS to support code in esc_calibration |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActuatorTest.msg)

:::details
Click here to see original file

```c
uint64 timestamp				# time since system start (microseconds)

# Topic to test individual actuator output functions

uint8 ACTION_RELEASE_CONTROL = 0	# exit test mode for the given function
uint8 ACTION_DO_CONTROL = 1			# enable actuator test mode

uint8 FUNCTION_MOTOR1 = 101
uint8 MAX_NUM_MOTORS  = 12
uint8 FUNCTION_SERVO1 = 201
uint8 MAX_NUM_SERVOS  = 8

uint8 action					# one of ACTION_*
uint16 function					# actuator output function
float32 value					# range: [-1, 1], where 1 means maximum positive output,
								# 0 to center servos or minimum motor thrust,
                   				# -1 maximum negative (if not supported by the motors, <0 maps to NaN),
                   				# and NaN maps to disarmed (stop the motors)
uint32 timeout_ms				# timeout in ms after which to exit test mode (if 0, do not time out)

uint8 ORB_QUEUE_LENGTH = 16                     # >= MAX_NUM_MOTORS to support code in esc_calibration
```

:::
