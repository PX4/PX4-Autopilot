---
pageClass: is-wide-page
---

# EscReport (UORB повідомлення)

**TOPICS:** esc_report

## Fields

| Назва                                  | Тип       | Unit [Frame] | Range/Enum                                                                    | Опис                                                                                                                 |
| -------------------------------------- | --------- | ---------------------------------------------------------------- | ----------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| timestamp                              | `uint64`  | us                                                               |                                                                               | Time since system start                                                                                              |
| esc_errorcount    | `uint32`  |                                                                  |                                                                               | Number of reported errors by ESC - if supported                                                                      |
| esc_rpm           | `int32`   | rpm                                                              |                                                                               | Motor RPM, negative for reverse rotation - if supported                                                              |
| esc_voltage       | `float32` | V                                                                |                                                                               | Voltage measured from current ESC - if supported                                                                     |
| esc_current       | `float32` | A                                                                |                                                                               | Current measured from current ESC - if supported                                                                     |
| esc_temperature   | `float32` | degC                                                             |                                                                               | Temperature measured from current ESC - if supported                                                                 |
| motor_temperature | `int16`   | degC                                                             |                                                                               | Temperature measured from current motor - if supported                                                               |
| esc_state         | `uint8`   |                                                                  |                                                                               | State of ESC - depend on Vendor                                                                                      |
| actuator_function | `uint8`   |                                                                  |                                                                               | Actuator output function (one of Motor1...MotorN) |
| failures                               | `uint16`  |                                                                  | [FAILURE](#FAILURE)                                                           | Bitmask to indicate the internal ESC faults                                                                          |
| esc_power         | `int8`    | %                                                                | [0 : 100] | Applied power (negative values reserved)                                                          |

## Enums

### FAILURE {#FAILURE}

| Назва                                                                                                                                      | Тип     | Значення | Опис                                                                                                                                                                                 |
| ------------------------------------------------------------------------------------------------------------------------------------------ | ------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="#FAILURE_OVER_CURRENT"></a> FAILURE_OVER_CURRENT                                          | `uint8` | 0        | (1 << 0)                                                                                                        |
| <a id="#FAILURE_OVER_VOLTAGE"></a> FAILURE_OVER_VOLTAGE                                          | `uint8` | 1        | (1 << 1)                                                                                                        |
| <a id="#FAILURE_MOTOR_OVER_TEMPERATURE"></a> FAILURE_MOTOR_OVER_TEMPERATURE | `uint8` | 2        | (1 << 2)                                                                                                        |
| <a id="#FAILURE_OVER_RPM"></a> FAILURE_OVER_RPM                                                  | `uint8` | 3        | (1 << 3)                                                                                                        |
| <a id="#FAILURE_INCONSISTENT_CMD"></a> FAILURE_INCONSISTENT_CMD                                  | `uint8` | 4        | (1 << 4) Set if ESC received an inconsistent command (i.e out of boundaries) |
| <a id="#FAILURE_MOTOR_STUCK"></a> FAILURE_MOTOR_STUCK                                            | `uint8` | 5        | (1 << 5)                                                                                                        |
| <a id="#FAILURE_GENERIC"></a> FAILURE_GENERIC                                                                         | `uint8` | 6        | (1 << 6)                                                                                                        |
| <a id="#FAILURE_MOTOR_WARN_TEMPERATURE"></a> FAILURE_MOTOR_WARN_TEMPERATURE | `uint8` | 7        | (1 << 7)                                                                                                        |
| <a id="#FAILURE_WARN_ESC_TEMPERATURE"></a> FAILURE_WARN_ESC_TEMPERATURE     | `uint8` | 8        | (1 << 8)                                                                                                        |
| <a id="#FAILURE_OVER_ESC_TEMPERATURE"></a> FAILURE_OVER_ESC_TEMPERATURE     | `uint8` | 9        | (1 << 9)                                                                                                        |

## Constants

| Назва                                                                                                                                | Тип     | Значення | Опис                                                                                                                     |
| ------------------------------------------------------------------------------------------------------------------------------------ | ------- | -------- | ------------------------------------------------------------------------------------------------------------------------ |
| <a id="#ACTUATOR_FUNCTION_MOTOR1"></a> ACTUATOR_FUNCTION_MOTOR1                            | `uint8` | 101      |                                                                                                                          |
| <a id="#ACTUATOR_FUNCTION_MOTOR_MAX"></a> ACTUATOR_FUNCTION_MOTOR_MAX | `uint8` | 112      | output_functions.yaml Motor.start + Motor.count - 1 |
| <a id="#ESC_FAILURE_COUNT"></a> ESC_FAILURE_COUNT                                          | `uint8` | 10       | Counter - keep it as last element!                                                                                       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EscReport.msg)

:::details
Click here to see original file

```c
uint64 timestamp # [us] Time since system start

uint32 esc_errorcount # [-] Number of reported errors by ESC - if supported
int32 esc_rpm # [rpm] Motor RPM, negative for reverse rotation - if supported
float32 esc_voltage # [V] Voltage measured from current ESC - if supported
float32 esc_current # [A] Current measured from current ESC - if supported
float32 esc_temperature # [degC] Temperature measured from current ESC - if supported
int16 motor_temperature # [degC] Temperature measured from current motor - if supported

uint8 esc_state # [-] State of ESC - depend on Vendor

uint8 actuator_function # [-] Actuator output function (one of Motor1...MotorN)

uint8 ACTUATOR_FUNCTION_MOTOR1 = 101
uint8 ACTUATOR_FUNCTION_MOTOR_MAX = 112 # output_functions.yaml Motor.start + Motor.count - 1

uint16 failures # [@enum FAILURE] Bitmask to indicate the internal ESC faults
int8 esc_power # [%] [@range 0,100] Applied power (negative values reserved)

uint8 FAILURE_OVER_CURRENT = 0 # (1 << 0)
uint8 FAILURE_OVER_VOLTAGE = 1 # (1 << 1)
uint8 FAILURE_MOTOR_OVER_TEMPERATURE = 2 # (1 << 2)
uint8 FAILURE_OVER_RPM = 3 # (1 << 3)
uint8 FAILURE_INCONSISTENT_CMD = 4 # (1 << 4) Set if ESC received an inconsistent command (i.e out of boundaries)
uint8 FAILURE_MOTOR_STUCK = 5 # (1 << 5)
uint8 FAILURE_GENERIC = 6 # (1 << 6)
uint8 FAILURE_MOTOR_WARN_TEMPERATURE = 7 # (1 << 7)
uint8 FAILURE_WARN_ESC_TEMPERATURE = 8 # (1 << 8)
uint8 FAILURE_OVER_ESC_TEMPERATURE = 9 # (1 << 9)
uint8 ESC_FAILURE_COUNT = 10 # Counter - keep it as last element!
```

:::
