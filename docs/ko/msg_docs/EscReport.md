---
pageClass: is-wide-page
---

# EscReport (UORB message)

**TOPICS:** esc_report

## Fields

| 명칭                                     | 형식        | Unit [Frame] | Range/Enum | 설명                                                                                                                   |
| -------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------- |
| timestamp                              | `uint64`  |                                                                  |            | time since system start (microseconds)                                                            |
| esc_errorcount    | `uint32`  |                                                                  |            | Number of reported errors by ESC - if supported                                                                      |
| esc_rpm           | `int32`   |                                                                  |            | Motor RPM, negative for reverse rotation [RPM] - if supported    |
| esc_voltage       | `float32` |                                                                  |            | Voltage measured from current ESC [V] - if supported             |
| esc_current       | `float32` |                                                                  |            | Current measured from current ESC [A] - if supported             |
| esc_temperature   | `float32` |                                                                  |            | Temperature measured from current ESC [degC] - if supported      |
| esc_address       | `uint8`   |                                                                  |            | Address of current ESC (in most cases 1-8 / must be set by driver)                                |
| esc_cmdcount      | `uint8`   |                                                                  |            | Counter of number of commands                                                                                        |
| esc_state         | `uint8`   |                                                                  |            | State of ESC - depend on Vendor                                                                                      |
| actuator_function | `uint8`   |                                                                  |            | actuator output function (one of Motor1...MotorN) |
| failures                               | `uint16`  |                                                                  |            | Bitmask to indicate the internal ESC faults                                                                          |
| esc_power         | `int8`    |                                                                  |            | Applied power 0-100 in % (negative values reserved)                                               |

## Constants

| 명칭                                                                                                                                           | 형식      | Value | 설명                                                                                                                                                                                   |
| -------------------------------------------------------------------------------------------------------------------------------------------- | ------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a href="#ACTUATOR_FUNCTION_MOTOR1"></a> ACTUATOR_FUNCTION_MOTOR1                                  | `uint8` | 101   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR2"></a> ACTUATOR_FUNCTION_MOTOR2                                  | `uint8` | 102   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR3"></a> ACTUATOR_FUNCTION_MOTOR3                                  | `uint8` | 103   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR4"></a> ACTUATOR_FUNCTION_MOTOR4                                  | `uint8` | 104   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR5"></a> ACTUATOR_FUNCTION_MOTOR5                                  | `uint8` | 105   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR6"></a> ACTUATOR_FUNCTION_MOTOR6                                  | `uint8` | 106   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR7"></a> ACTUATOR_FUNCTION_MOTOR7                                  | `uint8` | 107   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR8"></a> ACTUATOR_FUNCTION_MOTOR8                                  | `uint8` | 108   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR9"></a> ACTUATOR_FUNCTION_MOTOR9                                  | `uint8` | 109   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR10"></a> ACTUATOR_FUNCTION_MOTOR10                                | `uint8` | 110   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR11"></a> ACTUATOR_FUNCTION_MOTOR11                                | `uint8` | 111   |                                                                                                                                                                                      |
| <a href="#ACTUATOR_FUNCTION_MOTOR12"></a> ACTUATOR_FUNCTION_MOTOR12                                | `uint8` | 112   |                                                                                                                                                                                      |
| <a href="#FAILURE_OVER_CURRENT"></a> FAILURE_OVER_CURRENT                                          | `uint8` | 0     | (1 << 0)                                                                                                        |
| <a href="#FAILURE_OVER_VOLTAGE"></a> FAILURE_OVER_VOLTAGE                                          | `uint8` | 1     | (1 << 1)                                                                                                        |
| <a href="#FAILURE_MOTOR_OVER_TEMPERATURE"></a> FAILURE_MOTOR_OVER_TEMPERATURE | `uint8` | 2     | (1 << 2)                                                                                                        |
| <a href="#FAILURE_OVER_RPM"></a> FAILURE_OVER_RPM                                                  | `uint8` | 3     | (1 << 3)                                                                                                        |
| <a href="#FAILURE_INCONSISTENT_CMD"></a> FAILURE_INCONSISTENT_CMD                                  | `uint8` | 4     | (1 << 4) Set if ESC received an inconsistent command (i.e out of boundaries) |
| <a href="#FAILURE_MOTOR_STUCK"></a> FAILURE_MOTOR_STUCK                                            | `uint8` | 5     | (1 << 5)                                                                                                        |
| <a href="#FAILURE_GENERIC"></a> FAILURE_GENERIC                                                                         | `uint8` | 6     | (1 << 6)                                                                                                        |
| <a href="#FAILURE_MOTOR_WARN_TEMPERATURE"></a> FAILURE_MOTOR_WARN_TEMPERATURE | `uint8` | 7     | (1 << 7)                                                                                                        |
| <a href="#FAILURE_WARN_ESC_TEMPERATURE"></a> FAILURE_WARN_ESC_TEMPERATURE     | `uint8` | 8     | (1 << 8)                                                                                                        |
| <a href="#FAILURE_OVER_ESC_TEMPERATURE"></a> FAILURE_OVER_ESC_TEMPERATURE     | `uint8` | 9     | (1 << 9)                                                                                                        |
| <a href="#ESC_FAILURE_COUNT"></a> ESC_FAILURE_COUNT                                                | `uint8` | 10    | Counter - keep it as last element!                                                                                                                                                   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EscReport.msg)

:::details
Click here to see original file

```c
uint64 timestamp					# time since system start (microseconds)
uint32 esc_errorcount					# Number of reported errors by ESC - if supported
int32 esc_rpm						# Motor RPM, negative for reverse rotation [RPM] - if supported
float32 esc_voltage					# Voltage measured from current ESC [V] - if supported
float32 esc_current					# Current measured from current ESC [A] - if supported
float32 esc_temperature					# Temperature measured from current ESC [degC] - if supported
uint8 esc_address					# Address of current ESC (in most cases 1-8 / must be set by driver)
uint8 esc_cmdcount					# Counter of number of commands

uint8 esc_state					# State of ESC - depend on Vendor

uint8 actuator_function				# actuator output function (one of Motor1...MotorN)

uint8 ACTUATOR_FUNCTION_MOTOR1 = 101
uint8 ACTUATOR_FUNCTION_MOTOR2 = 102
uint8 ACTUATOR_FUNCTION_MOTOR3 = 103
uint8 ACTUATOR_FUNCTION_MOTOR4 = 104
uint8 ACTUATOR_FUNCTION_MOTOR5 = 105
uint8 ACTUATOR_FUNCTION_MOTOR6 = 106
uint8 ACTUATOR_FUNCTION_MOTOR7 = 107
uint8 ACTUATOR_FUNCTION_MOTOR8 = 108
uint8 ACTUATOR_FUNCTION_MOTOR9 = 109
uint8 ACTUATOR_FUNCTION_MOTOR10 = 110
uint8 ACTUATOR_FUNCTION_MOTOR11 = 111
uint8 ACTUATOR_FUNCTION_MOTOR12 = 112

uint16 failures					# Bitmask to indicate the internal ESC faults
int8 esc_power					# Applied power 0-100 in % (negative values reserved)

uint8 FAILURE_OVER_CURRENT = 0 			# (1 << 0)
uint8 FAILURE_OVER_VOLTAGE = 1 			# (1 << 1)
uint8 FAILURE_MOTOR_OVER_TEMPERATURE = 2 	# (1 << 2)
uint8 FAILURE_OVER_RPM = 3			# (1 << 3)
uint8 FAILURE_INCONSISTENT_CMD = 4 		# (1 << 4)  Set if ESC received an inconsistent command (i.e out of boundaries)
uint8 FAILURE_MOTOR_STUCK = 5			# (1 << 5)
uint8 FAILURE_GENERIC = 6			# (1 << 6)
uint8 FAILURE_MOTOR_WARN_TEMPERATURE = 7	# (1 << 7)
uint8 FAILURE_WARN_ESC_TEMPERATURE = 8		# (1 << 8)
uint8 FAILURE_OVER_ESC_TEMPERATURE = 9		# (1 << 9)
uint8 ESC_FAILURE_COUNT = 10 			# Counter - keep it as last element!
```

:::
