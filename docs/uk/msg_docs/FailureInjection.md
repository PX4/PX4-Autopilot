---
pageClass: is-wide-page
---

# FailureInjection (UORB message)

Failure injection configuration.

Currently active failure-injection configuration, published by the failure
injection manager (the sole subscriber to vehicle_command INJECT_FAILURE).
Republished only when the configuration changes, so command spam on
vehicle_command cannot propagate to the consumers applying the failures.

**TOPICS:** failure_injection

## Fields

| Назва                                                            | Тип         | Unit [Frame] | Range/Enum                                         | Опис                                                                    |
| ---------------------------------------------------------------- | ----------- | ---------------------------------------------------------------- | -------------------------------------------------- | ----------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                              | `uint64`    | us                                                               |                                                    | Time since system start                                                 |
| <a id="fld_count"></a>count                                      | `uint8`     |                                                                  |                                                    | number of valid entries in the arrays below                             |
| <a id="fld_unit"></a>unit                                        | `uint8[4]`  |                                                                  | [FAILURE_UNIT](#FAILURE_UNIT) | Affected component per entry                                            |
| <a id="fld_instance_mask"></a>instance_mask | `uint16[4]` |                                                                  |                                                    | Bit i targets instance (i+1); 0xFFFF = all instances |
| <a id="fld_failure_type"></a>failure_type   | `uint8[4]`  |                                                                  | [FAILURE_TYPE](#FAILURE_TYPE) | failure mode per entry                                                  |

## Enums

### FAILURE_UNIT {#FAILURE_UNIT}

Used in field(s): [unit](#fld_unit)

| Назва                                                                                                                                                                     | Тип     | Значення | Опис |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a id="#FAILURE_UNIT_SENSOR_GYRO"></a> FAILURE_UNIT_SENSOR_GYRO                                            | `uint8` | 0        |      |
| <a id="#FAILURE_UNIT_SENSOR_ACCEL"></a> FAILURE_UNIT_SENSOR_ACCEL                                          | `uint8` | 1        |      |
| <a id="#FAILURE_UNIT_SENSOR_MAG"></a> FAILURE_UNIT_SENSOR_MAG                                              | `uint8` | 2        |      |
| <a id="#FAILURE_UNIT_SENSOR_BARO"></a> FAILURE_UNIT_SENSOR_BARO                                            | `uint8` | 3        |      |
| <a id="#FAILURE_UNIT_SENSOR_GPS"></a> FAILURE_UNIT_SENSOR_GPS                                              | `uint8` | 4        |      |
| <a id="#FAILURE_UNIT_SENSOR_OPTICAL_FLOW"></a> FAILURE_UNIT_SENSOR_OPTICAL_FLOW       | `uint8` | 5        |      |
| <a id="#FAILURE_UNIT_SENSOR_VIO"></a> FAILURE_UNIT_SENSOR_VIO                                              | `uint8` | 6        |      |
| <a id="#FAILURE_UNIT_SENSOR_DISTANCE_SENSOR"></a> FAILURE_UNIT_SENSOR_DISTANCE_SENSOR | `uint8` | 7        |      |
| <a id="#FAILURE_UNIT_SENSOR_AIRSPEED"></a> FAILURE_UNIT_SENSOR_AIRSPEED                                    | `uint8` | 8        |      |
| <a id="#FAILURE_UNIT_SYSTEM_BATTERY"></a> FAILURE_UNIT_SYSTEM_BATTERY                                      | `uint8` | 100      |      |
| <a id="#FAILURE_UNIT_SYSTEM_MOTOR"></a> FAILURE_UNIT_SYSTEM_MOTOR                                          | `uint8` | 101      |      |
| <a id="#FAILURE_UNIT_SYSTEM_SERVO"></a> FAILURE_UNIT_SYSTEM_SERVO                                          | `uint8` | 102      |      |
| <a id="#FAILURE_UNIT_SYSTEM_AVOIDANCE"></a> FAILURE_UNIT_SYSTEM_AVOIDANCE                                  | `uint8` | 103      |      |
| <a id="#FAILURE_UNIT_SYSTEM_RC_SIGNAL"></a> FAILURE_UNIT_SYSTEM_RC_SIGNAL             | `uint8` | 104      |      |
| <a id="#FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL"></a> FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL   | `uint8` | 105      |      |
| <a id="#FAILURE_UNIT_SYSTEM_ESC"></a> FAILURE_UNIT_SYSTEM_ESC                                              | `uint8` | 106      |      |

### FAILURE_TYPE {#FAILURE_TYPE}

Used in field(s): [failure_type](#fld_failure_type)

| Назва                                                                                                       | Тип     | Значення | Опис |
| ----------------------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a id="#FAILURE_TYPE_OK"></a> FAILURE_TYPE_OK                     | `uint8` | 0        |      |
| <a id="#FAILURE_TYPE_OFF"></a> FAILURE_TYPE_OFF                   | `uint8` | 1        |      |
| <a id="#FAILURE_TYPE_STUCK"></a> FAILURE_TYPE_STUCK               | `uint8` | 2        |      |
| <a id="#FAILURE_TYPE_GARBAGE"></a> FAILURE_TYPE_GARBAGE           | `uint8` | 3        |      |
| <a id="#FAILURE_TYPE_WRONG"></a> FAILURE_TYPE_WRONG               | `uint8` | 4        |      |
| <a id="#FAILURE_TYPE_SLOW"></a> FAILURE_TYPE_SLOW                 | `uint8` | 5        |      |
| <a id="#FAILURE_TYPE_DELAYED"></a> FAILURE_TYPE_DELAYED           | `uint8` | 6        |      |
| <a id="#FAILURE_TYPE_INTERMITTENT"></a> FAILURE_TYPE_INTERMITTENT | `uint8` | 7        |      |

## Constants

| Назва                                                        | Тип     | Значення | Опис                                    |
| ------------------------------------------------------------ | ------- | -------- | --------------------------------------- |
| <a id="#MAX_FAILURES"></a> MAX_FAILURES | `uint8` | 4        | maximum number of simultaneous failures |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FailureInjection.msg)

:::details
Click here to see original file

```c
#Failure injection configuration.
#
# Currently active failure-injection configuration, published by the failure
# injection manager (the sole subscriber to vehicle_command INJECT_FAILURE).
# Republished only when the configuration changes, so command spam on
# vehicle_command cannot propagate to the consumers applying the failures.

uint64 timestamp # [us] Time since system start

uint8 MAX_FAILURES = 4 # maximum number of simultaneous failures

uint8 count # number of valid entries in the arrays below

uint8[4] unit # [@enum FAILURE_UNIT] Affected component per entry
# Failure unit (affected component). Mirrors MAVLink FAILURE_UNIT and the
# FAILURE_UNIT_* values in vehicle_command.
uint8 FAILURE_UNIT_SENSOR_GYRO = 0
uint8 FAILURE_UNIT_SENSOR_ACCEL = 1
uint8 FAILURE_UNIT_SENSOR_MAG = 2
uint8 FAILURE_UNIT_SENSOR_BARO = 3
uint8 FAILURE_UNIT_SENSOR_GPS = 4
uint8 FAILURE_UNIT_SENSOR_OPTICAL_FLOW = 5
uint8 FAILURE_UNIT_SENSOR_VIO = 6
uint8 FAILURE_UNIT_SENSOR_DISTANCE_SENSOR = 7
uint8 FAILURE_UNIT_SENSOR_AIRSPEED = 8
uint8 FAILURE_UNIT_SYSTEM_BATTERY = 100
uint8 FAILURE_UNIT_SYSTEM_MOTOR = 101
uint8 FAILURE_UNIT_SYSTEM_SERVO = 102
uint8 FAILURE_UNIT_SYSTEM_AVOIDANCE = 103
uint8 FAILURE_UNIT_SYSTEM_RC_SIGNAL = 104
uint8 FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL = 105
uint8 FAILURE_UNIT_SYSTEM_ESC = 106

uint16[4] instance_mask # Bit i targets instance (i+1); 0xFFFF = all instances

uint8[4] failure_type # [@enum FAILURE_TYPE] failure mode per entry
# Failure mode.
# Mirrors MAVLink FAILURE_TYPE and the FAILURE_TYPE_* values in vehicle_command.
uint8 FAILURE_TYPE_OK = 0
uint8 FAILURE_TYPE_OFF = 1
uint8 FAILURE_TYPE_STUCK = 2
uint8 FAILURE_TYPE_GARBAGE = 3
uint8 FAILURE_TYPE_WRONG = 4
uint8 FAILURE_TYPE_SLOW = 5
uint8 FAILURE_TYPE_DELAYED = 6
uint8 FAILURE_TYPE_INTERMITTENT = 7
```

:::
