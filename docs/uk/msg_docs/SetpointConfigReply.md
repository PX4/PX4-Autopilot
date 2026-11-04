---
pageClass: is-wide-page
---

# SetpointConfigReply (UORB message)

Reply to SetpointConfig.

**TOPICS:** setpoint_config_reply

## Fields

| Назва                                                                                                                              | Тип      | Unit [Frame] | Range/Enum        | Опис                                                                               |
| ---------------------------------------------------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ----------------- | ---------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                                                                | `uint64` | us                                                               |                   | Time since system start                                                            |
| <a id="fld_type"></a>type                                                                                                          | `uint16` |                                                                  |                   | See SetpointConfig::TYPE\_\* |
| <a id="fld_source_id"></a>source_id                                                                           | `uint8`  |                                                                  |                   | nav_state of the mode that sent the SetpointConfig            |
| <a id="fld_result"></a>result                                                                                                      | `uint8`  |                                                                  | [RESULT](#RESULT) |                                                                                    |
| <a id="fld_mode_req_angular_velocity"></a>mode_req_angular_velocity | `bool`   |                                                                  |                   |                                                                                    |
| <a id="fld_mode_req_attitude"></a>mode_req_attitude                                      | `bool`   |                                                                  |                   |                                                                                    |
| <a id="fld_mode_req_local_alt"></a>mode_req_local_alt               | `bool`   |                                                                  |                   |                                                                                    |
| <a id="fld_mode_req_local_position"></a>mode_req_local_position     | `bool`   |                                                                  |                   |                                                                                    |

## Enums

### RESULT {#RESULT}

Used in field(s): [result](#fld_result)

| Назва                                                                                                                                  | Тип     | Значення | Опис                                                 |
| -------------------------------------------------------------------------------------------------------------------------------------- | ------- | -------- | ---------------------------------------------------- |
| <a id="#RESULT_SUCCESS"></a> RESULT_SUCCESS                                                                       | `uint8` | 0        |                                                      |
| <a id="#RESULT_FAILURE_OTHER"></a> RESULT_FAILURE_OTHER                                      | `uint8` | 1        |                                                      |
| <a id="#RESULT_UNSUPPORTED"></a> RESULT_UNSUPPORTED                                                               | `uint8` | 2        | Setpoint type is unsupported for the current vehicle |
| <a id="#RESULT_UNKNOWN_SETPOINT_TYPE"></a> RESULT_UNKNOWN_SETPOINT_TYPE | `uint8` | 3        | The setpoint type is not known                       |

## Constants

| Назва                                                              | Тип      | Значення | Опис |
| ------------------------------------------------------------------ | -------- | -------- | ---- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/SetpointConfigReply.msg)

:::details
Click here to see original file

```c
# Reply to SetpointConfig

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start

uint16 type # See SetpointConfig::TYPE_*

uint8 source_id # nav_state of the mode that sent the SetpointConfig

uint8 RESULT_SUCCESS = 0
uint8 RESULT_FAILURE_OTHER = 1
uint8 RESULT_UNSUPPORTED = 2 # Setpoint type is unsupported for the current vehicle
uint8 RESULT_UNKNOWN_SETPOINT_TYPE = 3 # The setpoint type is not known
uint8 result # [@enum RESULT]

# Mode requirements for using the given setpoint type. A mode will use these and apply them to the arming check reply (PX4 does not do that itself).
# Certain setpoint types can be used in a reduced way, for example a TrajectorySetpoint without position control. In that case PX4 still sets all requirement flags, and the mode will then ignore mode_req_local_position.
bool mode_req_angular_velocity
bool mode_req_attitude
bool mode_req_local_alt
bool mode_req_local_position
```

:::
