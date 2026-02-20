---
pageClass: is-wide-page
---

# TakeoffStatus (UORB повідомлення)

Status of the takeoff state machine currently just available for multicopters.

**TOPICS:** takeoff_status

## Fields

| Назва                              | Тип       | Unit [Frame] | Range/Enum | Опис                                                                     |
| ---------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------ |
| timestamp                          | `uint64`  |                                                                  |            | time since system start (microseconds)                |
| takeoff_state | `uint8`   |                                                                  |            |                                                                          |
| tilt_limit    | `float32` |                                                                  |            | limited tilt feasibility during takeoff, contains maximum tilt otherwise |

## Constants

| Назва                                                                                                                                                               | Тип     | Значення | Опис |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#TAKEOFF_STATE_UNINITIALIZED"></a> TAKEOFF_STATE_UNINITIALIZED                                                   | `uint8` | 0        |      |
| <a href="#TAKEOFF_STATE_DISARMED"></a> TAKEOFF_STATE_DISARMED                                                             | `uint8` | 1        |      |
| <a href="#TAKEOFF_STATE_SPOOLUP"></a> TAKEOFF_STATE_SPOOLUP                                                               | `uint8` | 2        |      |
| <a href="#TAKEOFF_STATE_READY_FOR_TAKEOFF"></a> TAKEOFF_STATE_READY_FOR_TAKEOFF | `uint8` | 3        |      |
| <a href="#TAKEOFF_STATE_RAMPUP"></a> TAKEOFF_STATE_RAMPUP                                                                 | `uint8` | 4        |      |
| <a href="#TAKEOFF_STATE_FLIGHT"></a> TAKEOFF_STATE_FLIGHT                                                                 | `uint8` | 5        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TakeoffStatus.msg)

:::details
Click here to see original file

```c
# Status of the takeoff state machine currently just available for multicopters

uint64 timestamp # time since system start (microseconds)

uint8 TAKEOFF_STATE_UNINITIALIZED     = 0
uint8 TAKEOFF_STATE_DISARMED          = 1
uint8 TAKEOFF_STATE_SPOOLUP           = 2
uint8 TAKEOFF_STATE_READY_FOR_TAKEOFF = 3
uint8 TAKEOFF_STATE_RAMPUP            = 4
uint8 TAKEOFF_STATE_FLIGHT            = 5

uint8 takeoff_state

float32 tilt_limit # limited tilt feasibility during takeoff, contains maximum tilt otherwise
```

:::
