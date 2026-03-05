---
pageClass: is-wide-page
---

# ModeCompleted (повідомлення UORB)

Результат завершення режиму, опублікований активним режимом. Можливі значення nav_state визначені в повідомленні VehicleStatus. Note that this is not always published (e.g. when a user switches modes or on. failsafe activation).

**TOPICS:** mode_completed

## Fields

| Назва                          | Тип      | Unit [Frame] | Range/Enum | Опис                                                      |
| ------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                      | `uint64` |                                                                  |            | time since system start (microseconds) |
| result                         | `uint8`  |                                                                  |            | One of RESULT\_\*                   |
| nav_state | `uint8`  |                                                                  |            | Source mode (values in VehicleStatus)  |

## Constants

| Назва                                                                                               | Тип      | Значення | Опис                                           |
| --------------------------------------------------------------------------------------------------- | -------- | -------- | ---------------------------------------------- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                | `uint32` | 0        |                                                |
| <a href="#RESULT_SUCCESS"></a> RESULT_SUCCESS                                  | `uint8`  | 0        |                                                |
| <a href="#RESULT_FAILURE_OTHER"></a> RESULT_FAILURE_OTHER | `uint8`  | 100      | Mode failed (generic error) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/ModeCompleted.msg)

:::details
Click here to see original file

```c
# Mode completion result, published by an active mode.
# The possible values of nav_state are defined in the VehicleStatus msg.
# Note that this is not always published (e.g. when a user switches modes or on
# failsafe activation)

uint32 MESSAGE_VERSION = 0

uint64 timestamp				 # time since system start (microseconds)


uint8 RESULT_SUCCESS = 0
# [1-99]: reserved
uint8 RESULT_FAILURE_OTHER = 100 # Mode failed (generic error)

uint8 result                     # One of RESULT_*

uint8 nav_state                  # Source mode (values in VehicleStatus)
```

:::
