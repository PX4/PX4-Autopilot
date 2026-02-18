---
pageClass: is-wide-page
---

# PositionControllerLandingStatus (UORB message)

**TOPICS:** position_controllerlanding_status

## Fields

| Назва                                                              | Тип       | Unit [Frame] | Range/Enum | Опис                                                                |
| ------------------------------------------------------------------ | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------- |
| timestamp                                                          | `uint64`  | us                                                               |            | time since system start                                             |
| lateral_touchdown_offset | `float32` | m                                                                |            | lateral touchdown position offset manually commanded during landing |
| flaring                                                            | `bool`    |                                                                  |            | true if the aircraft is flaring                                     |
| abort_status                                  | `uint8`   |                                                                  |            |                                                                     |

## Constants

| Назва                                                                                                     | Тип     | Значення | Опис                                                                                                                                 |
| --------------------------------------------------------------------------------------------------------- | ------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| <a href="#NOT_ABORTED"></a> NOT_ABORTED                                              | `uint8` | 0        |                                                                                                                                      |
| <a href="#ABORTED_BY_OPERATOR"></a> ABORTED_BY_OPERATOR         | `uint8` | 1        |                                                                                                                                      |
| <a href="#TERRAIN_NOT_FOUND"></a> TERRAIN_NOT_FOUND             | `uint8` | 2        | FW_LND_ABORT (1 << 0) |
| <a href="#TERRAIN_TIMEOUT"></a> TERRAIN_TIMEOUT                                      | `uint8` | 3        | FW_LND_ABORT (1 << 1) |
| <a href="#UNKNOWN_ABORT_CRITERION"></a> UNKNOWN_ABORT_CRITERION | `uint8` | 4        |                                                                                                                                      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/PositionControllerLandingStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp # [us] time since system start
float32 lateral_touchdown_offset # [m] lateral touchdown position offset manually commanded during landing
bool flaring # true if the aircraft is flaring

# abort status is:
# 0 if not aborted
# >0 if aborted, with the singular abort criterion which triggered the landing abort enumerated by the following abort reasons
uint8 abort_status

# abort reasons
# after the manual operator abort, corresponds to individual bits of param FW_LND_ABORT
uint8 NOT_ABORTED = 0
uint8 ABORTED_BY_OPERATOR = 1
uint8 TERRAIN_NOT_FOUND = 2 # FW_LND_ABORT (1 << 0)
uint8 TERRAIN_TIMEOUT = 3 # FW_LND_ABORT (1 << 1)
uint8 UNKNOWN_ABORT_CRITERION = 4
```

:::
