---
pageClass: is-wide-page
---

# NavigatorStatus (UORB message)

Current status of a Navigator mode. The possible values of nav_state are defined in the VehicleStatus msg.

**TOPICS:** navigator_status

## Fields

| 명칭                             | 형식       | Unit [Frame] | Range/Enum | 설명                                                        |
| ------------------------------ | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                      | `uint64` |                                                                  |            | time since system start (microseconds) |
| nav_state | `uint8`  |                                                                  |            | Source mode (values in VehicleStatus)  |
| failure                        | `uint8`  |                                                                  |            | Navigator failure enum                                    |

## Constants

| 명칭                                                             | 형식      | Value | 설명                                                  |
| -------------------------------------------------------------- | ------- | ----- | --------------------------------------------------- |
| <a href="#FAILURE_NONE"></a> FAILURE_NONE | `uint8` | 0     |                                                     |
| <a href="#FAILURE_HAGL"></a> FAILURE_HAGL | `uint8` | 1     | Target altitude exceeds maximum height above ground |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/NavigatorStatus.msg)

:::details
Click here to see original file

```c
# Current status of a Navigator mode
# The possible values of nav_state are defined in the VehicleStatus msg.
uint64 timestamp  # time since system start (microseconds)

uint8 nav_state   # Source mode (values in VehicleStatus)
uint8 failure     # Navigator failure enum

uint8 FAILURE_NONE = 0
uint8 FAILURE_HAGL = 1 # Target altitude exceeds maximum height above ground
```

:::
