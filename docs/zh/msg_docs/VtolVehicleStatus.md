---
pageClass: is-wide-page
---

# VtolVehicleStatus (UORB message)

VEHICLE_VTOL_STATE, should match 1:1 MAVLinks's MAV_VTOL_STATE.

**TOPICS:** vtol_vehiclestatus

## Fields

| 参数名                                                                                      | 类型       | Unit [Frame] | Range/Enum | 描述                                                                                          |
| ---------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------- |
| timestamp                                                                                | `uint64` |                                                                  |            | time since system start (microseconds)                                   |
| vehicle_vtol_state                             | `uint8`  |                                                                  |            | current state of the vtol, see VEHICLE_VTOL_STATE |
| fixed_wing_system_failure | `bool`   |                                                                  |            | vehicle in fixed-wing system failure failsafe mode (after quad-chute)    |

## Constants

| 参数名                                                                                                                                                                                              | 类型       | 值 | 描述 |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION                                                                                                                             | `uint32` | 0 |    |
| <a href="#VEHICLE_VTOL_STATE_UNDEFINED"></a> VEHICLE_VTOL_STATE_UNDEFINED                                                         | `uint8`  | 0 |    |
| <a href="#VEHICLE_VTOL_STATE_TRANSITION_TO_FW"></a> VEHICLE_VTOL_STATE_TRANSITION_TO_FW | `uint8`  | 1 |    |
| <a href="#VEHICLE_VTOL_STATE_TRANSITION_TO_MC"></a> VEHICLE_VTOL_STATE_TRANSITION_TO_MC | `uint8`  | 2 |    |
| <a href="#VEHICLE_VTOL_STATE_MC"></a> VEHICLE_VTOL_STATE_MC                                                                       | `uint8`  | 3 |    |
| <a href="#VEHICLE_VTOL_STATE_FW"></a> VEHICLE_VTOL_STATE_FW                                                                       | `uint8`  | 4 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VtolVehicleStatus.msg)

:::details
Click here to see original file

```c
# VEHICLE_VTOL_STATE, should match 1:1 MAVLinks's MAV_VTOL_STATE

uint32 MESSAGE_VERSION = 0

uint8 VEHICLE_VTOL_STATE_UNDEFINED = 0
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_FW = 1
uint8 VEHICLE_VTOL_STATE_TRANSITION_TO_MC = 2
uint8 VEHICLE_VTOL_STATE_MC = 3
uint8 VEHICLE_VTOL_STATE_FW = 4

uint64 timestamp			# time since system start (microseconds)

uint8 vehicle_vtol_state		# current state of the vtol, see VEHICLE_VTOL_STATE

bool fixed_wing_system_failure		# vehicle in fixed-wing system failure failsafe mode (after quad-chute)
```

:::
