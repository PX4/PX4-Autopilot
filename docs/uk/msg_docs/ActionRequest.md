---
pageClass: is-wide-page
---

# ActionRequest (повідомлення UORB)

Action request for the vehicle's main state.

Message represents actions requested by a PX4 internal component towards the main state machine such as a request to arm or switch mode.
It allows mapping triggers from various external interfaces like RC channels or MAVLink to cause an action.
Request are published by `manual_control` and subscribed by the `commander` and `vtol_att_control` modules.

**TOPICS:** action_request

## Fields

| Назва     | Тип      | Unit [Frame] | Range/Enum        | Опис                                                                                                                                                                                                             |
| --------- | -------- | ---------------------------------------------------------------- | ----------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp | `uint64` | us                                                               |                   | Time since system start                                                                                                                                                                                          |
| action    | `uint8`  |                                                                  | [ACTION](#ACTION) | Requested action                                                                                                                                                                                                 |
| source    | `uint8`  |                                                                  | [SOURCE](#SOURCE) | Request trigger type, such as a switch, button or gesture                                                                                                                                                        |
| mode      | `uint8`  |                                                                  |                   | Requested mode. Only applies when `action` is `ACTION_SWITCH_MODE`. Values for this field are defined by the `vehicle_status_s::NAVIGATION_STATE_*` enumeration. |

## Enums

### ACTION {#ACTION}

| Назва                                                                                                                                                                           | Тип     | Значення | Опис                                                                                     |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | -------- | ---------------------------------------------------------------------------------------- |
| <a href="#ACTION_DISARM"></a> ACTION_DISARM                                                                                                                | `uint8` | 0        | Disarm vehicle                                                                           |
| <a href="#ACTION_ARM"></a> ACTION_ARM                                                                                                                      | `uint8` | 1        | Arm vehicle                                                                              |
| <a href="#ACTION_TOGGLE_ARMING"></a> ACTION_TOGGLE_ARMING                                                                             | `uint8` | 2        | Toggle arming                                                                            |
| <a href="#ACTION_UNKILL"></a> ACTION_UNKILL                                                                                                                | `uint8` | 3        | Revert a kill action                                                                     |
| <a href="#ACTION_KILL"></a> ACTION_KILL                                                                                                                    | `uint8` | 4        | Kill vehicle (instantly stop the motors)                              |
| <a href="#ACTION_SWITCH_MODE"></a> ACTION_SWITCH_MODE                                                                                 | `uint8` | 5        | Switch mode. The target mode is set in the `mode` field. |
| <a href="#ACTION_VTOL_TRANSITION_TO_MULTICOPTER"></a> ACTION_VTOL_TRANSITION_TO_MULTICOPTER | `uint8` | 6        | Transition to hover flight                                                               |
| <a href="#ACTION_VTOL_TRANSITION_TO_FIXEDWING"></a> ACTION_VTOL_TRANSITION_TO_FIXEDWING     | `uint8` | 7        | Transition to fast forward flight                                                        |
| <a href="#ACTION_TERMINATION"></a> ACTION_TERMINATION                                                                                                      | `uint8` | 8        | Irreversibly output failsafe values on all outputs, trigger parachute                    |

### SOURCE {#SOURCE}

| Назва                                                                                                                  | Тип     | Значення | Опис                                                            |
| ---------------------------------------------------------------------------------------------------------------------- | ------- | -------- | --------------------------------------------------------------- |
| <a href="#SOURCE_STICK_GESTURE"></a> SOURCE_STICK_GESTURE                    | `uint8` | 0        | Triggered by holding the sticks in a certain position           |
| <a href="#SOURCE_RC_SWITCH"></a> SOURCE_RC_SWITCH                            | `uint8` | 1        | Triggered by an RC switch moving into a certain position        |
| <a href="#SOURCE_RC_BUTTON"></a> SOURCE_RC_BUTTON                            | `uint8` | 2        | Triggered by a momentary button on the RC being pressed or held |
| <a href="#SOURCE_RC_MODE_SLOT"></a> SOURCE_RC_MODE_SLOT | `uint8` | 3        | Mode change through the RC mode selection mechanism             |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActionRequest.msg)

:::details
Click here to see original file

```c
# Action request for the vehicle's main state
#
# Message represents actions requested by a PX4 internal component towards the main state machine such as a request to arm or switch mode.
# It allows mapping triggers from various external interfaces like RC channels or MAVLink to cause an action.
# Request are published by `manual_control` and subscribed by the `commander` and `vtol_att_control` modules.

uint64 timestamp  # [us] Time since system start

uint8 action                                     # [@enum ACTION] Requested action
uint8 ACTION_DISARM = 0                          # Disarm vehicle
uint8 ACTION_ARM = 1                             # Arm vehicle
uint8 ACTION_TOGGLE_ARMING = 2                   # Toggle arming
uint8 ACTION_UNKILL = 3                          # Revert a kill action
uint8 ACTION_KILL = 4                            # Kill vehicle (instantly stop the motors)
uint8 ACTION_SWITCH_MODE = 5                     # Switch mode. The target mode is set in the `mode` field.
uint8 ACTION_VTOL_TRANSITION_TO_MULTICOPTER = 6  # Transition to hover flight
uint8 ACTION_VTOL_TRANSITION_TO_FIXEDWING = 7    # Transition to fast forward flight
uint8 ACTION_TERMINATION = 8                     # Irreversibly output failsafe values on all outputs, trigger parachute

uint8 source                    # [@enum SOURCE] Request trigger type, such as a switch, button or gesture
uint8 SOURCE_STICK_GESTURE = 0  # Triggered by holding the sticks in a certain position
uint8 SOURCE_RC_SWITCH = 1      # Triggered by an RC switch moving into a certain position
uint8 SOURCE_RC_BUTTON = 2      # Triggered by a momentary button on the RC being pressed or held
uint8 SOURCE_RC_MODE_SLOT = 3   # Mode change through the RC mode selection mechanism

uint8 mode  # Requested mode. Only applies when `action` is `ACTION_SWITCH_MODE`. Values for this field are defined by the `vehicle_status_s::NAVIGATION_STATE_*` enumeration.
```

:::
