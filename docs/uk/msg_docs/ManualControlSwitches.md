---
pageClass: is-wide-page
---

# ManualControlSwitches (UORB message)

**TOPICS:** manual_controlswitches

## Fields

| Назва                                                                                   | Тип      | Unit [Frame] | Range/Enum | Опис                                                                                                       |
| --------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------- |
| timestamp                                                                               | `uint64` |                                                                  |            | time since system start (microseconds)                                                  |
| timestamp_sample                                                   | `uint64` |                                                                  |            | the timestamp of the raw data (microseconds)                                            |
| mode_slot                                                          | `uint8`  |                                                                  |            | the slot a specific model selector is in                                                                   |
| arm_switch                                                         | `uint8`  |                                                                  |            | arm/disarm switch: _DISARMED_, ARMED                                                       |
| return_switch                                                      | `uint8`  |                                                                  |            | return to launch 2 position switch (mandatory): _NORMAL_, RTL           |
| loiter_switch                                                      | `uint8`  |                                                                  |            | loiter 2 position switch (optional): _MISSION_, LOITER                  |
| offboard_switch                                                    | `uint8`  |                                                                  |            | offboard 2 position switch (optional): _NORMAL_, OFFBOARD               |
| kill_switch                                                        | `uint8`  |                                                                  |            | throttle kill: _NORMAL_, KILL                                                              |
| termination_switch                                                 | `uint8`  |                                                                  |            | trigger termination which cannot be undone                                                                 |
| gear_switch                                                        | `uint8`  |                                                                  |            | landing gear switch: _DOWN_, UP                                                            |
| transition_switch                                                  | `uint8`  |                                                                  |            | VTOL transition switch: \_HOVER, FORWARD_FLIGHT |
| photo_switch                                                       | `uint8`  |                                                                  |            | Photo trigger switch                                                                                       |
| video_switch                                                       | `uint8`  |                                                                  |            | Photo trigger switch                                                                                       |
| engage_main_motor_switch | `uint8`  |                                                                  |            | Engage the main motor (for helicopters)                                                 |
| payload_power_switch                          | `uint8`  |                                                                  |            | Payload power switch                                                                                       |
| switch_changes                                                     | `uint32` |                                                                  |            | number of switch changes                                                                                   |

## Constants

| Назва                                                                                         | Тип     | Значення | Опис                                                 |
| --------------------------------------------------------------------------------------------- | ------- | -------- | ---------------------------------------------------- |
| <a href="#SWITCH_POS_NONE"></a> SWITCH_POS_NONE     | `uint8` | 0        | switch is not mapped                                 |
| <a href="#SWITCH_POS_ON"></a> SWITCH_POS_ON         | `uint8` | 1        | switch activated (value = 1)      |
| <a href="#SWITCH_POS_MIDDLE"></a> SWITCH_POS_MIDDLE | `uint8` | 2        | middle position (value = 0)       |
| <a href="#SWITCH_POS_OFF"></a> SWITCH_POS_OFF       | `uint8` | 3        | switch not activated (value = -1) |
| <a href="#MODE_SLOT_NONE"></a> MODE_SLOT_NONE       | `uint8` | 0        | no mode slot assigned                                |
| <a href="#MODE_SLOT_1"></a> MODE_SLOT_1             | `uint8` | 1        | mode slot 1 selected                                 |
| <a href="#MODE_SLOT_2"></a> MODE_SLOT_2             | `uint8` | 2        | mode slot 2 selected                                 |
| <a href="#MODE_SLOT_3"></a> MODE_SLOT_3             | `uint8` | 3        | mode slot 3 selected                                 |
| <a href="#MODE_SLOT_4"></a> MODE_SLOT_4             | `uint8` | 4        | mode slot 4 selected                                 |
| <a href="#MODE_SLOT_5"></a> MODE_SLOT_5             | `uint8` | 5        | mode slot 5 selected                                 |
| <a href="#MODE_SLOT_6"></a> MODE_SLOT_6             | `uint8` | 6        | mode slot 6 selected                                 |
| <a href="#MODE_SLOT_NUM"></a> MODE_SLOT_NUM         | `uint8` | 6        | number of slots                                      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ManualControlSwitches.msg)

:::details
Click here to see original file

```c
uint64 timestamp                 # time since system start (microseconds)

uint64 timestamp_sample          # the timestamp of the raw data (microseconds)

uint8 SWITCH_POS_NONE   = 0      # switch is not mapped
uint8 SWITCH_POS_ON     = 1      # switch activated (value = 1)
uint8 SWITCH_POS_MIDDLE = 2      # middle position (value = 0)
uint8 SWITCH_POS_OFF    = 3      # switch not activated (value = -1)

uint8 MODE_SLOT_NONE    = 0      # no mode slot assigned
uint8 MODE_SLOT_1       = 1      # mode slot 1 selected
uint8 MODE_SLOT_2       = 2      # mode slot 2 selected
uint8 MODE_SLOT_3       = 3      # mode slot 3 selected
uint8 MODE_SLOT_4       = 4      # mode slot 4 selected
uint8 MODE_SLOT_5       = 5      # mode slot 5 selected
uint8 MODE_SLOT_6       = 6      # mode slot 6 selected
uint8 MODE_SLOT_NUM     = 6      # number of slots

uint8 mode_slot                  # the slot a specific model selector is in

uint8 arm_switch                 # arm/disarm switch: _DISARMED_, ARMED
uint8 return_switch              # return to launch 2 position switch (mandatory): _NORMAL_, RTL
uint8 loiter_switch              # loiter 2 position switch (optional): _MISSION_, LOITER
uint8 offboard_switch            # offboard 2 position switch (optional): _NORMAL_, OFFBOARD
uint8 kill_switch                # throttle kill: _NORMAL_, KILL
uint8 termination_switch         # trigger termination which cannot be undone
uint8 gear_switch                # landing gear switch: _DOWN_, UP
uint8 transition_switch          # VTOL transition switch: _HOVER, FORWARD_FLIGHT

uint8 photo_switch               # Photo trigger switch
uint8 video_switch               # Photo trigger switch

uint8 engage_main_motor_switch   # Engage the main motor (for helicopters)

uint8 payload_power_switch       # Payload power switch

uint32 switch_changes            # number of switch changes
```

:::
