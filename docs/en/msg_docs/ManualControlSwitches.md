# ManualControlSwitches (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ManualControlSwitches.msg)

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
uint8 gear_switch                # landing gear switch: _DOWN_, UP
uint8 transition_switch          # VTOL transition switch: _HOVER, FORWARD_FLIGHT

uint8 photo_switch               # Photo trigger switch
uint8 video_switch               # Photo trigger switch

uint8 payload_power_switch       # Payload power switch

uint8 engage_main_motor_switch   # Engage the main motor (for helicopters)

uint32 switch_changes            # number of switch changes

```
