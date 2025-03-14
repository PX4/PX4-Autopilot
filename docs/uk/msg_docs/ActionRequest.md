# ActionRequest (повідомлення UORB)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ActionRequest.msg)

```c
uint64 timestamp # time since system start (microseconds)

uint8 action # what action is requested
uint8 ACTION_DISARM = 0
uint8 ACTION_ARM = 1
uint8 ACTION_TOGGLE_ARMING = 2
uint8 ACTION_UNKILL = 3
uint8 ACTION_KILL = 4
uint8 ACTION_SWITCH_MODE = 5
uint8 ACTION_VTOL_TRANSITION_TO_MULTICOPTER = 6
uint8 ACTION_VTOL_TRANSITION_TO_FIXEDWING = 7

uint8 source # how the request was triggered
uint8 SOURCE_STICK_GESTURE = 0
uint8 SOURCE_RC_SWITCH = 1
uint8 SOURCE_RC_BUTTON = 2
uint8 SOURCE_RC_MODE_SLOT = 3

uint8 mode # for ACTION_SWITCH_MODE what mode is requested according to vehicle_status_s::NAVIGATION_STATE_*

```
