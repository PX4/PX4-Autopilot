# Action request from a manual controller.
#
# Messages represents actions requested by an RC manual controller switch or gesture, such as a request to ARM or switch mode.
# Request are published by `manual_control` and subscribed by the `commander` and `vtol_att_control` modules.

uint64 timestamp # [us] Time since system start

uint8 action # [@enum ACTION] Requested action
uint8 ACTION_DISARM = 0 # Disarm vehicle
uint8 ACTION_ARM = 1 # Arm vehicle
uint8 ACTION_TOGGLE_ARMING = 2 # Toggle arming
uint8 ACTION_UNKILL = 3 # Revert a kill action
uint8 ACTION_KILL = 4 # Kill vehicle (activate flight termination)
uint8 ACTION_SWITCH_MODE = 5 # Switch mode. The target mode is set in the `mode` field.
uint8 ACTION_VTOL_TRANSITION_TO_MULTICOPTER = 6 # Transition to multicopter
uint8 ACTION_VTOL_TRANSITION_TO_FIXEDWING = 7 # Transition to fixed-wing

uint8 source # [@enum SOURCE] Request trigger type, such as a switch, button or gesture.
uint8 SOURCE_STICK_GESTURE = 0 # Gesture trigger
uint8 SOURCE_RC_SWITCH = 1 # RC switch trigger
uint8 SOURCE_RC_BUTTON = 2 # RC button trigger
uint8 SOURCE_RC_MODE_SLOT = 3 # RC mode slot trigger

uint8 mode # Requested mode. Only applies when `action` is `ACTION_SWITCH_MODE`. Modes values are set in `vehicle_status_s::NAVIGATION_STATE_*`.
