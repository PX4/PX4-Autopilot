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
