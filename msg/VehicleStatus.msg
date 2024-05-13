# Encodes the system state of the vehicle published by commander

uint64 timestamp # time since system start (microseconds)

uint64 armed_time # Arming timestamp (microseconds)
uint64 takeoff_time # Takeoff timestamp (microseconds)

uint8 arming_state
uint8 ARMING_STATE_DISARMED = 1
uint8 ARMING_STATE_ARMED    = 2

uint8 latest_arming_reason
uint8 latest_disarming_reason
uint8 ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0
uint8 ARM_DISARM_REASON_RC_STICK = 1
uint8 ARM_DISARM_REASON_RC_SWITCH = 2
uint8 ARM_DISARM_REASON_COMMAND_INTERNAL = 3
uint8 ARM_DISARM_REASON_COMMAND_EXTERNAL = 4
uint8 ARM_DISARM_REASON_MISSION_START = 5
uint8 ARM_DISARM_REASON_SAFETY_BUTTON = 6
uint8 ARM_DISARM_REASON_AUTO_DISARM_LAND = 7
uint8 ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8
uint8 ARM_DISARM_REASON_KILL_SWITCH = 9
uint8 ARM_DISARM_REASON_LOCKDOWN = 10
uint8 ARM_DISARM_REASON_FAILURE_DETECTOR = 11
uint8 ARM_DISARM_REASON_SHUTDOWN = 12
uint8 ARM_DISARM_REASON_UNIT_TEST = 13

uint64 nav_state_timestamp # time when current nav_state activated

uint8 nav_state_user_intention                  # Mode that the user selected (might be different from nav_state in a failsafe situation)

uint8 nav_state                                 # Currently active mode
uint8 NAVIGATION_STATE_MANUAL = 0               # Manual mode
uint8 NAVIGATION_STATE_ALTCTL = 1               # Altitude control mode
uint8 NAVIGATION_STATE_POSCTL = 2               # Position control mode
uint8 NAVIGATION_STATE_AUTO_MISSION = 3         # Auto mission mode
uint8 NAVIGATION_STATE_AUTO_LOITER = 4          # Auto loiter mode
uint8 NAVIGATION_STATE_AUTO_RTL = 5             # Auto return to launch mode
uint8 NAVIGATION_STATE_POSITION_SLOW = 6
uint8 NAVIGATION_STATE_FREE5 = 7
uint8 NAVIGATION_STATE_FREE4 = 8
uint8 NAVIGATION_STATE_FREE3 = 9
uint8 NAVIGATION_STATE_ACRO = 10                # Acro mode
uint8 NAVIGATION_STATE_FREE2 = 11
uint8 NAVIGATION_STATE_DESCEND = 12             # Descend mode (no position control)
uint8 NAVIGATION_STATE_TERMINATION = 13         # Termination mode
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15                # Stabilized mode
uint8 NAVIGATION_STATE_FREE1 = 16
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17        # Takeoff
uint8 NAVIGATION_STATE_AUTO_LAND = 18           # Land
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19  # Auto Follow
uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20       # Precision land with landing target
uint8 NAVIGATION_STATE_ORBIT = 21               # Orbit in a circle
uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22   # Takeoff, transition, establish loiter
uint8 NAVIGATION_STATE_EXTERNAL1 = 23
uint8 NAVIGATION_STATE_EXTERNAL2 = 24
uint8 NAVIGATION_STATE_EXTERNAL3 = 25
uint8 NAVIGATION_STATE_EXTERNAL4 = 26
uint8 NAVIGATION_STATE_EXTERNAL5 = 27
uint8 NAVIGATION_STATE_EXTERNAL6 = 28
uint8 NAVIGATION_STATE_EXTERNAL7 = 29
uint8 NAVIGATION_STATE_EXTERNAL8 = 30
uint8 NAVIGATION_STATE_MAX = 31

uint8 executor_in_charge                        # Current mode executor in charge (0=Autopilot)

uint32 valid_nav_states_mask                    # Bitmask for all valid nav_state values
uint32 can_set_nav_states_mask                  # Bitmask for all modes that a user can select

# Bitmask of detected failures
uint16 failure_detector_status
uint16 FAILURE_NONE = 0
uint16 FAILURE_ROLL = 1              # (1 << 0)
uint16 FAILURE_PITCH = 2             # (1 << 1)
uint16 FAILURE_ALT = 4               # (1 << 2)
uint16 FAILURE_EXT = 8               # (1 << 3)
uint16 FAILURE_ARM_ESC = 16          # (1 << 4)
uint16 FAILURE_BATTERY = 32          # (1 << 5)
uint16 FAILURE_IMBALANCED_PROP = 64  # (1 << 6)
uint16 FAILURE_MOTOR = 128           # (1 << 7)

uint8 hil_state
uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

# If it's a VTOL, then the value will be VEHICLE_TYPE_ROTARY_WING while flying as a multicopter, and VEHICLE_TYPE_FIXED_WING when flying as a fixed-wing
uint8 vehicle_type
uint8 VEHICLE_TYPE_UNKNOWN = 0
uint8 VEHICLE_TYPE_ROTARY_WING = 1
uint8 VEHICLE_TYPE_FIXED_WING = 2
uint8 VEHICLE_TYPE_ROVER = 3
uint8 VEHICLE_TYPE_AIRSHIP = 4
uint8 VEHICLE_TYPE_SPACECRAFT = 5

uint8 FAILSAFE_DEFER_STATE_DISABLED = 0
uint8 FAILSAFE_DEFER_STATE_ENABLED = 1
uint8 FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2 # Failsafes deferred, but would trigger a failsafe

bool failsafe # true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)
bool failsafe_and_user_took_over # true if system is in failsafe state but the user took over control
uint8 failsafe_defer_state # one of FAILSAFE_DEFER_STATE_*

# Link loss
bool gcs_connection_lost              # datalink to GCS lost
uint8 gcs_connection_lost_counter     # counts unique GCS connection lost events
bool high_latency_data_link_lost # Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost

# VTOL flags
bool is_vtol             # True if the system is VTOL capable
bool is_vtol_tailsitter  # True if the system performs a 90° pitch down rotation during transition from MC to FW
bool in_transition_mode  # True if VTOL is doing a transition
bool in_transition_to_fw # True if VTOL is doing a transition from MC to FW

# MAVLink identification
uint8 system_type  # system type, contains mavlink MAV_TYPE
uint8 system_id	   # system id, contains MAVLink's system ID field
uint8 component_id # subsystem / component id, contains MAVLink's component ID field

bool safety_button_available # Set to true if a safety button is connected
bool safety_off # Set to true if safety is off

bool power_input_valid                            # set if input power is valid
bool usb_connected                                # set to true (never cleared) once telemetry received from usb link

bool open_drone_id_system_present
bool open_drone_id_system_healthy

bool parachute_system_present
bool parachute_system_healthy

bool avoidance_system_required                    # Set to true if avoidance system is enabled via COM_OBS_AVOID parameter
bool avoidance_system_valid                       # Status of the obstacle avoidance system

bool rc_calibration_in_progress
bool calibration_enabled

bool pre_flight_checks_pass		# true if all checks necessary to arm pass
