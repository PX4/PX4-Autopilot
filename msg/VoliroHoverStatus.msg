# State and readiness of the fully onboard Voliro hover flight mode.
uint64 timestamp                         # [us] Time since system start

uint8 PHASE_INACTIVE = 0
uint8 PHASE_READY = 1
uint8 PHASE_LAUNCH = 2
uint8 PHASE_TAKEOFF = 3
uint8 PHASE_HOLD = 4
uint8 PHASE_LAND = 5
uint8 PHASE_LANDED = 6

uint8 FAILURE_NONE = 0
uint8 FAILURE_DISABLED = 1
uint8 FAILURE_ODOMETRY = 2
uint8 FAILURE_TILT_FEEDBACK = 3
uint8 FAILURE_TILT_NOT_CENTERED = 4
uint8 FAILURE_NOT_LANDED = 5
uint8 FAILURE_NOT_REGISTERED = 6

float32[3] position_setpoint             # [m] Current NED reference
float32 target_height                    # [m] Height above captured ground position
int8 mode_id                             # Commander external navigation-state ID
uint8 phase                              # PHASE_*
uint8 failure_reason                     # FAILURE_*
bool registered
bool active                              # Mode owns the Voliro setpoint stream
bool can_arm_and_run
