# Reply to SetpointConfig

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start

uint16 type # See SetpointConfig::TYPE_*

uint8 source_id # nav_state of the mode that sent the SetpointConfig

uint8 RESULT_SUCCESS = 0
uint8 RESULT_FAILURE_OTHER = 1
uint8 RESULT_UNSUPPORTED = 2 # Setpoint type is unsupported for the current vehicle
uint8 RESULT_UNKNOWN_SETPOINT_TYPE = 3 # The setpoint type is not known
uint8 result # [@enum RESULT]

# Mode requirements for using the given setpoint type. A mode will use these and apply them to the arming check reply (PX4 does not do that itself).
# Certain setpoint types can be used in a reduced way, for example a TrajectorySetpoint without position control. In that case PX4 still sets all requirement flags, and the mode will then ignore mode_req_local_position.
bool mode_req_angular_velocity
bool mode_req_attitude
bool mode_req_local_alt
bool mode_req_local_position
