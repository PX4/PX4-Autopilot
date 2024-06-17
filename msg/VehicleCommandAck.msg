# Vehicle Command Ackonwledgement uORB message.
# Used for acknowledging the vehicle command being received.
# Follows the MAVLink COMMAND_ACK message definition

uint64 timestamp		# time since system start (microseconds)

# Result cases. This follows the MAVLink MAV_RESULT enum definition
uint8 VEHICLE_CMD_RESULT_ACCEPTED = 0 # Command is valid (is supported and has valid parameters), and was executed
uint8 VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1 # Command is valid, but cannot be executed at this time
uint8 VEHICLE_CMD_RESULT_DENIED = 2 # Command is invalid (is supported but has invalid parameters)
uint8 VEHICLE_CMD_RESULT_UNSUPPORTED = 3 # Command is not supported (unknown)
uint8 VEHICLE_CMD_RESULT_FAILED = 4 # Command is valid, but execution has failed
uint8 VEHICLE_CMD_RESULT_IN_PROGRESS = 5 # Command is valid and is being executed
uint8 VEHICLE_CMD_RESULT_CANCELLED = 6 # Command has been cancelled (as a result of receiving a COMMAND_CANCEL message)
uint8 VEHICLE_CMD_RESULT_COMMAND_LONG_ONLY = 7 # Command is only accepted when sent as a COMMAND_LONG
uint8 VEHICLE_CMD_RESULT_COMMAND_INT_ONLY = 8 # Command is only accepted when sent as a COMMAND_INT
uint8 VEHICLE_CMD_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME = 9 # Command is invalid because a frame is required and the specified frame is not supported

# Arming denied specific cases
uint16 ARM_AUTH_DENIED_REASON_GENERIC = 0
uint16 ARM_AUTH_DENIED_REASON_NONE = 1
uint16 ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2
uint16 ARM_AUTH_DENIED_REASON_TIMEOUT = 3
uint16 ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4
uint16 ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5

uint8 ORB_QUEUE_LENGTH = 4

uint32 command						# Command that is being acknowledged
uint8 result						# Command result
uint8 result_param1					# Also used as progress[%], it can be set with the reason why the command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS
int32 result_param2					# Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.
uint8 target_system
uint16 target_component 				# Target component / mode executor

bool from_external					# Indicates if the command came from an external source
