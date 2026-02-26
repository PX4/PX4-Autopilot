# Vehicle Command Acknowledgement uORB message.
#
# Used for acknowledging the vehicle command being received.
# Follows the MAVLink COMMAND_ACK message definition

uint32 MESSAGE_VERSION = 1

uint64 timestamp # [us] time since system start

uint8 ORB_QUEUE_LENGTH = 8

uint32 command # [-] Command that is being acknowledged

uint8 result # [@enum VEHICLE_CMD_RESULT] Command result
# VEHICLE_CMD_RESULT Result cases. Follows the MAVLink MAV_RESULT enum definition
uint8 VEHICLE_CMD_RESULT_ACCEPTED = 0 # Command ACCEPTED and EXECUTED
uint8 VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1 # Command TEMPORARY REJECTED/DENIED
uint8 VEHICLE_CMD_RESULT_DENIED = 2 # Command PERMANENTLY DENIED
uint8 VEHICLE_CMD_RESULT_UNSUPPORTED = 3 # Command UNKNOWN/UNSUPPORTED
uint8 VEHICLE_CMD_RESULT_FAILED = 4 # Command executed, but failed
uint8 VEHICLE_CMD_RESULT_IN_PROGRESS = 5 # Command being executed
uint8 VEHICLE_CMD_RESULT_CANCELLED = 6 # Command Canceled
uint8 VEHICLE_CMD_RESULT_COMMAND_LONG_ONLY = 7 # Command is only accepted when sent as a COMMAND_LONG
uint8 VEHICLE_CMD_RESULT_COMMAND_INT_ONLY = 8 # Command is only accepted when sent as a COMMAND_INT
uint8 VEHICLE_CMD_RESULT_UNSUPPORTED_MAV_FRAME = 9 # Command does not support specified frame

uint8 result_param1 # [-] Can be set with the reason why the command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS (%)

int32 result_param2 # [enum ARM_AUTH_DENIED_REASON] Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied, or what ARM_AUTH_DENIED_REASON
# Arming denied specific cases
uint16 ARM_AUTH_DENIED_REASON_GENERIC = 0
uint16 ARM_AUTH_DENIED_REASON_NONE = 1
uint16 ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2
uint16 ARM_AUTH_DENIED_REASON_TIMEOUT = 3
uint16 ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4
uint16 ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5

uint8 target_system # [-] Target system
uint16 target_component # Target component / mode executor

bool from_external # Indicates if the command came from an external source
