uint64 timestamp				# time since system start (microseconds)

# Topic to test individual actuator output functions
# Note: this is only used with SYS_CTRL_ALLOC=1

uint8 ACTION_RELEASE_CONTROL = 0	# exit test mode for the given function
uint8 ACTION_DO_CONTROL = 1			# enable actuator test mode

uint8 FUNCTION_MOTOR1 = 101
uint8 MAX_NUM_MOTORS  = 12
uint8 FUNCTION_SERVO1 = 201
uint8 MAX_NUM_SERVOS  = 8

uint8 action					# one of ACTION_*
uint16 function					# actuator output function
float32 value					# range: [-1, 1], where 1 means maximum positive output,
								# 0 to center servos or minimum motor thrust,
                   				# -1 maximum negative (if not supported by the motors, <0 maps to NaN),
                   				# and NaN maps to disarmed (stop the motors)
uint32 timeout_ms				# timeout in ms after which to exit test mode (if 0, do not time out)

uint8 ORB_QUEUE_LENGTH = 8
