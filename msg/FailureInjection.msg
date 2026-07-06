#Failure injection configuration.
#
# Currently active failure-injection configuration, published by the failure
# injection manager (the sole subscriber to vehicle_command INJECT_FAILURE).
# Republished only when the configuration changes, so command spam on
# vehicle_command cannot propagate to the consumers applying the failures.

uint64 timestamp # [us] Time since system start

uint8 MAX_FAILURES = 4 # maximum number of simultaneous failures

uint8 count # number of valid entries in the arrays below

uint8[4] unit # [@enum FAILURE_UNIT] Affected component per entry
# Failure unit (affected component). Mirrors MAVLink FAILURE_UNIT and the
# FAILURE_UNIT_* values in vehicle_command.
uint8 FAILURE_UNIT_SENSOR_GYRO = 0
uint8 FAILURE_UNIT_SENSOR_ACCEL = 1
uint8 FAILURE_UNIT_SENSOR_MAG = 2
uint8 FAILURE_UNIT_SENSOR_BARO = 3
uint8 FAILURE_UNIT_SENSOR_GPS = 4
uint8 FAILURE_UNIT_SENSOR_OPTICAL_FLOW = 5
uint8 FAILURE_UNIT_SENSOR_VIO = 6
uint8 FAILURE_UNIT_SENSOR_DISTANCE_SENSOR = 7
uint8 FAILURE_UNIT_SENSOR_AIRSPEED = 8
uint8 FAILURE_UNIT_SYSTEM_BATTERY = 100
uint8 FAILURE_UNIT_SYSTEM_MOTOR = 101
uint8 FAILURE_UNIT_SYSTEM_SERVO = 102
uint8 FAILURE_UNIT_SYSTEM_AVOIDANCE = 103
uint8 FAILURE_UNIT_SYSTEM_RC_SIGNAL = 104
uint8 FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL = 105

uint16[4] instance_mask # Bit i targets instance (i+1); 0xFFFF = all instances

uint8[4] failure_type # [@enum FAILURE_TYPE] failure mode per entry
# Failure mode.
# Mirrors MAVLink FAILURE_TYPE and the FAILURE_TYPE_* values in vehicle_command.
uint8 FAILURE_TYPE_OK = 0
uint8 FAILURE_TYPE_OFF = 1
uint8 FAILURE_TYPE_STUCK = 2
uint8 FAILURE_TYPE_GARBAGE = 3
uint8 FAILURE_TYPE_WRONG = 4
uint8 FAILURE_TYPE_SLOW = 5
uint8 FAILURE_TYPE_DELAYED = 6
uint8 FAILURE_TYPE_INTERMITTENT = 7
