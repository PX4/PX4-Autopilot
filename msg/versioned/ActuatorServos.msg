# Servo control message
#
# Normalised output setpoint for up to 15 servos.
# Published by the vehicle's allocation and consumed by the actuator output drivers.

uint32 MESSAGE_VERSION = 1

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Sampling timestamp of the data this control response is based on

uint8 NUM_CONTROLS = 15
float32[15] control # [-] [@range -1, 1] Normalized output. 1 means maximum positive position. -1 maximum negative position (if not supported by the output, <0 maps to NaN). NaN maps to disarmed.
