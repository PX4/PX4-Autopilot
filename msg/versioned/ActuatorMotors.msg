# Motor control message
#
# Normalised thrust setpoint for up to 12 motors.
# Published by the vehicle's allocation and consumed by the ESC protocol drivers e.g. PWM, DSHOT, UAVCAN.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # Time since system start [us]
uint64 timestamp_sample # Timestamp of the data this control response is based on was sampled [us]

uint16 reversible_flags # Bitset indicating which motors are configured to be reversible

uint8 ACTUATOR_FUNCTION_MOTOR1 = 101

uint8 NUM_CONTROLS = 12
float32[12] control # Normalized thrust. [@range -1, 1] where 1 means maximum positive thrust, -1 maximum negative (if not supported by the output, <0 maps to NaN). NaN maps to disarmed (stop the motors).
