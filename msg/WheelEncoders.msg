# Wheel encoder readings
#
# Angular speed, unwrapped angle and raw encoder counts for up to four wheels.
# Published by the Roboclaw driver (one instance per controller, two encoder channels each)
# and by the Gazebo bridge in simulation.
# Streamed to MAVLink as WHEEL_DISTANCE, logged, and exported over uXRCE-DDS.
#
# Wheel order follows the PX4 motor index: entry i is the wheel driven by Motor(i+1) in the
# rover's actuator configuration.
# - Mecanum: 0 = front right, 1 = front left, 2 = rear right, 3 = rear left
# - Differential: 0 = right, 1 = left (2 and 3 unused)
# - Ackermann: 0 = drivetrain (motor or drive shaft, reported as wheel rotation), 1-3 unused
#
# Unused entries have counts_per_rev = 0.
#
# The WHEEL_DISTANCE stream supports a single publisher: with several instances (e.g. two Roboclaw
# controllers) they all fill the same slots and only the most recently updated instance is streamed.

uint64 timestamp # [us] Time since system start
float32[4] wheel_speed # [rad/s] Wheel angular speed. Positive is forward rotation.
float32[4] wheel_angle # [rad] Unwrapped wheel angle since boot, 2 * pi * encoder_counts / counts_per_rev
uint32[4] counts_per_rev # [-] Encoder counts per wheel revolution
int64[4] encoder_counts # [-] Wheel position in encoder counts since boot. Positive is forward rotation; negative is reverse rotation.
