# Wheel encoder readings
#
# Angular speed, unwrapped angle and raw encoder counts for up to four wheels.
# Published by the Roboclaw driver (one instance per controller, two encoder channels each)
# and by the Gazebo bridge in simulation.
# Streamed to MAVLink as WHEEL_DISTANCE, logged, and exported over uXRCE-DDS.
#
# Wheel order, right before left and rear axle first:
# - Four encoders: 0 = rear right, 1 = rear left, 2 = front right, 3 = front left
# - Two encoders: 0 = right, 1 = left (2 and 3 unused)
# - One encoder: 0 = drivetrain (motor or drive shaft, reported as wheel rotation), 1-3 unused
#
# Unused entries have counts_per_rev = 0.

uint64 timestamp			# time since system start (microseconds)
float32[4] wheel_speed # [rad/s]
float32[4] wheel_angle # [rad]
uint32[4] counts_per_rev # Revolutions based on encoder counts value
int64[4] encoder_counts # The wheel position, in encoder counts since boot. Positive is forward rotation, negative  is reverse rotation
