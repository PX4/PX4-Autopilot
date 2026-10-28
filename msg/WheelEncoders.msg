uint64 timestamp			# time since system start (microseconds)

# Wheel order, right before left and rear axle first:
#   four encoders:   0 = rear right,  1 = rear left,  2 = front right, 3 = front left
#   two encoders:    0 = right,       1 = left       (2 and 3 unused)
#   one encoder:     0 = drivetrain (motor or drive shaft, reported as wheel rotation), 1-3 unused
# Unused entries: counts_per_rev = 0, wheel_speed = NaN, wheel_angle = NaN
float32[4] wheel_speed # [rad/s]
float32[4] wheel_angle # [rad]
uint32[4] counts_per_rev # Revolutions based on encoder counts value
int64[4] encoder_counts # The wheel position, in encoder counts since boot. Positive is forward rotation, negative  is reverse rotation
