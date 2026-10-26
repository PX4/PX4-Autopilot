uint64 timestamp			# time since system start (microseconds)

# Four wheels
float32[4] wheel_speed # [rad/s]
float32[4] wheel_angle # [rad]
uint32[4] counts_per_rev # Revolutions based on encoder counts value
int64[4] encoder_counts # The wheel position, in encoder counts since boot. Positive is forward rotation, negative  is reverse rotation
