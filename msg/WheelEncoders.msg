uint64 timestamp			# time since system start (microseconds)

# Two wheels: 0 right, 1 left
float32[2] wheel_speed # [rad/s]
float32[2] wheel_angle # [rad]
uint32[2] counts_per_rev # Revolutions based on encoder counts value
int64[2] encoder_position # The wheel position, in encoder counts since boot. Positive is forward rotation, negative  is reverse rotation
