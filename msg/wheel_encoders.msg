uint64 timestamp			# time since system start (microseconds)

int64 encoder_position   # The wheel position, in encoder counts since boot. Positive is forward rotation, negative is reverse rotation
int32 speed              # Speed of each wheel, in encoder counts per second. Positive is forward, negative is reverse
uint32 pulses_per_rev    # Number of pulses per revolution for each wheel
