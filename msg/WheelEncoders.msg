# The WHEEL_DISTANCE stream supports a single publisher: with several instances (e.g. two Roboclaw
# controllers) they all fill the same slots and only the last updated instance is streamed.

uint64 timestamp			# time since system start (microseconds)
float32[4] wheel_speed # [rad/s] Wheel angular speed. Positive is forward rotation
float32[4] wheel_angle # [rad] Unwrapped wheel angle since boot, 2 * pi * encoder_counts / counts_per_rev
uint32[4] counts_per_rev # [counts/rev] Encoder counts per wheel revolution
int64[4] encoder_counts # The wheel position, in encoder counts since boot. Positive is forward rotation, negative is reverse rotation
