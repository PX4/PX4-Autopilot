# TODO: How should this mapping be done? What if there's a 6-wheeled (or more) rover?
uint8_t FRONT_RIGHT = 0
uint8_t FRONT_LEFT = 1
uint8_t REAR_RIGHT = 2
uint8_t REAR_LEFT = 3

uint64 timestamp			# time since system start (microseconds)

# TODO: How large should the arrays be? What if we have a 6-wheeled rover?
bool[4] has_encoder         # True for each wheel that has an encoder
int64[4] encoder_position   # The wheel position, in encoder counts since boot. Positive is forward rotation, negative is reverse rotation
float[4] speed              # Speed of each wheel, in encoder counts per second. Positive is forward, negative is reverse

# TODO: Should this be just one uint32, assuming each wheel has the same encoder?
uint32[4] pulses_per_rev    # Number of pulses per revolution for each wheel
