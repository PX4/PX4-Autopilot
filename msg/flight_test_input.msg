
uint8 TYPE_NONE = 0
uint8 TYPE_FAILURE = 1
uint8 TYPE_GAUSSIAN = 2
uint8 TYPE_NOISE_WHITE  = 3
uint8 TYPE_NOISE_PINK   = 4
uint8 TYPE_NOISE_BROWN  = 5
uint8 TYPE_NOISE_BLUE   = 6
uint8 TYPE_NOISE_VIOLET = 7
uint8 TYPE_NOISE_GREY   = 8


uint64 timestamp

char[9] name		# max 8 characters (null terminated)

uint8 type

float32 input

float32 added_noise

float32 output
