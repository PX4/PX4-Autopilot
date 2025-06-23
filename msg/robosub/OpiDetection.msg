uint64 timestamp		# time since system start (microseconds)

# type of OPI
uint8 OPI_BOUY = 0
uint8 OPI_ONE = 1
uint8 OPI_TWO = 2
uint8 OPI_THREE = 3
uint8 OPI_FOUR = 4
uint8 OPI_FIVE = 5
uint8 OPI_SIX = 6
uint8 OPI_SEVEN = 7
uint8 OPI_EIGHT = 8
uint8 OPI_GREEN = 9
uint8 OPI_RED = 10

# colors
uint8 COLOR_ORANGE = 0
uint8 COLOR_WHITE = 1
uint8 COLOR_RED = 2
uint8 COLOR_BLACK = 3
uint8 COLOR_YELLOW = 4

# OPI information
uint64 distance
float32 angle_hor
float32 angle_ver
uint8 color # see COLOR_* for colors
uint8 confidence
uint32 direction_angle


uint8 opi # see OPI_* for types

