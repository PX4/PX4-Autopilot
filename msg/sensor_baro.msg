uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 pressure          # static pressure measurement in millibar

float32 temperature       # temperature in degrees Celsius

uint32 error_count

bool is_external	# if true the mag is external (i.e. not built into the board)

uint8 ORB_QUEUE_LENGTH = 4
