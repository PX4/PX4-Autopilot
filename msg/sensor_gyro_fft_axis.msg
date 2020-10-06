uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)

uint16 samples             # number of valid samples

float32 resolution_hz

int16[128] fft

uint16 peak_index
uint16 peak_index_quinns

float32 peak_frequency
float32 peak_frequency_quinns

uint8 AXIS_X = 0
uint8 AXIS_Y = 1
uint8 AXIS_Z = 2
uint8 axis
