uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)
float32 scale

uint8 samples             # number of valid samples

uint8 peak_index

float32 peak_frequency_0
float32 peak_frequency_1
float32 peak_frequency_2
