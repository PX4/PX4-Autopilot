uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)

float32 resolution_hz

uint8[3] peak_index_0
uint8[3] peak_index_1

float32[3] peak_frequency_0 # largest frequency peak found per sensor axis (0 if none)
float32[3] peak_frequency_1 # second largest frequency peak found per sensor axis (0 if none)

uint8[3] peak_index_quinns
float32[3] peak_frequency_quinns
