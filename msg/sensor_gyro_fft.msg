uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 sensor_sample_rate_hz
float32 resolution_hz

float32[4] peak_frequencies_x # x axis peak frequencies
float32[4] peak_frequencies_y # y axis peak frequencies
float32[4] peak_frequencies_z # z axis peak frequencies

float32[4] peak_snr_x # x axis peak SNR
float32[4] peak_snr_y # y axis peak SNR
float32[4] peak_snr_z # z axis peak SNR
