uint64 timestamp	# time since system start (microseconds)
uint32 device_id	# unique device ID for the sensor that does not change between power cycles

uint64 error_count

float32 temperature

uint8 rotation

# clipping per axis?
uint64[3] clipping

uint16 measure_rate
uint16 sample_rate

float32 full_scale_range

float32 coning_vibration	# Level of coning vibration in the IMU delta angles (rad^2)

float32 high_frequency_vibration # high frequency vibration level in the IMU delta angle data (rad)
