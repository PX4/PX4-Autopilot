uint64 error_count
float32 x # [Gauss]
float32 y # [Gauss]
float32 z # [Gauss]
float32 range_ga
float32 scaling
float32 temperature

int16 x_raw # [milli Gauss]
int16 y_raw # [milli Gauss]
int16 z_raw # [milli Gauss]

uint32 device_id	# unique device ID for the sensor that does not change between power cycles
bool  is_external  # if true the mag is external (i.e. not built into the board)
