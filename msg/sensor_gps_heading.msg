uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id                # unique device ID for the sensor that does not change between power cycles

float32 heading
float32 heading_accuracy                 # [1e-5 deg] Accuracy of the heading of the relative position vector

float32[3] relative_position    # relative position vector
float32[3] relative_position_accuracy
