
uint64 timestamp            # time since system start (microseconds)

uint64 timestamp_sample     # the timestamp of the raw data (microseconds)

uint32 device_id            # unique device ID for the selected magnetometer

float32[3] magnetometer_raw_rms  # Raw magnetic field RMS amplitude

float32[3] magnetometer_filtered_rms  # Filtered magnetic field RMS amplitude

# Ideally we publish this externally in sensor_mag as this is published in vehicle_magnetometer.cpp (only 80Hz)
# However, this seems to stick at a low rate, so this will do for now.
float32[3] magnetometer_raw # Raw magnetometer data
