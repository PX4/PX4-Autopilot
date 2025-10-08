# Baro sensor
# 
# ? This is used in the `SCALED_PRESSURE_n` MAVLink messages (values must be zero-filled).

uint64 timestamp         # [us] Time since system start
uint64 timestamp_sample  # [us] Timestamp of the raw data

uint32 device_id     # [-] Unique device ID for the sensor that does not change between power cycles
float32 pressure     # [Pa] Static pressure measurement
float32 temperature  # [degC] Temperature.
uint32 error_count   # [-] Number of errors detected by driver.

uint8 ORB_QUEUE_LENGTH = 4
