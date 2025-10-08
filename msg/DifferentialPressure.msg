# Differential-pressure sensor
#
# ?

uint64 timestamp         # [us] Time since system start
uint64 timestamp_sample  # [us] Timestamp of the raw data

uint32 device_id                  # [-] Unique device ID for the sensor that does not change between power cycles
float32 differential_pressure_pa  # [Pa] Differential pressure reading in Pascals (may be negative)
float32 temperature               # [degC] [@invalid NaN if unknown] Temperature
uint32 error_count                # [-] Number of errors detected by driver
