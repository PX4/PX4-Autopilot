# Differential-pressure (airspeed) sensor
#
# This is populated by airspeed sensor drivers and used by the sensor module to calculate airspeed.
# The information is published in the `SCALED_PRESSURE_n` MAVLink messages (along with information from a corresponding `SensorBaro` instance).

uint64 timestamp         # [us] Time of publication (since system start)
uint64 timestamp_sample  # [us] Time of raw data capture

uint32 device_id                  # [-] Unique device ID for the sensor that does not change between power cycles
float32 differential_pressure_pa  # [Pa] Differential pressure reading (may be negative)
float32 temperature               # [degC] [@invalid NaN if unknown] Temperature
uint32 error_count                # [-] Number of errors detected by driver
