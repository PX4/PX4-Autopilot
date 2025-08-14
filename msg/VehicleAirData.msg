# Vehicle air data
#
# Data from the currently selected barometer (plus ambient temperature from the source specified in temperature_source).
# Includes calculated data such as barometric altitude and air density.

uint64 timestamp             # [us] Time since system start
uint64 timestamp_sample      # [us] Timestamp of the raw data
uint32 baro_device_id        # Unique device ID for the selected barometer
float32 baro_alt_meter       # [m] [@frame MSL] Altitude above MSL calculated from temperature compensated baro sensor data using an ISA corrected for sea level pressure SENS_BARO_QNH
float32 baro_pressure_pa     # [Pa] Absolute pressure
float32 ambient_temperature  # [degC] Ambient temperature
uint8 temperature_source     # Source of temperature data: 0: Default Temperature (15°C), 1: External Baro, 2: Airspeed
float32 rho                  # [kg/m^3] Air density
uint8 calibration_count      # Calibration changed counter. Monotonically increases whenever calibration changes.
