---
pageClass: is-wide-page
---

# VehicleAirData (UORB message)

Vehicle air data.

Data from the currently selected barometer (plus ambient temperature from the source specified in temperature_source).
Includes calculated data such as barometric altitude and air density.

**TOPICS:** vehicle_air_data

## Fields

| Name                                                    | Type      | Unit [Frame] | Range/Enum | Description                                                                                                                             |
| ------------------------------------------------------- | --------- | ------------ | ---------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                     | `uint64`  | us           |            | Time since system start                                                                                                                 |
| <a id="fld_timestamp_sample"></a>timestamp_sample       | `uint64`  | us           |            | Timestamp of the raw data                                                                                                               |
| <a id="fld_baro_device_id"></a>baro_device_id           | `uint32`  |              |            | Unique device ID for the selected barometer                                                                                             |
| <a id="fld_baro_alt_meter"></a>baro_alt_meter           | `float32` | m [MSL]      |            | Altitude above MSL calculated from temperature compensated baro sensor data using an ISA corrected for sea level pressure SENS_BARO_QNH |
| <a id="fld_baro_pressure_pa"></a>baro_pressure_pa       | `float32` | Pa           |            | Absolute pressure                                                                                                                       |
| <a id="fld_ambient_temperature"></a>ambient_temperature | `float32` | degC         |            | Ambient temperature                                                                                                                     |
| <a id="fld_temperature_source"></a>temperature_source   | `uint8`   |              |            | Source of temperature data: 0: Default Temperature (15°C), 1: External Baro, 2: Airspeed                                                |
| <a id="fld_rho"></a>rho                                 | `float32` | kg/m^3       |            | Air density                                                                                                                             |
| <a id="fld_calibration_count"></a>calibration_count     | `uint8`   |              |            | Calibration changed counter. Monotonically increases whenever calibration changes.                                                      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleAirData.msg)

::: details Click here to see original file

```c
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
```

:::
