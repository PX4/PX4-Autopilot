---
pageClass: is-wide-page
---

# VehicleAirData (повідомлення UORB)

Vehicle air data.

Data from the currently selected barometer (plus ambient temperature from the source specified in temperature_source).
Includes calculated data such as barometric altitude and air density.

**TOPICS:** vehicle_airdata

## Fields

| Назва                                                      | Тип       | Unit [Frame] | Range/Enum | Опис                                                                                                                                                                              |
| ---------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                  | `uint64`  | us                                                               |            | Time since system start                                                                                                                                                           |
| timestamp_sample                      | `uint64`  | us                                                               |            | Timestamp of the raw data                                                                                                                                                         |
| baro_device_id   | `uint32`  |                                                                  |            | Unique device ID for the selected barometer                                                                                                                                       |
| baro_alt_meter   | `float32` | m [MSL]      |            | Altitude above MSL calculated from temperature compensated baro sensor data using an ISA corrected for sea level pressure SENS_BARO_QNH |
| baro_pressure_pa | `float32` | Pa                                                               |            | Absolute pressure                                                                                                                                                                 |
| ambient_temperature                   | `float32` | degC                                                             |            | Температура середовища                                                                                                                                                            |
| temperature_source                    | `uint8`   |                                                                  |            | Source of temperature data: 0: Default Temperature (15°C), 1: External Baro, 2: Airspeed       |
| rho                                                        | `float32` | kg/m^3                                                           |            | Air density                                                                                                                                                                       |
| calibration_count                     | `uint8`   |                                                                  |            | Calibration changed counter. Monotonically increases whenever calibration changes.                                                                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleAirData.msg)

:::details
Click here to see original file

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
