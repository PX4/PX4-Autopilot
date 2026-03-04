---
pageClass: is-wide-page
---

# AirspeedValidated (повідомлення UORB)

Validated airspeed.

Provides information about airspeed (indicated, true, calibrated) and the source of the data.
Used by controllers, estimators and for airspeed reporting to operator.

**TOPICS:** airspeed_validated

## Fields

| Назва                                                                                                                                     | Тип       | Unit [Frame] | Range/Enum        | Опис                                                                                                                                                            |
| ----------------------------------------------------------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ----------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                                                                 | `uint64`  | us                                                               |                   | Time since system start                                                                                                                                         |
| indicated_airspeed_m_s                                                     | `float32` | m/s                                                              |                   | Indicated airspeed (IAS) (Invalid: NaN)                                                                   |
| calibrated_airspeed_m_s                                                    | `float32` | m/s                                                              |                   | Calibrated airspeed (CAS) (Invalid: NaN)                                                                  |
| true_airspeed_m_s                                                          | `float32` | m/s                                                              |                   | True airspeed (TAS) (Invalid: NaN)                                                                        |
| airspeed_source                                                                                                      | `int8`    |                                                                  | [SOURCE](#SOURCE) | Source of currently published airspeed values                                                                                                                   |
| calibrated_ground_minus_wind_m_s | `float32` | m/s                                                              |                   | CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption (Invalid: NaN) |
| calibraded_airspeed_synth_m_s                         | `float32` | m/s                                                              |                   | Synthetic airspeed (Invalid: NaN)                                                                                            |
| airspeed_derivative_filtered                                                                    | `float32` | m/s^2                                                            |                   | Filtered indicated airspeed derivative                                                                                                                          |
| throttle_filtered                                                                                                    | `float32` |                                                                  |                   | Filtered fixed-wing throttle                                                                                                                                    |
| pitch_filtered                                                                                                       | `float32` | rad                                                              |                   | Filtered pitch                                                                                                                                                  |

## Enums

### SOURCE {#SOURCE}

| Назва                                                                                                                            | Тип    | Значення | Опис                    |
| -------------------------------------------------------------------------------------------------------------------------------- | ------ | -------- | ----------------------- |
| <a href="#SOURCE_DISABLED"></a> SOURCE_DISABLED                                                             | `int8` | -1       | Disabled                |
| <a href="#SOURCE_GROUND_MINUS_WIND"></a> SOURCE_GROUND_MINUS_WIND | `int8` | 0        | Ground speed minus wind |
| <a href="#SOURCE_SENSOR_1"></a> SOURCE_SENSOR_1                                        | `int8` | 1        | Sensor 1                |
| <a href="#SOURCE_SENSOR_2"></a> SOURCE_SENSOR_2                                        | `int8` | 2        | Sensor 2                |
| <a href="#SOURCE_SENSOR_3"></a> SOURCE_SENSOR_3                                        | `int8` | 3        | Sensor 3                |
| <a href="#SOURCE_SYNTHETIC"></a> SOURCE_SYNTHETIC                                                           | `int8` | 4        | Synthetic airspeed      |

## Constants

| Назва                                                                | Тип      | Значення | Опис |
| -------------------------------------------------------------------- | -------- | -------- | ---- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 1        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/AirspeedValidated.msg)

:::details
Click here to see original file

```c
# Validated airspeed
#
# Provides information about airspeed (indicated, true, calibrated) and the source of the data.
# Used by controllers, estimators and for airspeed reporting to operator.


uint32 MESSAGE_VERSION = 1

uint64 timestamp  # [us] Time since system start

float32 indicated_airspeed_m_s   # [m/s] [@invalid NaN] Indicated airspeed (IAS)
float32 calibrated_airspeed_m_s  # [m/s] [@invalid NaN] Calibrated airspeed (CAS)
float32 true_airspeed_m_s        # [m/s] [@invalid NaN] True airspeed (TAS)

int8 airspeed_source               # [@enum SOURCE] Source of currently published airspeed values
int8 SOURCE_DISABLED = -1          # Disabled
int8 SOURCE_GROUND_MINUS_WIND = 0  # Ground speed minus wind
int8 SOURCE_SENSOR_1 = 1           # Sensor 1
int8 SOURCE_SENSOR_2 = 2           # Sensor 2
int8 SOURCE_SENSOR_3 = 3           # Sensor 3
int8 SOURCE_SYNTHETIC = 4          # Synthetic airspeed

float32 calibrated_ground_minus_wind_m_s  # [m/s] [@invalid NaN] CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption
float32 calibraded_airspeed_synth_m_s     # [m/s] [@invalid NaN] Synthetic airspeed
float32 airspeed_derivative_filtered      # [m/s^2] Filtered indicated airspeed derivative
float32 throttle_filtered                 # [-] Filtered fixed-wing throttle
float32 pitch_filtered                    # [rad] Filtered pitch
```

:::
