---
pageClass: is-wide-page
---

# DifferentialPressure (UORB message)

Differential-pressure (airspeed) sensor.

This is populated by airspeed sensor drivers and used by the sensor module to calculate airspeed.
The information is published in the `SCALED_PRESSURE_n` MAVLink messages (along with information from a corresponding `SensorBaro` instance).

**TOPICS:** differential_pressure

## Fields

| 명칭                                                                 | 형식        | Unit [Frame] | Range/Enum | 설명                                                                        |
| ------------------------------------------------------------------ | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| timestamp                                                          | `uint64`  | us                                                               |            | Time of publication (since system start)               |
| timestamp_sample                              | `uint64`  | us                                                               |            | Time of raw data capture                                                  |
| device_id                                     | `uint32`  |                                                                  |            | Unique device ID for the sensor that does not change between power cycles |
| differential_pressure_pa | `float32` | Pa                                                               |            | Differential pressure reading (may be negative)        |
| temperature                                                        | `float32` | degC                                                             |            | Temperature (Invalid: NaN if unknown)  |
| error_count                                   | `uint32`  |                                                                  |            | Number of errors detected by driver                                       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DifferentialPressure.msg)

:::details
Click here to see original file

```c
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
```

:::
