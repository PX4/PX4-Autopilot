---
pageClass: is-wide-page
---

# SensorBaro (UORB message)

Barometer sensor.

This is populated by barometer drivers and used by the EKF2 estimator.
The information is published in the `SCALED_PRESSURE_n` MAVLink messages (along with information from a corresponding `DifferentialPressure` instance).

**TOPICS:** sensor_baro

## Fields

| Назва                                                                  | Тип       | Unit [Frame] | Range/Enum | Опис                                                                      |
| ---------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                    | `uint64`  | us                                                               |            | Time of publication (since system start)               |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`  | us                                                               |            | Time of raw data capture                                                  |
| <a id="fld_device_id"></a>device_id               | `uint32`  |                                                                  |            | Unique device ID for the sensor that does not change between power cycles |
| <a id="fld_pressure"></a>pressure                                      | `float32` | Pa                                                               |            | Static pressure measurement                                               |
| <a id="fld_temperature"></a>temperature                                | `float32` | °C                                                               |            | Temperature.                                              |
| <a id="fld_error_count"></a>error_count           | `uint32`  |                                                                  |            | Number of errors detected by driver.                      |

## Constants

| Назва                                                                                     | Тип     | Значення | Опис |
| ----------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorBaro.msg)

:::details
Click here to see original file

```c
# Barometer sensor
#
# This is populated by barometer drivers and used by the EKF2 estimator.
# The information is published in the `SCALED_PRESSURE_n` MAVLink messages (along with information from a corresponding `DifferentialPressure` instance).

uint64 timestamp         # [us] Time of publication (since system start)
uint64 timestamp_sample  # [us] Time of raw data capture

uint32 device_id     # [-] Unique device ID for the sensor that does not change between power cycles
float32 pressure     # [Pa] Static pressure measurement
float32 temperature  # [°C] Temperature.
uint32 error_count   # [-] Number of errors detected by driver.

uint8 ORB_QUEUE_LENGTH = 4
```

:::
