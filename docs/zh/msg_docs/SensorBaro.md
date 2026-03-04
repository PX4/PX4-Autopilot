---
pageClass: is-wide-page
---

# SensorBaro (UORB message)

Barometer sensor.

This is populated by barometer drivers and used by the EKF2 estimator.
The information is published in the `SCALED_PRESSURE_n` MAVLink messages (along with information from a corresponding `DifferentialPressure` instance).

**TOPICS:** sensor_baro

## Fields

| 参数名                                   | 类型        | Unit [Frame] | Range/Enum | 描述                                                                        |
| ------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| timestamp                             | `uint64`  | us                                                               |            | Time of publication (since system start)               |
| timestamp_sample | `uint64`  | us                                                               |            | Time of raw data capture                                                  |
| device_id        | `uint32`  |                                                                  |            | Unique device ID for the sensor that does not change between power cycles |
| pressure                              | `float32` | Pa                                                               |            | Static pressure measurement                                               |
| temperature                           | `float32` | degC                                                             |            | Temperature.                                              |
| error_count      | `uint32`  |                                                                  |            | Number of errors detected by driver.                      |

## Constants

| 参数名                                                                                         | 类型      | 值 | 描述 |
| ------------------------------------------------------------------------------------------- | ------- | - | -- |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4 |    |

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
float32 temperature  # [degC] Temperature.
uint32 error_count   # [-] Number of errors detected by driver.

uint8 ORB_QUEUE_LENGTH = 4
```

:::
