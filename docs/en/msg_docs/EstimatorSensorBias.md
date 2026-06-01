---
pageClass: is-wide-page
---

# EstimatorSensorBias (UORB message)

Sensor readings and in-run biases in SI-unit form. Sensor readings are compensated for static offsets,. scale errors, in-run bias and thermal drift (if thermal compensation is enabled and available).

**TOPICS:** estimator_sensor_bias

## Fields

| Name                                                    | Type         | Unit [Frame] | Range/Enum | Description                                                               |
| ------------------------------------------------------- | ------------ | ------------ | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                     | `uint64`     |              |            | time since system start (microseconds)                                    |
| <a id="fld_timestamp_sample"></a>timestamp_sample       | `uint64`     |              |            | the timestamp of the raw data (microseconds)                              |
| <a id="fld_gyro_device_id"></a>gyro_device_id           | `uint32`     |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_gyro_bias"></a>gyro_bias                     | `float32[3]` |              |            | gyroscope in-run bias in body frame (rad/s)                               |
| <a id="fld_gyro_bias_limit"></a>gyro_bias_limit         | `float32`    |              |            | magnitude of maximum gyroscope in-run bias in body frame (rad/s)          |
| <a id="fld_gyro_bias_variance"></a>gyro_bias_variance   | `float32[3]` |              |            |
| <a id="fld_gyro_bias_valid"></a>gyro_bias_valid         | `bool`       |              |            |
| <a id="fld_gyro_bias_stable"></a>gyro_bias_stable       | `bool`       |              |            | true when the gyro bias estimate is stable enough to use for calibration  |
| <a id="fld_accel_device_id"></a>accel_device_id         | `uint32`     |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_accel_bias"></a>accel_bias                   | `float32[3]` |              |            | accelerometer in-run bias in body frame (m/s^2)                           |
| <a id="fld_accel_bias_limit"></a>accel_bias_limit       | `float32`    |              |            | magnitude of maximum accelerometer in-run bias in body frame (m/s^2)      |
| <a id="fld_accel_bias_variance"></a>accel_bias_variance | `float32[3]` |              |            |
| <a id="fld_accel_bias_valid"></a>accel_bias_valid       | `bool`       |              |            |
| <a id="fld_accel_bias_stable"></a>accel_bias_stable     | `bool`       |              |            | true when the accel bias estimate is stable enough to use for calibration |
| <a id="fld_mag_device_id"></a>mag_device_id             | `uint32`     |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_mag_bias"></a>mag_bias                       | `float32[3]` |              |            | magnetometer in-run bias in body frame (Gauss)                            |
| <a id="fld_mag_bias_limit"></a>mag_bias_limit           | `float32`    |              |            | magnitude of maximum magnetometer in-run bias in body frame (Gauss)       |
| <a id="fld_mag_bias_variance"></a>mag_bias_variance     | `float32[3]` |              |            |
| <a id="fld_mag_bias_valid"></a>mag_bias_valid           | `bool`       |              |            |
| <a id="fld_mag_bias_stable"></a>mag_bias_stable         | `bool`       |              |            | true when the mag bias estimate is stable enough to use for calibration   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorSensorBias.msg)

::: details Click here to see original file

```c
#
# Sensor readings and in-run biases in SI-unit form. Sensor readings are compensated for static offsets,
# scale errors, in-run bias and thermal drift (if thermal compensation is enabled and available).
#

uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

# In-run bias estimates (subtract from uncorrected data)

uint32 gyro_device_id           # unique device ID for the sensor that does not change between power cycles
float32[3] gyro_bias            # gyroscope in-run bias in body frame (rad/s)
float32 gyro_bias_limit         # magnitude of maximum gyroscope in-run bias in body frame (rad/s)
float32[3] gyro_bias_variance
bool gyro_bias_valid
bool gyro_bias_stable		# true when the gyro bias estimate is stable enough to use for calibration

uint32 accel_device_id          # unique device ID for the sensor that does not change between power cycles
float32[3] accel_bias           # accelerometer in-run bias in body frame (m/s^2)
float32 accel_bias_limit        # magnitude of maximum accelerometer in-run bias in body frame (m/s^2)
float32[3] accel_bias_variance
bool accel_bias_valid
bool accel_bias_stable		# true when the accel bias estimate is stable enough to use for calibration

uint32 mag_device_id            # unique device ID for the sensor that does not change between power cycles
float32[3] mag_bias             # magnetometer in-run bias in body frame (Gauss)
float32 mag_bias_limit          # magnitude of maximum magnetometer in-run bias in body frame (Gauss)
float32[3] mag_bias_variance
bool mag_bias_valid
bool mag_bias_stable		# true when the mag bias estimate is stable enough to use for calibration
```

:::
