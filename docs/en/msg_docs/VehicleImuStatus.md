---
pageClass: is-wide-page
---

# VehicleImuStatus (UORB message)

**TOPICS:** vehicle_imu_status

## Fields

| Name                                                                | Type         | Unit [Frame] | Range/Enum | Description                                                               |
| ------------------------------------------------------------------- | ------------ | ------------ | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                 | `uint64`     |              |            | time since system start (microseconds)                                    |
| <a id="fld_accel_device_id"></a>accel_device_id                     | `uint32`     |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_gyro_device_id"></a>gyro_device_id                       | `uint32`     |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_accel_clipping"></a>accel_clipping                       | `uint32[3]`  |              |            | total clipping per axis                                                   |
| <a id="fld_gyro_clipping"></a>gyro_clipping                         | `uint32[3]`  |              |            | total clipping per axis                                                   |
| <a id="fld_accel_error_count"></a>accel_error_count                 | `uint32`     |              |            |
| <a id="fld_gyro_error_count"></a>gyro_error_count                   | `uint32`     |              |            |
| <a id="fld_accel_rate_hz"></a>accel_rate_hz                         | `float32`    |              |            |
| <a id="fld_gyro_rate_hz"></a>gyro_rate_hz                           | `float32`    |              |            |
| <a id="fld_accel_raw_rate_hz"></a>accel_raw_rate_hz                 | `float32`    |              |            | full raw sensor sample rate (Hz)                                          |
| <a id="fld_gyro_raw_rate_hz"></a>gyro_raw_rate_hz                   | `float32`    |              |            | full raw sensor sample rate (Hz)                                          |
| <a id="fld_accel_vibration_metric"></a>accel_vibration_metric       | `float32`    |              |            | high frequency vibration level in the accelerometer data (m/s/s)          |
| <a id="fld_gyro_vibration_metric"></a>gyro_vibration_metric         | `float32`    |              |            | high frequency vibration level in the gyro data (rad/s)                   |
| <a id="fld_delta_angle_coning_metric"></a>delta_angle_coning_metric | `float32`    |              |            | average IMU delta angle coning correction (rad^2)                         |
| <a id="fld_mean_accel"></a>mean_accel                               | `float32[3]` |              |            | average accelerometer readings since last publication                     |
| <a id="fld_mean_gyro"></a>mean_gyro                                 | `float32[3]` |              |            | average gyroscope readings since last publication                         |
| <a id="fld_var_accel"></a>var_accel                                 | `float32[3]` |              |            | accelerometer variance since last publication                             |
| <a id="fld_var_gyro"></a>var_gyro                                   | `float32[3]` |              |            | gyroscope variance since last publication                                 |
| <a id="fld_temperature_accel"></a>temperature_accel                 | `float32`    |              |            |
| <a id="fld_temperature_gyro"></a>temperature_gyro                   | `float32`    |              |            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleImuStatus.msg)

::: details Click here to see original file

```c
uint64 timestamp                # time since system start (microseconds)

uint32 accel_device_id          # unique device ID for the sensor that does not change between power cycles
uint32 gyro_device_id           # unique device ID for the sensor that does not change between power cycles

uint32[3] accel_clipping        # total clipping per axis
uint32[3] gyro_clipping         # total clipping per axis

uint32 accel_error_count
uint32 gyro_error_count

float32 accel_rate_hz
float32 gyro_rate_hz

float32 accel_raw_rate_hz       # full raw sensor sample rate (Hz)
float32 gyro_raw_rate_hz        # full raw sensor sample rate (Hz)

float32 accel_vibration_metric  # high frequency vibration level in the accelerometer data (m/s/s)
float32 gyro_vibration_metric   # high frequency vibration level in the gyro data (rad/s)
float32 delta_angle_coning_metric # average IMU delta angle coning correction (rad^2)

float32[3] mean_accel           # average accelerometer readings since last publication
float32[3] mean_gyro            # average gyroscope readings since last publication
float32[3] var_accel            # accelerometer variance since last publication
float32[3] var_gyro             # gyroscope variance since last publication

float32 temperature_accel
float32 temperature_gyro
```

:::
