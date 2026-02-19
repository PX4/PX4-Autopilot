---
pageClass: is-wide-page
---

# VehicleImuStatus (повідомлення UORB)

**TOPICS:** vehicle_imustatus

## Fields

| Назва                                                                                    | Тип          | Unit [Frame] | Range/Enum | Опис                                                                                |
| ---------------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------- |
| timestamp                                                                                | `uint64`     |                                                                  |            | time since system start (microseconds)                           |
| accel_device_id                                | `uint32`     |                                                                  |            | unique device ID for the sensor that does not change between power cycles           |
| gyro_device_id                                 | `uint32`     |                                                                  |            | unique device ID for the sensor that does not change between power cycles           |
| accel_clipping                                                      | `uint32[3]`  |                                                                  |            | total clipping per axis                                                             |
| gyro_clipping                                                       | `uint32[3]`  |                                                                  |            | total clipping per axis                                                             |
| accel_error_count                              | `uint32`     |                                                                  |            |                                                                                     |
| gyro_error_count                               | `uint32`     |                                                                  |            |                                                                                     |
| accel_rate_hz                                  | `float32`    |                                                                  |            |                                                                                     |
| gyro_rate_hz                                   | `float32`    |                                                                  |            |                                                                                     |
| accel_raw_rate_hz         | `float32`    |                                                                  |            | full raw sensor sample rate (Hz)                                 |
| gyro_raw_rate_hz          | `float32`    |                                                                  |            | full raw sensor sample rate (Hz)                                 |
| accel_vibration_metric                         | `float32`    |                                                                  |            | high frequency vibration level in the accelerometer data (m/s/s) |
| gyro_vibration_metric                          | `float32`    |                                                                  |            | high frequency vibration level in the gyro data (rad/s)          |
| delta_angle_coning_metric | `float32`    |                                                                  |            | average IMU delta angle coning correction (rad^2)                |
| mean_accel                                                          | `float32[3]` |                                                                  |            | average accelerometer readings since last publication                               |
| mean_gyro                                                           | `float32[3]` |                                                                  |            | average gyroscope readings since last publication                                   |
| var_accel                                                           | `float32[3]` |                                                                  |            | accelerometer variance since last publication                                       |
| var_gyro                                                            | `float32[3]` |                                                                  |            | gyroscope variance since last publication                                           |
| temperature_accel                                                   | `float32`    |                                                                  |            |                                                                                     |
| temperature_gyro                                                    | `float32`    |                                                                  |            |                                                                                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleImuStatus.msg)

:::details
Click here to see original file

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
