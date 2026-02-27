---
pageClass: is-wide-page
---

# SensorOpticalFlow (UORB message)

**TOPICS:** sensor_opticalflow

## Fields

| 参数名                                                               | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                                                                       |
| ----------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| timestamp                                                         | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                                                                                |
| timestamp_sample                             | `uint64`     |                                                                  |            |                                                                                                                                                                                                                          |
| device_id                                    | `uint32`     |                                                                  |            | unique device ID for the sensor that does not change between power cycles                                                                                                                                                |
| pixel_flow                                   | `float32[2]` |                                                                  |            | (radians) optical flow in radians where a positive value is produced by a RH rotation of the sensor about the body axis                                                                               |
| delta_angle                                  | `float32[3]` |                                                                  |            | (radians) accumulated gyro radians where a positive value is produced by a RH rotation about the body axis. Set to NaN if flow sensor does not have 3-axis gyro data. |
| delta_angle_available   | `bool`       |                                                                  |            |                                                                                                                                                                                                                          |
| distance_m                                   | `float32`    |                                                                  |            | (meters) Distance to the center of the flow field                                                                                                                                                     |
| distance_available                           | `bool`       |                                                                  |            |                                                                                                                                                                                                                          |
| integration_timespan_us | `uint32`     |                                                                  |            | (microseconds) accumulation timespan in microseconds                                                                                                                                                  |
| quality                                                           | `uint8`      |                                                                  |            | quality, 0: bad quality, 255: maximum quality                                                                                                                                            |
| error_count                                  | `uint32`     |                                                                  |            |                                                                                                                                                                                                                          |
| max_flow_rate           | `float32`    |                                                                  |            | (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably                                                                                                           |
| min_ground_distance     | `float32`    |                                                                  |            | (meters) Minimum distance from ground at which the optical flow sensor operates reliably                                                                                                              |
| max_ground_distance     | `float32`    |                                                                  |            | (meters) Maximum distance from ground at which the optical flow sensor operates reliably                                                                                                              |
| mode                                                              | `uint8`      |                                                                  |            |                                                                                                                                                                                                                          |

## Constants

| 参数名                                                                                               | 类型      | 值 | 描述 |
| ------------------------------------------------------------------------------------------------- | ------- | - | -- |
| <a href="#MODE_UNKNOWN"></a> MODE_UNKNOWN                                    | `uint8` | 0 |    |
| <a href="#MODE_BRIGHT"></a> MODE_BRIGHT                                      | `uint8` | 1 |    |
| <a href="#MODE_LOWLIGHT"></a> MODE_LOWLIGHT                                  | `uint8` | 2 |    |
| <a href="#MODE_SUPER_LOWLIGHT"></a> MODE_SUPER_LOWLIGHT | `uint8` | 3 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorOpticalFlow.msg)

:::details
Click here to see original file

```c
uint64 timestamp               # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id               # unique device ID for the sensor that does not change between power cycles

float32[2] pixel_flow          # (radians) optical flow in radians where a positive value is produced by a RH rotation of the sensor about the body axis

float32[3] delta_angle         # (radians) accumulated gyro radians where a positive value is produced by a RH rotation about the body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
bool delta_angle_available

float32 distance_m             # (meters) Distance to the center of the flow field
bool distance_available

uint32 integration_timespan_us # (microseconds) accumulation timespan in microseconds

uint8 quality                  # quality, 0: bad quality, 255: maximum quality

uint32 error_count

float32 max_flow_rate          # (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably

float32 min_ground_distance    # (meters) Minimum distance from ground at which the optical flow sensor operates reliably
float32 max_ground_distance    # (meters) Maximum distance from ground at which the optical flow sensor operates reliably

uint8 MODE_UNKNOWN        = 0
uint8 MODE_BRIGHT         = 1
uint8 MODE_LOWLIGHT       = 2
uint8 MODE_SUPER_LOWLIGHT = 3

uint8 mode
```

:::
