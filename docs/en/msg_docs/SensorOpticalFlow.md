---
pageClass: is-wide-page
---

# SensorOpticalFlow (UORB message)

**TOPICS:** sensor_optical_flow

## Fields

| Name                                                            | Type         | Unit [Frame] | Range/Enum | Description                                                                                                                                                           |
| --------------------------------------------------------------- | ------------ | ------------ | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                             | `uint64`     |              |            | time since system start (microseconds)                                                                                                                                |
| <a id="fld_timestamp_sample"></a>timestamp_sample               | `uint64`     |              |            |
| <a id="fld_device_id"></a>device_id                             | `uint32`     |              |            | unique device ID for the sensor that does not change between power cycles                                                                                             |
| <a id="fld_pixel_flow"></a>pixel_flow                           | `float32[2]` |              |            | (radians) optical flow in radians where a positive value is produced by a RH rotation of the sensor about the body axis                                               |
| <a id="fld_delta_angle"></a>delta_angle                         | `float32[3]` |              |            | (radians) accumulated gyro radians where a positive value is produced by a RH rotation about the body axis. Set to NaN if flow sensor does not have 3-axis gyro data. |
| <a id="fld_delta_angle_available"></a>delta_angle_available     | `bool`       |              |            |
| <a id="fld_distance_m"></a>distance_m                           | `float32`    |              |            | (meters) Distance to the center of the flow field                                                                                                                     |
| <a id="fld_distance_available"></a>distance_available           | `bool`       |              |            |
| <a id="fld_integration_timespan_us"></a>integration_timespan_us | `uint32`     |              |            | (microseconds) accumulation timespan in microseconds                                                                                                                  |
| <a id="fld_quality"></a>quality                                 | `uint8`      |              |            | quality, 0: bad quality, 255: maximum quality                                                                                                                         |
| <a id="fld_error_count"></a>error_count                         | `uint32`     |              |            |
| <a id="fld_max_flow_rate"></a>max_flow_rate                     | `float32`    |              |            | (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably                                                                           |
| <a id="fld_min_ground_distance"></a>min_ground_distance         | `float32`    |              |            | (meters) Minimum distance from ground at which the optical flow sensor operates reliably                                                                              |
| <a id="fld_max_ground_distance"></a>max_ground_distance         | `float32`    |              |            | (meters) Maximum distance from ground at which the optical flow sensor operates reliably                                                                              |
| <a id="fld_mode"></a>mode                                       | `uint8`      |              |            |

## Constants

| Name                                                  | Type    | Value | Description |
| ----------------------------------------------------- | ------- | ----- | ----------- |
| <a id="#MODE_UNKNOWN"></a> MODE_UNKNOWN               | `uint8` | 0     |
| <a id="#MODE_BRIGHT"></a> MODE_BRIGHT                 | `uint8` | 1     |
| <a id="#MODE_LOWLIGHT"></a> MODE_LOWLIGHT             | `uint8` | 2     |
| <a id="#MODE_SUPER_LOWLIGHT"></a> MODE_SUPER_LOWLIGHT | `uint8` | 3     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorOpticalFlow.msg)

::: details Click here to see original file

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
