---
pageClass: is-wide-page
---

# VehicleOpticalFlow (UORB message)

Optical flow in XYZ body frame in SI units.

**TOPICS:** vehicle_optical_flow

## Fields

| Name                                                            | Type         | Unit [Frame] | Range/Enum | Description                                                                                                                                    |
| --------------------------------------------------------------- | ------------ | ------------ | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                             | `uint64`     |              |            | time since system start (microseconds)                                                                                                         |
| <a id="fld_timestamp_sample"></a>timestamp_sample               | `uint64`     |              |            |
| <a id="fld_device_id"></a>device_id                             | `uint32`     |              |            | unique device ID for the sensor that does not change between power cycles                                                                      |
| <a id="fld_pixel_flow"></a>pixel_flow                           | `float32[2]` |              |            | (radians) accumulated optical flow in radians where a positive value is produced by a RH rotation about the body axis                          |
| <a id="fld_delta_angle"></a>delta_angle                         | `float32[3]` |              |            | (radians) accumulated gyro radians where a positive value is produced by a RH rotation of the sensor about the body axis. (NAN if unavailable) |
| <a id="fld_distance_m"></a>distance_m                           | `float32`    |              |            | (meters) Distance to the center of the flow field (NAN if unavailable)                                                                         |
| <a id="fld_integration_timespan_us"></a>integration_timespan_us | `uint32`     |              |            | (microseconds) accumulation timespan in microseconds                                                                                           |
| <a id="fld_quality"></a>quality                                 | `uint8`      |              |            | Average of quality of accumulated frames, 0: bad quality, 255: maximum quality                                                                 |
| <a id="fld_max_flow_rate"></a>max_flow_rate                     | `float32`    |              |            | (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably                                                    |
| <a id="fld_min_ground_distance"></a>min_ground_distance         | `float32`    |              |            | (meters) Minimum distance from ground at which the optical flow sensor operates reliably                                                       |
| <a id="fld_max_ground_distance"></a>max_ground_distance         | `float32`    |              |            | (meters) Maximum distance from ground at which the optical flow sensor operates reliably                                                       |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleOpticalFlow.msg)

::: details Click here to see original file

```c
# Optical flow in XYZ body frame in SI units.

uint64 timestamp               # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id               # unique device ID for the sensor that does not change between power cycles

float32[2] pixel_flow          # (radians) accumulated optical flow in radians where a positive value is produced by a RH rotation about the body axis

float32[3] delta_angle         # (radians) accumulated gyro radians where a positive value is produced by a RH rotation of the sensor about the body axis. (NAN if unavailable)

float32 distance_m             # (meters) Distance to the center of the flow field (NAN if unavailable)

uint32 integration_timespan_us # (microseconds) accumulation timespan in microseconds

uint8 quality                  # Average of quality of accumulated frames, 0: bad quality, 255: maximum quality

float32 max_flow_rate          # (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably

float32 min_ground_distance    # (meters) Minimum distance from ground at which the optical flow sensor operates reliably
float32 max_ground_distance    # (meters) Maximum distance from ground at which the optical flow sensor operates reliably
```

:::
