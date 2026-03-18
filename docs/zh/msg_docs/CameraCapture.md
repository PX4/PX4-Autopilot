---
pageClass: is-wide-page
---

# CameraCapture (UORB message)

**TOPICS:** camera_capture

## Fields

| 参数名                                  | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                                       |
| ------------------------------------ | ------------ | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------- |
| timestamp                            | `uint64`     |                                                                  |            | time since system start (microseconds)                                                |
| timestamp_utc   | `uint64`     |                                                                  |            | Capture time in UTC / GPS time                                                                           |
| seq                                  | `uint32`     |                                                                  |            | Image sequence number                                                                                    |
| lat                                  | `float64`    |                                                                  |            | Latitude in degrees (WGS84)                                                           |
| lon                                  | `float64`    |                                                                  |            | Longitude in degrees (WGS84)                                                          |
| alt                                  | `float32`    |                                                                  |            | Altitude (AMSL)                                                                       |
| ground_distance | `float32`    |                                                                  |            | Altitude above ground (meters)                                                        |
| q                                    | `float32[4]` |                                                                  |            | Attitude of the camera relative to NED earth-fixed frame when using a gimbal, otherwise vehicle attitude |
| result                               | `int8`       |                                                                  |            | 1 for success, 0 for failure, -1 if camera does not provide feedback                                     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CameraCapture.msg)

:::details
Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_utc		# Capture time in UTC / GPS time
uint32 seq					# Image sequence number
float64 lat					# Latitude in degrees (WGS84)
float64 lon					# Longitude in degrees (WGS84)
float32 alt					# Altitude (AMSL)
float32 ground_distance			# Altitude above ground (meters)
float32[4] q					# Attitude of the camera relative to NED earth-fixed frame when using a gimbal, otherwise vehicle attitude
int8 result					# 1 for success, 0 for failure, -1 if camera does not provide feedback
```

:::
