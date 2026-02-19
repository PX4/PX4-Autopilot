---
pageClass: is-wide-page
---

# GimbalDeviceAttitudeStatus (UORB message)

**TOPICS:** gimbal_deviceattitude_status

## Fields

| 명칭                                                              | 형식           | Unit [Frame] | Range/Enum | 설명                                                        |
| --------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                       | `uint64`     |                                                                  |            | time since system start (microseconds) |
| target_system                              | `uint8`      |                                                                  |            |                                                           |
| target_component                           | `uint8`      |                                                                  |            |                                                           |
| device_flags                               | `uint16`     |                                                                  |            |                                                           |
| q                                                               | `float32[4]` |                                                                  |            |                                                           |
| angular_velocity_x    | `float32`    |                                                                  |            |                                                           |
| angular_velocity_y    | `float32`    |                                                                  |            |                                                           |
| angular_velocity_z    | `float32`    |                                                                  |            |                                                           |
| failure_flags                              | `uint32`     |                                                                  |            |                                                           |
| delta_yaw                                  | `float32`    |                                                                  |            |                                                           |
| delta_yaw_velocity    | `float32`    |                                                                  |            |                                                           |
| gimbal_device_id      | `uint8`      |                                                                  |            |                                                           |
| received_from_mavlink | `bool`       |                                                                  |            |                                                           |

## Constants

| 명칭                                                                                                                                                                                           | 형식       | Value | 설명 |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | ----- | -- |
| <a href="#DEVICE_FLAGS_RETRACT"></a> DEVICE_FLAGS_RETRACT                                                                                          | `uint16` | 1     |    |
| <a href="#DEVICE_FLAGS_NEUTRAL"></a> DEVICE_FLAGS_NEUTRAL                                                                                          | `uint16` | 2     |    |
| <a href="#DEVICE_FLAGS_ROLL_LOCK"></a> DEVICE_FLAGS_ROLL_LOCK                                                                 | `uint16` | 4     |    |
| <a href="#DEVICE_FLAGS_PITCH_LOCK"></a> DEVICE_FLAGS_PITCH_LOCK                                                               | `uint16` | 8     |    |
| <a href="#DEVICE_FLAGS_YAW_LOCK"></a> DEVICE_FLAGS_YAW_LOCK                                                                   | `uint16` | 16    |    |
| <a href="#DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME"></a> DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME | `uint16` | 32    |    |
| <a href="#DEVICE_FLAGS_YAW_IN_EARTH_FRAME"></a> DEVICE_FLAGS_YAW_IN_EARTH_FRAME     | `uint16` | 64    |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalDeviceAttitudeStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp						# time since system start (microseconds)

uint8 target_system
uint8 target_component
uint16 device_flags

uint16 DEVICE_FLAGS_RETRACT = 1
uint16 DEVICE_FLAGS_NEUTRAL = 2
uint16 DEVICE_FLAGS_ROLL_LOCK = 4
uint16 DEVICE_FLAGS_PITCH_LOCK = 8
uint16 DEVICE_FLAGS_YAW_LOCK = 16
uint16 DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME = 32
uint16 DEVICE_FLAGS_YAW_IN_EARTH_FRAME = 64


float32[4] q
float32 angular_velocity_x
float32 angular_velocity_y
float32 angular_velocity_z

uint32 failure_flags
float32 delta_yaw
float32 delta_yaw_velocity
uint8 gimbal_device_id

bool received_from_mavlink
```

:::
