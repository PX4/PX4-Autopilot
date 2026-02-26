---
pageClass: is-wide-page
---

# GimbalDeviceInformation (UORB message)

**TOPICS:** gimbal_deviceinformation

## Fields

| 参数名                                                        | 类型          | Unit [Frame] | Range/Enum | 描述                                                        |
| ---------------------------------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                  | `uint64`    |                                                                  |            | time since system start (microseconds) |
| vendor_name                           | `uint8[32]` |                                                                  |            |                                                           |
| model_name                            | `uint8[32]` |                                                                  |            |                                                           |
| custom_name                           | `uint8[32]` |                                                                  |            |                                                           |
| firmware_version                      | `uint32`    |                                                                  |            |                                                           |
| hardware_version                      | `uint32`    |                                                                  |            |                                                           |
| uid                                                        | `uint64`    |                                                                  |            |                                                           |
| cap_flags                             | `uint16`    |                                                                  |            |                                                           |
| custom_cap_flags | `uint16`    |                                                                  |            |                                                           |
| roll_min                              | `float32`   | rad                                                              |            |                                                           |
| roll_max                              | `float32`   | rad                                                              |            |                                                           |
| pitch_min                             | `float32`   | rad                                                              |            |                                                           |
| pitch_max                             | `float32`   | rad                                                              |            |                                                           |
| yaw_min                               | `float32`   | rad                                                              |            |                                                           |
| yaw_max                               | `float32`   | rad                                                              |            |                                                           |
| gimbal_device_id | `uint8`     |                                                                  |            |                                                           |

## Constants

| 参数名                                                                                                                                                                                                                                       | 类型       | 值    | 描述 |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | ---- | -- |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT                                          | `uint32` | 1    |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL                                          | `uint32` | 2    |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS                 | `uint32` | 4    |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW             | `uint32` | 8    |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK                 | `uint32` | 16   |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS               | `uint32` | 32   |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW           | `uint32` | 64   |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK               | `uint32` | 128  |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS                   | `uint32` | 256  |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW               | `uint32` | 512  |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK                   | `uint32` | 1024 |    |
| <a href="#GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW"></a> GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW | `uint32` | 2048 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalDeviceInformation.msg)

:::details
Click here to see original file

```c
uint64 timestamp						# time since system start (microseconds)

uint8[32] vendor_name
uint8[32] model_name
uint8[32] custom_name
uint32 firmware_version
uint32 hardware_version
uint64 uid

uint16 cap_flags

uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512
uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024
uint32 GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048

uint16 custom_cap_flags

float32 roll_min # [rad]
float32 roll_max # [rad]

float32 pitch_min # [rad]
float32 pitch_max # [rad]

float32 yaw_min # [rad]
float32 yaw_max # [rad]

uint8 gimbal_device_id
```

:::
