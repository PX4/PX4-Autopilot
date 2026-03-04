---
pageClass: is-wide-page
---

# GimbalDeviceSetAttitude (UORB message)

**TOPICS:** gimbal_deviceset_attitude

## Fields

| 参数名                                                          | 类型           | Unit [Frame] | Range/Enum | 描述                                                        |
| ------------------------------------------------------------ | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                    | `uint64`     |                                                                  |            | time since system start (microseconds) |
| target_system                           | `uint8`      |                                                                  |            |                                                           |
| target_component                        | `uint8`      |                                                                  |            |                                                           |
| flags                                                        | `uint16`     |                                                                  |            |                                                           |
| q                                                            | `float32[4]` |                                                                  |            |                                                           |
| angular_velocity_x | `float32`    |                                                                  |            |                                                           |
| angular_velocity_y | `float32`    |                                                                  |            |                                                           |
| angular_velocity_z | `float32`    |                                                                  |            |                                                           |

## Constants

| 参数名                                                                                                                                                               | 类型       | 值  | 描述 |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | -- | -- |
| <a href="#GIMBAL_DEVICE_FLAGS_RETRACT"></a> GIMBAL_DEVICE_FLAGS_RETRACT                            | `uint32` | 1  |    |
| <a href="#GIMBAL_DEVICE_FLAGS_NEUTRAL"></a> GIMBAL_DEVICE_FLAGS_NEUTRAL                            | `uint32` | 2  |    |
| <a href="#GIMBAL_DEVICE_FLAGS_ROLL_LOCK"></a> GIMBAL_DEVICE_FLAGS_ROLL_LOCK   | `uint32` | 4  |    |
| <a href="#GIMBAL_DEVICE_FLAGS_PITCH_LOCK"></a> GIMBAL_DEVICE_FLAGS_PITCH_LOCK | `uint32` | 8  |    |
| <a href="#GIMBAL_DEVICE_FLAGS_YAW_LOCK"></a> GIMBAL_DEVICE_FLAGS_YAW_LOCK     | `uint32` | 16 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalDeviceSetAttitude.msg)

:::details
Click here to see original file

```c
uint64 timestamp						# time since system start (microseconds)

uint8 target_system
uint8 target_component

uint16 flags
uint32 GIMBAL_DEVICE_FLAGS_RETRACT = 1
uint32 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2
uint32 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4
uint32 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8
uint32 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16

float32[4] q

float32 angular_velocity_x
float32 angular_velocity_y
float32 angular_velocity_z
```

:::
