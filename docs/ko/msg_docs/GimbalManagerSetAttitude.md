---
pageClass: is-wide-page
---

# GimbalManagerSetAttitude (UORB message)

**TOPICS:** gimbal_managerset_attitude

## Fields

| 명칭                                                           | 형식           | Unit [Frame] | Range/Enum | 설명                                                        |
| ------------------------------------------------------------ | ------------ | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                    | `uint64`     |                                                                  |            | time since system start (microseconds) |
| origin_sysid                            | `uint8`      |                                                                  |            |                                                           |
| origin_compid                           | `uint8`      |                                                                  |            |                                                           |
| target_system                           | `uint8`      |                                                                  |            |                                                           |
| target_component                        | `uint8`      |                                                                  |            |                                                           |
| flags                                                        | `uint32`     |                                                                  |            |                                                           |
| gimbal_device_id   | `uint8`      |                                                                  |            |                                                           |
| q                                                            | `float32[4]` |                                                                  |            |                                                           |
| angular_velocity_x | `float32`    |                                                                  |            |                                                           |
| angular_velocity_y | `float32`    |                                                                  |            |                                                           |
| angular_velocity_z | `float32`    |                                                                  |            |                                                           |

## Constants

| 명칭                                                                                                                                                                  | 형식       | Value | 설명 |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | ----- | -- |
| <a href="#GIMBAL_MANAGER_FLAGS_RETRACT"></a> GIMBAL_MANAGER_FLAGS_RETRACT                            | `uint32` | 1     |    |
| <a href="#GIMBAL_MANAGER_FLAGS_NEUTRAL"></a> GIMBAL_MANAGER_FLAGS_NEUTRAL                            | `uint32` | 2     |    |
| <a href="#GIMBAL_MANAGER_FLAGS_ROLL_LOCK"></a> GIMBAL_MANAGER_FLAGS_ROLL_LOCK   | `uint32` | 4     |    |
| <a href="#GIMBAL_MANAGER_FLAGS_PITCH_LOCK"></a> GIMBAL_MANAGER_FLAGS_PITCH_LOCK | `uint32` | 8     |    |
| <a href="#GIMBAL_MANAGER_FLAGS_YAW_LOCK"></a> GIMBAL_MANAGER_FLAGS_YAW_LOCK     | `uint32` | 16    |    |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                                                         | `uint8`  | 2     |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalManagerSetAttitude.msg)

:::details
Click here to see original file

```c
uint64 timestamp						# time since system start (microseconds)

uint8 origin_sysid
uint8 origin_compid

uint8 target_system
uint8 target_component

uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1
uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2
uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4
uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8
uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16

uint32 flags
uint8 gimbal_device_id

float32[4] q

float32 angular_velocity_x
float32 angular_velocity_y
float32 angular_velocity_z

uint8 ORB_QUEUE_LENGTH = 2
```

:::
