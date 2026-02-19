---
pageClass: is-wide-page
---

# GimbalManagerInformation (повідомлення UORB)

**TOPICS:** gimbal_managerinformation

## Fields

| Назва                                                      | Тип       | Unit [Frame] | Range/Enum | Опис                                                      |
| ---------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------- |
| timestamp                                                  | `uint64`  |                                                                  |            | time since system start (microseconds) |
| cap_flags                             | `uint32`  |                                                                  |            |                                                           |
| gimbal_device_id | `uint8`   |                                                                  |            |                                                           |
| roll_min                              | `float32` | rad                                                              |            |                                                           |
| roll_max                              | `float32` | rad                                                              |            |                                                           |
| pitch_min                             | `float32` | rad                                                              |            |                                                           |
| pitch_max                             | `float32` | rad                                                              |            |                                                           |
| yaw_min                               | `float32` | rad                                                              |            |                                                           |
| yaw_max                               | `float32` | rad                                                              |            |                                                           |

## Constants

| Назва                                                                                                                                                                                                                                                                    | Тип      | Значення | Опис |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------- | -------- | ---- |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT                                                                       | `uint32` | 1        |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL                                                                       | `uint32` | 2        |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS                                              | `uint32` | 4        |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW                                          | `uint32` | 8        |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK                                              | `uint32` | 16       |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS                                            | `uint32` | 32       |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW                                        | `uint32` | 64       |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK                                            | `uint32` | 128      |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS                                                | `uint32` | 256      |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW                                            | `uint32` | 512      |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK"></a> GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK                                                | `uint32` | 1024     |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW"></a> GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW                              | `uint32` | 2048     |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL"></a> GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL   | `uint32` | 65536    |      |
| <a href="#GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL"></a> GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL | `uint32` | 131072   |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalManagerInformation.msg)

:::details
Click here to see original file

```c
uint64 timestamp						# time since system start (microseconds)

uint32 cap_flags

uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT = 1
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL = 2
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS = 4
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW = 8
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK = 16
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS = 32
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW = 64
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK = 128
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS = 256
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW = 512
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK = 1024
uint32 GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048
uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536
uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072

uint8 gimbal_device_id

float32 roll_min # [rad]
float32 roll_max # [rad]

float32 pitch_min # [rad]
float32 pitch_max # [rad]

float32 yaw_min # [rad]
float32 yaw_max # [rad]
```

:::
