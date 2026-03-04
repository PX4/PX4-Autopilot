---
pageClass: is-wide-page
---

# GimbalManagerSetManualControl (UORB message)

**TOPICS:** gimbal_managerset_manualcontrol

## Fields

| 参数名                                                        | 类型        | Unit [Frame] | Range/Enum | 描述                                                         |
| ---------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------- |
| timestamp                                                  | `uint64`  |                                                                  |            | time since system start (microseconds)  |
| origin_sysid                          | `uint8`   |                                                                  |            |                                                            |
| origin_compid                         | `uint8`   |                                                                  |            |                                                            |
| target_system                         | `uint8`   |                                                                  |            |                                                            |
| target_component                      | `uint8`   |                                                                  |            |                                                            |
| flags                                                      | `uint32`  |                                                                  |            |                                                            |
| gimbal_device_id | `uint8`   |                                                                  |            |                                                            |
| pitch                                                      | `float32` |                                                                  |            | unitless -1..1, can be NAN |
| yaw                                                        | `float32` |                                                                  |            | unitless -1..1, can be NAN |
| pitch_rate                            | `float32` |                                                                  |            | unitless -1..1, can be NAN |
| yaw_rate                              | `float32` |                                                                  |            | unitless -1..1, can be NAN |

## Constants

| 参数名                                                                                                                                                                 | 类型       | 值  | 描述 |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | -- | -- |
| <a href="#GIMBAL_MANAGER_FLAGS_RETRACT"></a> GIMBAL_MANAGER_FLAGS_RETRACT                            | `uint32` | 1  |    |
| <a href="#GIMBAL_MANAGER_FLAGS_NEUTRAL"></a> GIMBAL_MANAGER_FLAGS_NEUTRAL                            | `uint32` | 2  |    |
| <a href="#GIMBAL_MANAGER_FLAGS_ROLL_LOCK"></a> GIMBAL_MANAGER_FLAGS_ROLL_LOCK   | `uint32` | 4  |    |
| <a href="#GIMBAL_MANAGER_FLAGS_PITCH_LOCK"></a> GIMBAL_MANAGER_FLAGS_PITCH_LOCK | `uint32` | 8  |    |
| <a href="#GIMBAL_MANAGER_FLAGS_YAW_LOCK"></a> GIMBAL_MANAGER_FLAGS_YAW_LOCK     | `uint32` | 16 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalManagerSetManualControl.msg)

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

float32 pitch      # unitless -1..1, can be NAN
float32 yaw        # unitless -1..1, can be NAN
float32 pitch_rate # unitless -1..1, can be NAN
float32 yaw_rate   # unitless -1..1, can be NAN
```

:::
