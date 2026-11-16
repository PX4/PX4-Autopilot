---
pageClass: is-wide-page
---

# GimbalManagerSetPitchyaw (UORB message)

Gimbal manager pitch/yaw setpoint.

This is sent by PX4 (acting as a gimbal manager client) to an
external gimbal manager). Streamed out as GIMBAL_MANAGER_SET_PITCHYAW.

**TOPICS:** gimbal_manager_set_pitchyaw

## Fields

| Name                                              | Type      | Unit [Frame] | Range/Enum                                    | Description                                 |
| ------------------------------------------------- | --------- | ------------ | --------------------------------------------- | ------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`  | us           |                                               | Time since system start                     |
| <a id="fld_target_system"></a>target_system       | `uint8`   |              |                                               | System id of the external gimbal manager    |
| <a id="fld_target_component"></a>target_component | `uint8`   |              |                                               | Component id of the external gimbal manager |
| <a id="fld_gimbal_device_id"></a>gimbal_device_id | `uint8`   |              |                                               | Gimbal device to address (0 for all)        |
| <a id="fld_flags"></a>flags                       | `uint32`  |              | [GIMBAL_MANAGER_FLAGS](#GIMBAL_MANAGER_FLAGS) | GIMBAL_MANAGER_FLAGS bitmask                |
| <a id="fld_pitch"></a>pitch                       | `float32` | rad          |                                               | pitch angle (Invalid: NaN to ignore)        |
| <a id="fld_yaw"></a>yaw                           | `float32` | rad          |                                               | yaw angle (Invalid: NaN to ignore)          |
| <a id="fld_pitch_rate"></a>pitch_rate             | `float32` | rad/s        |                                               | Pitch angular rate (Invalid: NaN to ignore) |
| <a id="fld_yaw_rate"></a>yaw_rate                 | `float32` | rad/s        |                                               | Yaw angular rate (Invalid: NaN to ignore)   |

## Enums

### GIMBAL_MANAGER_FLAGS {#GIMBAL_MANAGER_FLAGS}

Used in field(s): [flags](#fld_flags)

| Name                                                                                          | Type     | Value | Description |
| --------------------------------------------------------------------------------------------- | -------- | ----- | ----------- |
| <a id="#GIMBAL_MANAGER_FLAGS_PITCH_LOCK"></a> GIMBAL_MANAGER_FLAGS_PITCH_LOCK                 | `uint32` | 8     |
| <a id="#GIMBAL_MANAGER_FLAGS_YAW_LOCK"></a> GIMBAL_MANAGER_FLAGS_YAW_LOCK                     | `uint32` | 16    |
| <a id="#GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME"></a> GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME | `uint32` | 64    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalManagerSetPitchyaw.msg)

::: details Click here to see original file

```c
# Gimbal manager pitch/yaw setpoint
#
# This is sent by PX4 (acting as a gimbal manager client) to an
# external gimbal manager). Streamed out as GIMBAL_MANAGER_SET_PITCHYAW.

uint64 timestamp # [us] Time since system start

uint8 target_system # [-] System id of the external gimbal manager
uint8 target_component	# [-] Component id of the external gimbal manager
uint8 gimbal_device_id # [-] Gimbal device to address (0 for all)

uint32 flags # [@enum GIMBAL_MANAGER_FLAGS] GIMBAL_MANAGER_FLAGS bitmask
uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8
uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16
uint32 GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME = 64

float32 pitch # [rad] [@invalid NaN to ignore] pitch angle
float32 yaw # [rad] [@invalid NaN to ignore] yaw angle
float32 pitch_rate # [rad/s] [@invalid NaN to ignore] Pitch angular rate
float32 yaw_rate # [rad/s] [@invalid NaN to ignore] Yaw angular rate
```

:::
