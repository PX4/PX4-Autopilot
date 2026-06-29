---
pageClass: is-wide-page
---

# GimbalDeviceInformation (UORB message)

**TOPICS:** gimbal_device_information

## Fields

| Name                                              | Type        | Unit [Frame] | Range/Enum | Description                            |
| ------------------------------------------------- | ----------- | ------------ | ---------- | -------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`    |              |            | time since system start (microseconds) |
| <a id="fld_vendor_name"></a>vendor_name           | `uint8[32]` |              |            |
| <a id="fld_model_name"></a>model_name             | `uint8[32]` |              |            |
| <a id="fld_custom_name"></a>custom_name           | `uint8[32]` |              |            |
| <a id="fld_firmware_version"></a>firmware_version | `uint32`    |              |            |
| <a id="fld_hardware_version"></a>hardware_version | `uint32`    |              |            |
| <a id="fld_uid"></a>uid                           | `uint64`    |              |            |
| <a id="fld_cap_flags"></a>cap_flags               | `uint16`    |              |            |
| <a id="fld_custom_cap_flags"></a>custom_cap_flags | `uint16`    |              |            |
| <a id="fld_roll_min"></a>roll_min                 | `float32`   | rad          |            |
| <a id="fld_roll_max"></a>roll_max                 | `float32`   | rad          |            |
| <a id="fld_pitch_min"></a>pitch_min               | `float32`   | rad          |            |
| <a id="fld_pitch_max"></a>pitch_max               | `float32`   | rad          |            |
| <a id="fld_yaw_min"></a>yaw_min                   | `float32`   | rad          |            |
| <a id="fld_yaw_max"></a>yaw_max                   | `float32`   | rad          |            |
| <a id="fld_gimbal_device_id"></a>gimbal_device_id | `uint8`     |              |            |

## Constants

| Name                                                                                                      | Type     | Value | Description |
| --------------------------------------------------------------------------------------------------------- | -------- | ----- | ----------- |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT                     | `uint32` | 1     |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL                     | `uint32` | 2     |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS                 | `uint32` | 4     |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW             | `uint32` | 8     |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK                 | `uint32` | 16    |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS               | `uint32` | 32    |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW           | `uint32` | 64    |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK               | `uint32` | 128   |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS                   | `uint32` | 256   |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW               | `uint32` | 512   |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK"></a> GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK                   | `uint32` | 1024  |
| <a id="#GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW"></a> GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW | `uint32` | 2048  |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GimbalDeviceInformation.msg)

::: details Click here to see original file

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
