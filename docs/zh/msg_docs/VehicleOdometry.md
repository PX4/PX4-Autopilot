---
pageClass: is-wide-page
---

# VehicleOdometry (UORB message)

Vehicle odometry data.

Fits ROS REP 147 for aerial vehicles

**TOPICS:** vehicle_odometry vehicle_mocap_odometry vehicle_visual_odometry estimator_odometry

## Fields

| 参数名                                       | 类型           | Unit [Frame]                                                                                                 | Range/Enum                                             | 描述                                                                                                                                                                                                                                                                                                                                    |
| ----------------------------------------- | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                 | `uint64`     | us                                                                                                                                                               |                                                        | Time since system start                                                                                                                                                                                                                                                                                                               |
| timestamp_sample     | `uint64`     | us                                                                                                                                                               |                                                        | Timestamp sample                                                                                                                                                                                                                                                                                                                      |
| pose_frame           | `uint8`      |                                                                                                                                                                  | [POSE_FRAME](#POSE_FRAME)         | Position and orientation frame of reference                                                                                                                                                                                                                                                                                           |
| 位置                                        | `float32[3]` | m [local frame]                                                                                              |                                                        | Position. Origin is position of GC at startup. (Invalid: NaN If invalid/unknown)                                                                                                                                                                                   |
| q                                         | `float32[4]` |                                                                                                                                                                  |                                                        | Attitude (expressed as a quaternion) relative to pose reference frame at current location. Follows the Hamiltonian convention (w, x, y, z, right-handed, passive rotations from body to world) (Invalid: NaN First value if invalid/unknown) |
| velocity_frame       | `uint8`      |                                                                                                                                                                  | [VELOCITY_FRAME](#VELOCITY_FRAME) | Reference frame of the velocity data                                                                                                                                                                                                                                                                                                  |
| 速度                                        | `float32[3]` | m/s [@velocity_frame]                                                      |                                                        | Velocity. (Invalid: NaN If invalid/unknown)                                                                                                                                                                                                                                        |
| angular_velocity     | `float32[3]` | rad/s [@VELOCITY_FRAME_BODY_FRD] |                                                        | Angular velocity in body-fixed frame (Invalid: NaN If invalid/unknown)                                                                                                                                                                                                                             |
| position_variance    | `float32[3]` | m^2                                                                                                                                                              |                                                        | Variance of position error                                                                                                                                                                                                                                                                                                            |
| orientation_variance | `float32[3]` | rad^2                                                                                                                                                            |                                                        | Variance of orientation/attitude error (expressed in body frame)                                                                                                                                                                                                                                                   |
| velocity_variance    | `float32[3]` | m^2/s^2                                                                                                                                                          |                                                        | Variance of velocity error                                                                                                                                                                                                                                                                                                            |
| reset_counter        | `uint8`      |                                                                                                                                                                  |                                                        | Reset counter. Counts reset events on attitude, velocity and position.                                                                                                                                                                                                                                |
| quality                                   | `int8`       |                                                                                                                                                                  |                                                        | Quality. Unused. (Invalid: 0)                                                                                                                                                                                                                                      |

## Enums

### POSE_FRAME {#POSE_FRAME}

| 参数名                                                                                             | 类型      | 值 | 描述                                                                                                                                                               |
| ----------------------------------------------------------------------------------------------- | ------- | - | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#POSE_FRAME_UNKNOWN"></a> POSE_FRAME_UNKNOWN | `uint8` | 0 | Unknown frame                                                                                                                                                    |
| <a href="#POSE_FRAME_NED"></a> POSE_FRAME_NED         | `uint8` | 1 | North-East-Down (NED) navigation frame. Aligned with True North.                                              |
| <a href="#POSE_FRAME_FRD"></a> POSE_FRAME_FRD         | `uint8` | 2 | Forward-Right-Down (FRD) frame. Constant arbitrary heading offset from True North. Z is down. |

### VELOCITY_FRAME {#VELOCITY_FRAME}

| 参数名                                                                                                                            | 类型      | 值 | 描述                                                                                                                                                      |
| ------------------------------------------------------------------------------------------------------------------------------ | ------- | - | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#VELOCITY_FRAME_UNKNOWN"></a> VELOCITY_FRAME_UNKNOWN                        | `uint8` | 0 | Unknown frame                                                                                                                                           |
| <a href="#VELOCITY_FRAME_NED"></a> VELOCITY_FRAME_NED                                | `uint8` | 1 | NED navigation frame at current position.                                                                                               |
| <a href="#VELOCITY_FRAME_FRD"></a> VELOCITY_FRAME_FRD                                | `uint8` | 2 | FRD navigation frame at current position. Constant arbitrary heading offset from True North. Z is down. |
| <a href="#VELOCITY_FRAME_BODY_FRD"></a> VELOCITY_FRAME_BODY_FRD | `uint8` | 3 | FRD body-fixed frame                                                                                                                                    |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述 |
| -------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleOdometry.msg)

:::details
Click here to see original file

```c
# Vehicle odometry data
#
# Fits ROS REP 147 for aerial vehicles

uint32 MESSAGE_VERSION = 0

uint64 timestamp         # [us] Time since system start
uint64 timestamp_sample  # [us] Timestamp sample

uint8 pose_frame              # [@enum POSE_FRAME] Position and orientation frame of reference
uint8 POSE_FRAME_UNKNOWN = 0  # Unknown frame
uint8 POSE_FRAME_NED     = 1  # North-East-Down (NED) navigation frame. Aligned with True North.
uint8 POSE_FRAME_FRD     = 2  # Forward-Right-Down (FRD) frame. Constant arbitrary heading offset from True North. Z is down.

float32[3] position  # [m] [@frame local frame] [@invalid NaN If invalid/unknown] Position. Origin is position of GC at startup.
float32[4] q         # [-] [@invalid NaN First value if invalid/unknown] Attitude (expressed as a quaternion) relative to pose reference frame at current location. Follows the Hamiltonian convention (w, x, y, z, right-handed, passive rotations from body to world)

uint8 velocity_frame               # [@enum VELOCITY_FRAME] Reference frame of the velocity data
uint8 VELOCITY_FRAME_UNKNOWN  = 0  # Unknown frame
uint8 VELOCITY_FRAME_NED      = 1  # NED navigation frame at current position.
uint8 VELOCITY_FRAME_FRD      = 2  # FRD navigation frame at current position. Constant arbitrary heading offset from True North. Z is down.
uint8 VELOCITY_FRAME_BODY_FRD = 3  # FRD body-fixed frame

float32[3] velocity          # [m/s] [@frame @velocity_frame] [@invalid NaN If invalid/unknown] Velocity.
float32[3] angular_velocity  # [rad/s] [@frame @VELOCITY_FRAME_BODY_FRD] [@invalid NaN If invalid/unknown] Angular velocity in body-fixed frame

float32[3] position_variance     # [m^2] Variance of position error
float32[3] orientation_variance  # [rad^2] Variance of orientation/attitude error (expressed in body frame)
float32[3] velocity_variance     # [m^2/s^2] Variance of velocity error

uint8 reset_counter  # [-] Reset counter. Counts reset events on attitude, velocity and position.
int8 quality         # [-] [@invalid 0] Quality. Unused.

# TOPICS vehicle_odometry vehicle_mocap_odometry vehicle_visual_odometry
# TOPICS estimator_odometry
```

:::
