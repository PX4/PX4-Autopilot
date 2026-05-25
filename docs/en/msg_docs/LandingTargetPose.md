---
pageClass: is-wide-page
---

# LandingTargetPose (UORB message)

Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames.

**TOPICS:** landing_target_pose

## Fields

| Name               | Type      | Unit [Frame] | Range/Enum | Description                                                                               |
| ------------------ | --------- | ------------ | ---------- | ----------------------------------------------------------------------------------------- |
| timestamp          | `uint64`  | us           |            | Time since system start                                                                   |
| is_static          | `bool`    |              |            | Flag indicating whether the landing target is static or moving with respect to the ground |
| rel_pos_valid      | `bool`    |              |            | Flag showing whether relative position is valid                                           |
| rel_vel_valid      | `bool`    |              |            | Flag showing whether relative velocity is valid                                           |
| rel_vel_ekf2_valid | `bool`    |              |            | Flag showing whether relative velocity is valid for EKF2 auxiliary velocity aiding        |
| x_rel              | `float32` | m            |            | X/north position of target, relative to vehicle (navigation frame)                        |
| y_rel              | `float32` | m            |            | Y/east position of target, relative to vehicle (navigation frame)                         |
| z_rel              | `float32` | m            |            | Z/down position of target, relative to vehicle (navigation frame)                         |
| vx_rel             | `float32` | m/s          |            | X/north velocity of target, relative to vehicle (navigation frame)                        |
| vy_rel             | `float32` | m/s          |            | Y/east velocity of target, relative to vehicle (navigation frame)                         |
| vz_rel             | `float32` | m/s          |            | Z/down velocity of target, relative to vehicle (navigation frame)                         |
| cov_x_rel          | `float32` | m^2          |            | X/north position variance                                                                 |
| cov_y_rel          | `float32` | m^2          |            | Y/east position variance                                                                  |
| cov_z_rel          | `float32` | m^2          |            | Z/down position variance                                                                  |
| cov_vx_rel         | `float32` | (m/s)^2      |            | X/north velocity variance                                                                 |
| cov_vy_rel         | `float32` | (m/s)^2      |            | Y/east velocity variance                                                                  |
| cov_vz_rel         | `float32` | (m/s)^2      |            | Z/down velocity variance                                                                  |
| abs_pos_valid      | `bool`    |              |            | Flag showing whether absolute position is valid                                           |
| x_abs              | `float32` | m            |            | X/north position of target, relative to origin (navigation frame)                         |
| y_abs              | `float32` | m            |            | Y/east position of target, relative to origin (navigation frame)                          |
| z_abs              | `float32` | m            |            | Z/down position of target, relative to origin (navigation frame)                          |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/LandingTargetPose.msg)

::: details Click here to see original file

```c
# Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

uint64 timestamp # [us] Time since system start

bool is_static # [-] Flag indicating whether the landing target is static or moving with respect to the ground

bool rel_pos_valid # [-] Flag showing whether relative position is valid
bool rel_vel_valid # [-] Flag showing whether relative velocity is valid
bool rel_vel_ekf2_valid # [-] Flag showing whether relative velocity is valid for EKF2 auxiliary velocity aiding

float32 x_rel # [m] X/north position of target, relative to vehicle (navigation frame)
float32 y_rel # [m] Y/east position of target, relative to vehicle (navigation frame)
float32 z_rel # [m] Z/down position of target, relative to vehicle (navigation frame)

float32 vx_rel # [m/s] X/north velocity of target, relative to vehicle (navigation frame)
float32 vy_rel # [m/s] Y/east velocity of target, relative to vehicle (navigation frame)
float32 vz_rel # [m/s] Z/down velocity of target, relative to vehicle (navigation frame)

float32 cov_x_rel # [m^2] X/north position variance
float32 cov_y_rel # [m^2] Y/east position variance
float32 cov_z_rel # [m^2] Z/down position variance

float32 cov_vx_rel # [(m/s)^2] X/north velocity variance
float32 cov_vy_rel # [(m/s)^2] Y/east velocity variance
float32 cov_vz_rel # [(m/s)^2] Z/down velocity variance

bool abs_pos_valid # [-] Flag showing whether absolute position is valid

float32 x_abs # [m] X/north position of target, relative to origin (navigation frame)
float32 y_abs # [m] Y/east position of target, relative to origin (navigation frame)
float32 z_abs # [m] Z/down position of target, relative to origin (navigation frame)
```

:::
