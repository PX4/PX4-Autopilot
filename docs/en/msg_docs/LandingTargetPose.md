---
pageClass: is-wide-page
---

# LandingTargetPose (UORB message)

Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames.

**TOPICS:** landing_target_pose

## Fields

| Name                                                  | Type      | Unit [Frame] | Range/Enum | Description                                                                               |
| ----------------------------------------------------- | --------- | ------------ | ---------- | ----------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                   | `uint64`  | us           |            | Time since system start                                                                   |
| <a id="fld_is_static"></a>is_static                   | `bool`    |              |            | Flag indicating whether the landing target is static or moving with respect to the ground |
| <a id="fld_rel_pos_valid"></a>rel_pos_valid           | `bool`    |              |            | Flag showing whether relative position is valid                                           |
| <a id="fld_rel_vel_valid"></a>rel_vel_valid           | `bool`    |              |            | Flag showing whether relative velocity is valid                                           |
| <a id="fld_rel_vel_ekf2_valid"></a>rel_vel_ekf2_valid | `bool`    |              |            | Flag showing whether relative velocity is valid for EKF2 auxiliary velocity aiding        |
| <a id="fld_x_rel"></a>x_rel                           | `float32` | m            |            | X/north position of target, relative to vehicle (navigation frame)                        |
| <a id="fld_y_rel"></a>y_rel                           | `float32` | m            |            | Y/east position of target, relative to vehicle (navigation frame)                         |
| <a id="fld_z_rel"></a>z_rel                           | `float32` | m            |            | Z/down position of target, relative to vehicle (navigation frame)                         |
| <a id="fld_vx_rel"></a>vx_rel                         | `float32` | m/s          |            | X/north velocity of target, relative to vehicle (navigation frame)                        |
| <a id="fld_vy_rel"></a>vy_rel                         | `float32` | m/s          |            | Y/east velocity of target, relative to vehicle (navigation frame)                         |
| <a id="fld_vz_rel"></a>vz_rel                         | `float32` | m/s          |            | Z/down velocity of target, relative to vehicle (navigation frame)                         |
| <a id="fld_cov_x_rel"></a>cov_x_rel                   | `float32` | m^2          |            | X/north position variance                                                                 |
| <a id="fld_cov_y_rel"></a>cov_y_rel                   | `float32` | m^2          |            | Y/east position variance                                                                  |
| <a id="fld_cov_z_rel"></a>cov_z_rel                   | `float32` | m^2          |            | Z/down position variance                                                                  |
| <a id="fld_cov_vx_rel"></a>cov_vx_rel                 | `float32` | (m/s)^2      |            | X/north velocity variance                                                                 |
| <a id="fld_cov_vy_rel"></a>cov_vy_rel                 | `float32` | (m/s)^2      |            | Y/east velocity variance                                                                  |
| <a id="fld_cov_vz_rel"></a>cov_vz_rel                 | `float32` | (m/s)^2      |            | Z/down velocity variance                                                                  |
| <a id="fld_abs_pos_valid"></a>abs_pos_valid           | `bool`    |              |            | Flag showing whether absolute position is valid                                           |
| <a id="fld_x_abs"></a>x_abs                           | `float32` | m            |            | X/north position of target, relative to origin (navigation frame)                         |
| <a id="fld_y_abs"></a>y_abs                           | `float32` | m            |            | Y/east position of target, relative to origin (navigation frame)                          |
| <a id="fld_z_abs"></a>z_abs                           | `float32` | m            |            | Z/down position of target, relative to origin (navigation frame)                          |

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
