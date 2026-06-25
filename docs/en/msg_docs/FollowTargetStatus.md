---
pageClass: is-wide-page
---

# FollowTargetStatus (UORB message)

**TOPICS:** follow_target_status

## Fields

| Name                                                        | Type         | Unit [Frame] | Range/Enum | Description                                                                            |
| ----------------------------------------------------------- | ------------ | ------------ | ---------- | -------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                         | `uint64`     | microseconds |            | time since system start                                                                |
| <a id="fld_tracked_target_course"></a>tracked_target_course | `float32`    | rad          |            | Tracked target course in NED local frame (North is course zero)                        |
| <a id="fld_follow_angle"></a>follow_angle                   | `float32`    | rad          |            | Current follow angle setting                                                           |
| <a id="fld_orbit_angle_setpoint"></a>orbit_angle_setpoint   | `float32`    | rad          |            | Current orbit angle setpoint from the smooth trajectory generator                      |
| <a id="fld_angular_rate_setpoint"></a>angular_rate_setpoint | `float32`    | rad/s        |            | Angular rate commanded from Jerk-limited Orbit Angle trajectory for Orbit Angle        |
| <a id="fld_desired_position_raw"></a>desired_position_raw   | `float32[3]` | m            |            | Raw 'idealistic' desired drone position if a drone could teleport from place to places |
| <a id="fld_in_emergency_ascent"></a>in_emergency_ascent     | `bool`       | bool         |            | True when doing emergency ascent (when distance to ground is below safety altitude)    |
| <a id="fld_gimbal_pitch"></a>gimbal_pitch                   | `float32`    | rad          |            | Gimbal pitch commanded to track target in the center of the frame                      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FollowTargetStatus.msg)

::: details Click here to see original file

```c
uint64 timestamp                  # [microseconds] time since system start

float32 tracked_target_course     # [rad] Tracked target course in NED local frame (North is course zero)
float32 follow_angle              # [rad] Current follow angle setting

float32 orbit_angle_setpoint      # [rad] Current orbit angle setpoint from the smooth trajectory generator
float32 angular_rate_setpoint     # [rad/s] Angular rate commanded from Jerk-limited Orbit Angle trajectory for Orbit Angle

float32[3] desired_position_raw   # [m] Raw 'idealistic' desired drone position if a drone could teleport from place to places

bool in_emergency_ascent          # [bool] True when doing emergency ascent (when distance to ground is below safety altitude)
float32 gimbal_pitch              # [rad] Gimbal pitch commanded to track target in the center of the frame
```

:::
