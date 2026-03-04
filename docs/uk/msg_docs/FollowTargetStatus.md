---
pageClass: is-wide-page
---

# FollowTargetStatus (повідомлення UORB)

**TOPICS:** follow_targetstatus

## Fields

| Назва                                                           | Тип          | Unit [Frame] | Range/Enum | Опис                                                                                                   |
| --------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------ |
| timestamp                                                       | `uint64`     | microseconds                                                     |            | time since system start                                                                                |
| tracked_target_course | `float32`    | rad                                                              |            | Tracked target course in NED local frame (North is course zero)                     |
| follow_angle                               | `float32`    | rad                                                              |            | Current follow angle setting                                                                           |
| orbit_angle_setpoint  | `float32`    | rad                                                              |            | Current orbit angle setpoint from the smooth trajectory generator                                      |
| angular_rate_setpoint | `float32`    | rad/s                                                            |            | Angular rate commanded from Jerk-limited Orbit Angle trajectory for Orbit Angle                        |
| desired_position_raw  | `float32[3]` | m                                                                |            | Raw 'idealistic' desired drone position if a drone could teleport from place to places                 |
| in_emergency_ascent   | `bool`       | bool                                                             |            | True when doing emergency ascent (when distance to ground is below safety altitude) |
| gimbal_pitch                               | `float32`    | rad                                                              |            | Gimbal pitch commanded to track target in the center of the frame                                      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FollowTargetStatus.msg)

:::details
Click here to see original file

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
