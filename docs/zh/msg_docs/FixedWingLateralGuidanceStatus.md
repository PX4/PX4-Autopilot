---
pageClass: is-wide-page
---

# FixedWingLateralGuidanceStatus (UORB message)

Fixed Wing Lateral Guidance Status message. Published by fw_pos_control module to report the resultant lateral setpoints and NPFG debug outputs.

**TOPICS:** fixed_wing_lateral_guidance_status

## Fields

| 参数名                                                                                                                        | 类型        | Unit [Frame] | Range/Enum                                                                     | 描述                                                                                                                                                                                                                                                                                                                      |
| -------------------------------------------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                                                        | `uint64`  |                                                                  |                                                                                | time since system start (microseconds)                                                                                                                                                                                                                                                               |
| <a id="fld_course_setpoint"></a>course_setpoint                                                       | `float32` | rad                                                              | [-pi : pi] | Desired direction of travel over ground w.r.t (true) North. Set by guidance law                                                                                                                                                                      |
| <a id="fld_lateral_acceleration_ff"></a>lateral_acceleration_ff                  | `float32` | FRD                                                              |                                                                                | lateral acceleration demand only for maintaining curvature                                                                                                                                                                                                                                                              |
| <a id="fld_bearing_feas"></a>bearing_feas                                                             | `float32` |                                                                  | [0 : 1]    | bearing feasibility                                                                                                                                                                                                                                                                                                     |
| <a id="fld_bearing_feas_on_track"></a>bearing_feas_on_track | `float32` |                                                                  | [0 : 1]    | on-track bearing feasibility                                                                                                                                                                                                                                                                                            |
| <a id="fld_signed_track_error"></a>signed_track_error                            | `float32` | 米                                                                |                                                                                | signed track error                                                                                                                                                                                                                                                                                                      |
| <a id="fld_track_error_bound"></a>track_error_bound                              | `float32` | 米                                                                |                                                                                | track error bound                                                                                                                                                                                                                                                                                                       |
| <a id="fld_switch_distance"></a>switch_distance                                                       | `float32` | 米                                                                | [0 : INF]  | distance from the current waypoint at which the navigator should advance to the next one (turn anticipation). If below NAV_ACC_RAD, the parameter value is used instead. (Invalid: NaN) |
| <a id="fld_adapted_period"></a>adapted_period                                                         | `float32` | s                                                                |                                                                                | adapted period (if auto-tuning enabled)                                                                                                                                                                                                                                                              |
| <a id="fld_wind_est_valid"></a>wind_est_valid                                    | `uint8`   | boolean                                                          |                                                                                | true = wind estimate is valid and/or being used by controller (also indicates if wind estimate usage is disabled despite being valid)                                                                                                                                                                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FixedWingLateralGuidanceStatus.msg)

:::details
Click here to see original file

```c
# Fixed Wing Lateral Guidance Status message
# Published by fw_pos_control module to report the resultant lateral setpoints and NPFG debug outputs

uint64 timestamp                # time since system start (microseconds)

float32 course_setpoint         # [rad] [@range -pi, pi] Desired direction of travel over ground w.r.t (true) North. Set by guidance law
float32 lateral_acceleration_ff # [m/s^2] [FRD] lateral acceleration demand only for maintaining curvature
float32 bearing_feas            # [@range 0,1] bearing feasibility
float32 bearing_feas_on_track   # [@range 0,1] on-track bearing feasibility
float32 signed_track_error      # [m] signed track error
float32 track_error_bound       # [m] track error bound
float32 switch_distance         # [m] [@range 0, INF] [@invalid NaN] distance from the current waypoint at which the navigator should advance to the next one (turn anticipation). If below NAV_ACC_RAD, the parameter value is used instead.
float32 adapted_period          # [s] adapted period (if auto-tuning enabled)
uint8 wind_est_valid            # [boolean] true = wind estimate is valid and/or being used by controller (also indicates if wind estimate usage is disabled despite being valid)
```

:::
