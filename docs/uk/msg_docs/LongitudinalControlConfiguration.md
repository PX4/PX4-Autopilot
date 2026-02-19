---
pageClass: is-wide-page
---

# LongitudinalControlConfiguration (UORB message)

Fixed Wing Longitudinal Control Configuration message. Used by the fw_lateral_longitudinal_control module and TECS to constrain FixedWingLongitudinalSetpoint messages. and configure the resultant setpoints.

**TOPICS:** longitudinal_controlconfiguration

## Fields

| Назва                                                                                       | Тип       | Unit [Frame] | Range/Enum                                                                     | Опис                                                                                                                                                                                                                                                                                  |
| ------------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                   | `uint64`  |                                                                  |                                                                                | time since system start (microseconds)                                                                                                                                                                                                                             |
| pitch_min                                                              | `float32` | rad                                                              | [-pi : pi] | defaults to FW_P_LIM_MIN if NAN.                                                                                                                                                                       |
| pitch_max                                                              | `float32` | rad                                                              | [-pi : pi] | defaults to FW_P_LIM_MAX if NAN.                                                                                                                                                                       |
| throttle_min                                                           | `float32` | norm                                                             | [0 : 1]    | deaults to FW_THR_MIN if NAN.                                                                                                                                                                                               |
| throttle_max                                                           | `float32` | norm                                                             | [0 : 1]    | defaults to FW_THR_MAX if NAN.                                                                                                                                                                                              |
| climb_rate_target                                 | `float32` | m/s                                                              |                                                                                | target climbrate to change altitude. Defaults to FW_T_CLIMB_MAX if NAN. Not used if height_rate is directly set in FixedWingLongitudinalSetpoint. |
| sink_rate_target                                  | `float32` | m/s                                                              |                                                                                | target sinkrate to change altitude. Defaults to FW_T_SINK_MAX if NAN. Not used if height_rate is directly set in FixedWingLongitudinalSetpoint.   |
| speed_weight                                                           | `float32` |                                                                  | [0 : 2]    | , 0=pitch controls altitude only, 2=pitch controls airspeed only                                                                                                                                                                                                                      |
| enforce_low_height_condition | `bool`    | boolean                                                          |                                                                                | if true, the altitude controller is configured with an alternative timeconstant for tighter altitude tracking                                                                                                                                                                         |
| disable_underspeed_protection                     | `bool`    | boolean                                                          |                                                                                | if true, underspeed handling is disabled in the altitude controller                                                                                                                                                                                                                   |

## Constants

| Назва                                                                | Тип      | Значення | Опис |
| -------------------------------------------------------------------- | -------- | -------- | ---- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/LongitudinalControlConfiguration.msg)

:::details
Click here to see original file

```c
# Fixed Wing Longitudinal Control Configuration message
# Used by the fw_lateral_longitudinal_control module and TECS to constrain FixedWingLongitudinalSetpoint messages
# and configure the resultant setpoints.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                        # time since system start (microseconds)

float32 pitch_min   			# [rad][@range -pi, pi] defaults to FW_P_LIM_MIN if NAN.
float32 pitch_max   			# [rad][@range -pi, pi] defaults to FW_P_LIM_MAX if NAN.
float32 throttle_min 			# [norm] [@range 0,1] deaults to FW_THR_MIN if NAN.
float32 throttle_max 			# [norm] [@range 0,1] defaults to FW_THR_MAX if NAN.
float32 climb_rate_target 		# [m/s] target climbrate to change altitude. Defaults to FW_T_CLIMB_MAX if NAN. Not used if height_rate is directly set in FixedWingLongitudinalSetpoint.
float32 sink_rate_target 		# [m/s] target sinkrate to change altitude. Defaults to FW_T_SINK_MAX if NAN. Not used if height_rate is directly set in FixedWingLongitudinalSetpoint.
float32 speed_weight 			# [@range 0,2], 0=pitch controls altitude only, 2=pitch controls airspeed only
bool enforce_low_height_condition 	# [boolean] if true, the altitude controller is configured with an alternative timeconstant for tighter altitude tracking
bool disable_underspeed_protection 	# [boolean] if true, underspeed handling is disabled in the altitude controller
```

:::
