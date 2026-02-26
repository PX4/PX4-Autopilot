---
pageClass: is-wide-page
---

# FixedWingLongitudinalSetpoint (UORB message)

Fixed Wing Longitudinal Setpoint message. Used by the fw_lateral_longitudinal_control module. If pitch_direct and throttle_direct are not both finite, then the controller relies on altitude/height_rate and equivalent_airspeed to control vertical motion. If both altitude and height_rate are NAN, the controller maintains the current altitude.

**TOPICS:** fixed_winglongitudinal_setpoint

## Fields

| 参数名                                      | 类型        | Unit [Frame] | Range/Enum                                                                     | 描述                                                                                                      |
| ---------------------------------------- | --------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------- |
| timestamp                                | `uint64`  |                                                                  |                                                                                | time since system start (microseconds)                                               |
| altitude                                 | `float32` | 米                                                                |                                                                                | Altitude setpoint AMSL, not controlled directly if NAN or if height_rate is finite |
| height_rate         | `float32` | ENU                                                              |                                                                                | Scalar height rate setpoint. NAN if not controlled directly                             |
| equivalent_airspeed | `float32` | 米/秒                                                              | [0 : inf]  | Scalar equivalent airspeed setpoint. NAN if system default should be used               |
| pitch_direct        | `float32` | FRD                                                              | [-pi : pi] | NAN if not controlled, overrides total energy controller                                                |
| throttle_direct     | `float32` | norm                                                             | [0 : 1]    | NAN if not controlled, overrides total energy controller                                                |

## Constants

| 参数名                                                                  | 类型       | 值 | 描述 |
| -------------------------------------------------------------------- | -------- | - | -- |
| <a href="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/FixedWingLongitudinalSetpoint.msg)

:::details
Click here to see original file

```c
# Fixed Wing Longitudinal Setpoint message
# Used by the fw_lateral_longitudinal_control module
# If pitch_direct and throttle_direct are not both finite, then the controller relies on altitude/height_rate and equivalent_airspeed to control vertical motion.
# If both altitude and height_rate are NAN, the controller maintains the current altitude.

uint32 MESSAGE_VERSION = 0

uint64 timestamp                        # time since system start (microseconds)

float32 altitude  			# [m] Altitude setpoint AMSL, not controlled directly if NAN or if height_rate is finite
float32 height_rate 			# [m/s] [ENU] Scalar height rate setpoint. NAN if not controlled directly
float32 equivalent_airspeed 		# [m/s] [@range 0, inf] Scalar equivalent airspeed setpoint. NAN if system default should be used
float32 pitch_direct 			# [rad] [@range -pi, pi] [FRD] NAN if not controlled, overrides total energy controller
float32 throttle_direct 		# [norm] [@range 0,1] NAN if not controlled, overrides total energy controller
```

:::
