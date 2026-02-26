---
pageClass: is-wide-page
---

# FailsafeFlags (UORB message)

Input flags for the failsafe state machine set by the arming & health checks.

Flags must be named such that false == no failure (e.g. \_invalid, \_unhealthy, \_lost)
The flag comments are used as label for the failsafe state machine simulation

**TOPICS:** failsafe_flags

## Fields

| 명칭                                                                                                                                                                     | 형식       | Unit [Frame] | Range/Enum | 설명                                                                                                                                           |
| ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                                                                                              | `uint64` |                                                                  |            | time since system start (microseconds)                                                                                    |
| mode_req_angular_velocity                                                                               | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_attitude                                                                                                            | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_local_alt                                                                                      | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_local_position                                                                                 | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_local_position_relaxed                                                    | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_global_position                                                                                | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_global_position_relaxed                                                   | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_mission                                                                                                             | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_offboard_signal                                                                                | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_home_position                                                                                  | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_wind_and_flight_time_compliance | `uint32` |                                                                  |            | if set, mode cannot be entered if wind or flight time limit exceeded                                                                         |
| mode_req_prevent_arming                                                                                 | `uint32` |                                                                  |            | if set, cannot arm while in this mode                                                                                                        |
| mode_req_manual_control                                                                                 | `uint32` |                                                                  |            |                                                                                                                                              |
| mode_req_other                                                                                                               | `uint32` |                                                                  |            | other requirements, not covered above (for external modes)                                                                |
| angular_velocity_invalid                                                                                                     | `bool`   |                                                                  |            | Angular velocity invalid                                                                                                                     |
| attitude_invalid                                                                                                                                  | `bool`   |                                                                  |            | Attitude invalid                                                                                                                             |
| local_altitude_invalid                                                                                                       | `bool`   |                                                                  |            | Local altitude invalid                                                                                                                       |
| local_position_invalid                                                                                                       | `bool`   |                                                                  |            | Local position estimate invalid                                                                                                              |
| local_position_invalid_relaxed                                                                          | `bool`   |                                                                  |            | Local position with reduced accuracy requirements invalid (e.g. flying with optical flow) |
| local_velocity_invalid                                                                                                       | `bool`   |                                                                  |            | Local velocity estimate invalid                                                                                                              |
| global_position_invalid                                                                                                      | `bool`   |                                                                  |            | Global position estimate invalid                                                                                                             |
| global_position_invalid_relaxed                                                                         | `bool`   |                                                                  |            | Global position estimate invalid with relaxed accuracy requirements                                                                          |
| auto_mission_missing                                                                                                         | `bool`   |                                                                  |            | No mission available                                                                                                                         |
| offboard_control_signal_lost                                                                            | `bool`   |                                                                  |            | Offboard signal lost                                                                                                                         |
| home_position_invalid                                                                                                        | `bool`   |                                                                  |            | No home position available                                                                                                                   |
| manual_control_signal_lost                                                                              | `bool`   |                                                                  |            | Manual control (RC) signal lost                                                                                           |
| gcs_connection_lost                                                                                                          | `bool`   |                                                                  |            | GCS connection lost                                                                                                                          |
| battery_warning                                                                                                                                   | `uint8`  |                                                                  |            | Battery warning level (see BatteryStatus.msg)                                                             |
| battery_low_remaining_time                                                                              | `bool`   |                                                                  |            | Low battery based on remaining flight time                                                                                                   |
| battery_unhealthy                                                                                                                                 | `bool`   |                                                                  |            | Battery unhealthy                                                                                                                            |
| geofence_breached                                                                                                                                 | `bool`   |                                                                  |            | Geofence breached (one or multiple)                                                                                       |
| mission_failure                                                                                                                                   | `bool`   |                                                                  |            | Mission failure                                                                                                                              |
| vtol_fixed_wing_system_failure                                                     | `bool`   |                                                                  |            | vehicle in fixed-wing system failure failsafe mode (after quad-chute)                                                     |
| wind_limit_exceeded                                                                                                          | `bool`   |                                                                  |            | Wind limit exceeded                                                                                                                          |
| flight_time_limit_exceeded                                                                              | `bool`   |                                                                  |            | Maximum flight time exceeded                                                                                                                 |
| position_accuracy_low                                                                                                        | `bool`   |                                                                  |            | Position estimate has dropped below threshold, but is currently still declared valid                                                         |
| navigator_failure                                                                                                                                 | `bool`   |                                                                  |            | Navigator failed to execute a mode                                                                                                           |
| fd_critical_failure                                                                                                          | `bool`   |                                                                  |            | Critical failure (attitude/altitude limit exceeded, or external ATS)                                                      |
| fd_esc_arming_failure                                                                                   | `bool`   |                                                                  |            | ESC failed to arm                                                                                                                            |
| fd_imbalanced_prop                                                                                                           | `bool`   |                                                                  |            | Imbalanced propeller detected                                                                                                                |
| fd_motor_failure                                                                                                             | `bool`   |                                                                  |            | Motor failure                                                                                                                                |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FailsafeFlags.msg)

:::details
Click here to see original file

```c
# Input flags for the failsafe state machine set by the arming & health checks.
#
# Flags must be named such that false == no failure (e.g. _invalid, _unhealthy, _lost)
# The flag comments are used as label for the failsafe state machine simulation

uint64 timestamp				# time since system start (microseconds)

# Per-mode requirements
uint32 mode_req_angular_velocity
uint32 mode_req_attitude
uint32 mode_req_local_alt
uint32 mode_req_local_position
uint32 mode_req_local_position_relaxed
uint32 mode_req_global_position
uint32 mode_req_global_position_relaxed
uint32 mode_req_mission
uint32 mode_req_offboard_signal
uint32 mode_req_home_position
uint32 mode_req_wind_and_flight_time_compliance # if set, mode cannot be entered if wind or flight time limit exceeded
uint32 mode_req_prevent_arming    # if set, cannot arm while in this mode
uint32 mode_req_manual_control
uint32 mode_req_other             # other requirements, not covered above (for external modes)


# Mode requirements
bool angular_velocity_invalid         # Angular velocity invalid
bool attitude_invalid                 # Attitude invalid
bool local_altitude_invalid           # Local altitude invalid
bool local_position_invalid           # Local position estimate invalid
bool local_position_invalid_relaxed   # Local position with reduced accuracy requirements invalid (e.g. flying with optical flow)
bool local_velocity_invalid           # Local velocity estimate invalid
bool global_position_invalid          # Global position estimate invalid
bool global_position_invalid_relaxed  # Global position estimate invalid with relaxed accuracy requirements
bool auto_mission_missing             # No mission available
bool offboard_control_signal_lost     # Offboard signal lost
bool home_position_invalid            # No home position available

# Control links
bool manual_control_signal_lost       # Manual control (RC) signal lost
bool gcs_connection_lost              # GCS connection lost

# Battery
uint8 battery_warning                 # Battery warning level (see BatteryStatus.msg)
bool battery_low_remaining_time       # Low battery based on remaining flight time
bool battery_unhealthy                # Battery unhealthy

# Other
bool geofence_breached        	      # Geofence breached (one or multiple)
bool mission_failure                  # Mission failure
bool vtol_fixed_wing_system_failure   # vehicle in fixed-wing system failure failsafe mode (after quad-chute)
bool wind_limit_exceeded              # Wind limit exceeded
bool flight_time_limit_exceeded       # Maximum flight time exceeded
bool position_accuracy_low            # Position estimate has dropped below threshold, but is currently still declared valid
bool navigator_failure        	      # Navigator failed to execute a mode

# Failure detector
bool fd_critical_failure              # Critical failure (attitude/altitude limit exceeded, or external ATS)
bool fd_esc_arming_failure            # ESC failed to arm
bool fd_imbalanced_prop               # Imbalanced propeller detected
bool fd_motor_failure                 # Motor failure
```

:::
