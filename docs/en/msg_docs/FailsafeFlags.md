---
pageClass: is-wide-page
---

# FailsafeFlags (UORB message)

Input flags for the failsafe state machine set by the arming & health checks.

Flags must be named such that false == no failure (e.g. \_invalid, \_unhealthy, \_lost)
The flag comments are used as label for the failsafe state machine simulation

**TOPICS:** failsafe_flags

## Fields

| Name                                                                                              | Type     | Unit [Frame] | Range/Enum | Description                                                                                      |
| ------------------------------------------------------------------------------------------------- | -------- | ------------ | ---------- | ------------------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                                                               | `uint64` |              |            | time since system start (microseconds)                                                           |
| <a id="fld_mode_req_angular_velocity"></a>mode_req_angular_velocity                               | `uint32` |              |            |
| <a id="fld_mode_req_attitude"></a>mode_req_attitude                                               | `uint32` |              |            |
| <a id="fld_mode_req_local_alt"></a>mode_req_local_alt                                             | `uint32` |              |            |
| <a id="fld_mode_req_local_position"></a>mode_req_local_position                                   | `uint32` |              |            |
| <a id="fld_mode_req_local_position_relaxed"></a>mode_req_local_position_relaxed                   | `uint32` |              |            |
| <a id="fld_mode_req_global_position"></a>mode_req_global_position                                 | `uint32` |              |            |
| <a id="fld_mode_req_global_position_relaxed"></a>mode_req_global_position_relaxed                 | `uint32` |              |            |
| <a id="fld_mode_req_mission"></a>mode_req_mission                                                 | `uint32` |              |            |
| <a id="fld_mode_req_offboard_signal"></a>mode_req_offboard_signal                                 | `uint32` |              |            |
| <a id="fld_mode_req_home_position"></a>mode_req_home_position                                     | `uint32` |              |            |
| <a id="fld_mode_req_wind_and_flight_time_compliance"></a>mode_req_wind_and_flight_time_compliance | `uint32` |              |            | if set, mode cannot be entered if wind or flight time limit exceeded                             |
| <a id="fld_mode_req_prevent_arming"></a>mode_req_prevent_arming                                   | `uint32` |              |            | if set, cannot arm while in this mode                                                            |
| <a id="fld_mode_req_manual_control"></a>mode_req_manual_control                                   | `uint32` |              |            |
| <a id="fld_mode_req_other"></a>mode_req_other                                                     | `uint32` |              |            | other requirements, not covered above (for external modes)                                       |
| <a id="fld_angular_velocity_invalid"></a>angular_velocity_invalid                                 | `bool`   |              |            | Angular velocity invalid                                                                         |
| <a id="fld_attitude_invalid"></a>attitude_invalid                                                 | `bool`   |              |            | Attitude invalid                                                                                 |
| <a id="fld_local_altitude_invalid"></a>local_altitude_invalid                                     | `bool`   |              |            | Local altitude invalid                                                                           |
| <a id="fld_local_position_invalid"></a>local_position_invalid                                     | `bool`   |              |            | Local position estimate invalid                                                                  |
| <a id="fld_local_position_invalid_relaxed"></a>local_position_invalid_relaxed                     | `bool`   |              |            | Local position with reduced accuracy requirements invalid (e.g. flying with optical flow)        |
| <a id="fld_local_velocity_invalid"></a>local_velocity_invalid                                     | `bool`   |              |            | Local velocity estimate invalid                                                                  |
| <a id="fld_global_position_invalid"></a>global_position_invalid                                   | `bool`   |              |            | Global position estimate invalid                                                                 |
| <a id="fld_global_position_invalid_relaxed"></a>global_position_invalid_relaxed                   | `bool`   |              |            | Global position estimate invalid with relaxed accuracy requirements                              |
| <a id="fld_auto_mission_missing"></a>auto_mission_missing                                         | `bool`   |              |            | No mission available                                                                             |
| <a id="fld_offboard_control_signal_lost"></a>offboard_control_signal_lost                         | `bool`   |              |            | Offboard signal lost                                                                             |
| <a id="fld_home_position_invalid"></a>home_position_invalid                                       | `bool`   |              |            | No home position available                                                                       |
| <a id="fld_manual_control_signal_lost"></a>manual_control_signal_lost                             | `bool`   |              |            | Manual control (RC) signal lost                                                                  |
| <a id="fld_gcs_connection_lost"></a>gcs_connection_lost                                           | `bool`   |              |            | GCS connection lost                                                                              |
| <a id="fld_battery_warning"></a>battery_warning                                                   | `uint8`  |              |            | Battery warning level (see BatteryStatus.msg)                                                    |
| <a id="fld_battery_low_remaining_time"></a>battery_low_remaining_time                             | `bool`   |              |            | Low battery based on remaining flight time                                                       |
| <a id="fld_battery_unhealthy"></a>battery_unhealthy                                               | `bool`   |              |            | Battery unhealthy                                                                                |
| <a id="fld_fd_critical_failure"></a>fd_critical_failure                                           | `bool`   |              |            | Critical failure (attitude limit exceeded, or external ATS)                                      |
| <a id="fld_fd_esc_arming_failure"></a>fd_esc_arming_failure                                       | `bool`   |              |            | ESC failed to arm                                                                                |
| <a id="fld_fd_imbalanced_prop"></a>fd_imbalanced_prop                                             | `bool`   |              |            | Imbalanced propeller detected                                                                    |
| <a id="fld_fd_motor_failure"></a>fd_motor_failure                                                 | `bool`   |              |            | Motor failure                                                                                    |
| <a id="fld_fd_alt_loss"></a>fd_alt_loss                                                           | `bool`   |              |            | Uncommanded altitude loss (rotary-wing, altitude-controlled flight)                              |
| <a id="fld_geofence_breached"></a>geofence_breached                                               | `bool`   |              |            | Geofence breached (one or multiple)                                                              |
| <a id="fld_mission_failure"></a>mission_failure                                                   | `bool`   |              |            | Mission failure                                                                                  |
| <a id="fld_vtol_fixed_wing_system_failure"></a>vtol_fixed_wing_system_failure                     | `bool`   |              |            | vehicle in fixed-wing system failure failsafe mode (after quad-chute)                            |
| <a id="fld_wind_limit_exceeded"></a>wind_limit_exceeded                                           | `bool`   |              |            | Wind limit exceeded                                                                              |
| <a id="fld_flight_time_limit_exceeded"></a>flight_time_limit_exceeded                             | `bool`   |              |            | Maximum flight time exceeded                                                                     |
| <a id="fld_position_accuracy_low"></a>position_accuracy_low                                       | `bool`   |              |            | Position estimate has dropped below threshold, but is currently still declared valid             |
| <a id="fld_navigator_failure"></a>navigator_failure                                               | `bool`   |              |            | Navigator failed to execute a mode                                                               |
| <a id="fld_parachute_unhealthy"></a>parachute_unhealthy                                           | `bool`   |              |            | Parachute system missing or unhealthy                                                            |
| <a id="fld_remote_id_unhealthy"></a>remote_id_unhealthy                                           | `bool`   |              |            | Remote ID (Open Drone ID) system missing or unhealthy                                            |
| <a id="fld_gnss_lost"></a>gnss_lost                                                               | `bool`   |              |            | Active GNSS count dropped below SYS_HAS_NUM_GNSS, or two receivers report inconsistent positions |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FailsafeFlags.msg)

::: details Click here to see original file

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

# Failure detector
bool fd_critical_failure              # Critical failure (attitude limit exceeded, or external ATS)
bool fd_esc_arming_failure            # ESC failed to arm
bool fd_imbalanced_prop               # Imbalanced propeller detected
bool fd_motor_failure                 # Motor failure
bool fd_alt_loss                      # Uncommanded altitude loss (rotary-wing, altitude-controlled flight)

# Other
bool geofence_breached        	      # Geofence breached (one or multiple)
bool mission_failure                  # Mission failure
bool vtol_fixed_wing_system_failure   # vehicle in fixed-wing system failure failsafe mode (after quad-chute)
bool wind_limit_exceeded              # Wind limit exceeded
bool flight_time_limit_exceeded       # Maximum flight time exceeded
bool position_accuracy_low            # Position estimate has dropped below threshold, but is currently still declared valid
bool navigator_failure        	      # Navigator failed to execute a mode
bool parachute_unhealthy              # Parachute system missing or unhealthy
bool remote_id_unhealthy              # Remote ID (Open Drone ID) system missing or unhealthy
bool gnss_lost                        # Active GNSS count dropped below SYS_HAS_NUM_GNSS, or two receivers report inconsistent positions
```

:::
