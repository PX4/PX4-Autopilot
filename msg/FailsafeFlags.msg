# Input flags for the failsafe state machine set by the arming & health checks.
#
# Flags must be named such that false == no failure (e.g. _invalid, _unhealthy, _lost)
# The flag comments are used as label for the failsafe state machine simulation

uint64 timestamp # time since system start (microseconds)

# Per-mode requirements
uint32 mode_req_ang_vel
uint32 mode_req_attitude
uint32 mode_req_local_alt
uint32 mode_req_local_pos
uint32 mode_req_local_pos_relaxed
uint32 mode_req_global_pos
uint32 mode_req_global_pos_relaxed
uint32 mode_req_mission
uint32 mode_req_offboard_signal
uint32 mode_req_home_pos
uint32 mode_req_wind_flight_time # if set, mode cannot be entered if wind or flight time limit exceeded
uint32 mode_req_prevent_arming # if set, cannot arm while in this mode
uint32 mode_req_manual_control
uint32 mode_req_other # other requirements, not covered above (for external modes)
uint32 mode_not_supported # if set, mode is not supported by the current vehicle architecture

# Mode requirements
bool ang_vel_bad # Angular velocity invalid
bool attitude_bad # Attitude invalid
bool local_alt_bad # Local altitude invalid
bool local_pos_bad # Local position estimate invalid
bool local_pos_bad_relaxed # Local position with reduced accuracy requirements invalid (e.g. flying with optical flow)
bool local_vel_bad # Local velocity estimate invalid
bool global_pos_bad # Global position estimate invalid
bool global_pos_bad_relaxed # Global position estimate invalid with relaxed accuracy requirements
bool auto_mission_missing # No mission available
bool offboard_signal_lost # Offboard signal lost
bool home_pos_bad # No home position available

# Control links
bool rc_conn_lost # Manual control (RC) signal lost
bool gcs_conn_lost # GCS connection lost

# Battery
uint8 bat_warning # Battery warning level (see BatteryStatus.msg)
bool bat_low_time # Low battery based on remaining flight time
bool bat_bad # Battery unhealthy

# Failure detector
bool fd_critical_fail # Critical failure (attitude limit exceeded, or external ATS)
bool fd_esc_arming_fail # ESC failed to arm
bool fd_imbalanced_prop # Imbalanced propeller detected
bool fd_motor_fail # Motor failure
bool fd_motor_stuck_fail # Motor-stuck failure during spool-up
bool fd_alt_loss # Uncommanded altitude loss (rotary-wing, altitude-controlled flight)

# Other
bool gf_emer_breach # Emergency geofence breached (one or multiple)
bool gf_cont_breach # Contingency geofence breached (one or multiple)
bool mission_fail # Mission failure
bool vtol_fw_system_fail # vehicle in fixed-wing system failure failsafe mode (after quad-chute)
bool wind_limit_exceeded # Wind limit exceeded
bool flight_time_exceeded # Maximum flight time exceeded
bool pos_accur_low # Position estimate has dropped below threshold, but is currently still declared valid
bool navigator_fail # Navigator failed to execute a mode
bool parachute_unhealthy # Parachute system missing or unhealthy
bool traff_avoid_unhealthy # Traffic avoidance (ADS-B/FLARM) system missing or unhealthy
bool remote_id_unhealthy # Remote ID (Open Drone ID) system missing or unhealthy
bool gnss_lost # Fewer GNSS receivers are healthy than required, or two receivers report inconsistent positions
