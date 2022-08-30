# TODO: rename to failsafe_flags (will be input to failsafe state machine)
#
# Input flags for the failsafe state machine.
# Flags must be named such that false == no failure (e.g. _invalid, _unhealthy, _lost)
# The flag comments are used as label for the failsafe state machine simulation

uint64 timestamp				# time since system start (microseconds)

# Per-mode requirements
uint32 mode_req_angular_velocity
uint32 mode_req_attitude
uint32 mode_req_local_position
uint32 mode_req_local_position_relaxed
uint32 mode_req_global_position
uint32 mode_req_local_alt
uint32 mode_req_mission
uint32 mode_req_offboard_signal
uint32 mode_req_home_position
uint32 mode_req_prevent_arming    # if set, cannot arm while in this mode
uint32 mode_req_other             # other requirements, not covered above (for external modes)


bool calibration_enabled
bool pre_flight_checks_pass		# true if all checks necessary to arm pass
bool auto_mission_available		# Mission available
bool angular_velocity_valid 		# Angular velocity valid
bool attitude_valid 			# Attitude valid
bool local_altitude_valid               # Local altitude valid
bool local_position_valid		# Local position estimate valid
bool local_position_valid_relaxed	# Local position with reduced accuracy requirements (e.g. flying with optical flow)
bool local_velocity_valid		# set to true by the commander app if the quality of the local horizontal velocity data is good enough to use for navigation
bool global_position_valid		# Global position estimate valid
bool gps_position_valid
bool home_position_valid		# Home position valid

bool offboard_control_signal_lost # Offboard signal lost
bool geofence_violated 				# Geofence violated

bool rc_signal_lost # RC signal lost
bool data_link_lost # Data link lost
bool mission_failure # Mission failure
bool rc_calibration_in_progress
bool vtol_transition_failure                        # VTOL transition failed

bool battery_low_remaining_time
bool battery_unhealthy

bool wind_limit_exceeded
bool flight_time_limit_exceeded

# failure detector
bool fd_esc_arming_failure
bool fd_critical_failure # Critical failure (attitude/altitude limit exceeded, or external ATS)
bool fd_imbalanced_prop # Imbalanced propeller detected
bool fd_motor_failure

uint8 battery_warning # Battery warning level

