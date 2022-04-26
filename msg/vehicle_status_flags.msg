# Input flags for the failsafe state machine.
# Flags must be named such that false == no failure (e.g. _invalid, _unhealthy, _lost)
# The flag comments are used as label for the failsafe state machine simulation

uint64 timestamp				# time since system start (microseconds)

bool calibration_enabled
bool system_sensors_initialized
bool system_hotplug_timeout		# true if the hotplug sensor search is over
bool auto_mission_available		# Mission available
bool angular_velocity_valid
bool attitude_valid
bool local_altitude_valid               # Local altitude valid
bool local_position_valid		# Local position estimate valid
bool local_velocity_valid		# set to true by the commander app if the quality of the local horizontal velocity data is good enough to use for navigation
bool global_position_valid		# Global position estimate valid
bool gps_position_valid
bool home_position_valid		# Home position valid
bool power_input_valid                # set if input power is valid
bool battery_healthy                # Battery healthy
bool escs_error		      # One or more ESCs offline
bool escs_failure		      # One or more ESCs has a failure

bool position_reliant_on_gps
bool position_reliant_on_optical_flow
bool position_reliant_on_vision_position

bool dead_reckoning
bool flight_terminated

bool circuit_breaker_engaged_power_check
bool circuit_breaker_engaged_airspd_check
bool circuit_breaker_flight_termination_disabled
bool circuit_breaker_engaged_usb_check
bool circuit_breaker_engaged_posfailure_check	# set to true when the position valid checks have been disabled
bool circuit_breaker_vtol_fw_arming_check 	# set to true if for VTOLs arming in fixed-wing mode should be allowed

bool offboard_control_signal_lost # Offboard signal lost
bool geofence_violated 				# Geofence violated

bool rc_signal_found_once
bool rc_signal_lost # RC signal lost
bool data_link_lost # Data link lost
bool mission_failure # Mission failure
bool rc_calibration_in_progress
bool rc_calibration_valid                            # set if RC calibration is valid
bool vtol_transition_failure                        # VTOL transition failed
bool usb_connected                                # status of the USB power supply
bool sd_card_detected_once                        # set to true if the SD card was detected

bool avoidance_system_required					  # Set to true if avoidance system is enabled via COM_OBS_AVOID parameter
bool avoidance_system_valid                       # Status of the obstacle avoidance system

bool parachute_system_present
bool parachute_system_healthy
