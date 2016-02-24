# Main state, i.e. what user wants. Controlled by RC or from ground station via telemetry link.
uint8 MAIN_STATE_MANUAL = 0
uint8 MAIN_STATE_ALTCTL = 1
uint8 MAIN_STATE_POSCTL = 2
uint8 MAIN_STATE_AUTO_MISSION = 3
uint8 MAIN_STATE_AUTO_LOITER = 4
uint8 MAIN_STATE_AUTO_RTL = 5
uint8 MAIN_STATE_ACRO = 6
uint8 MAIN_STATE_OFFBOARD = 7
uint8 MAIN_STATE_STAB = 8
uint8 MAIN_STATE_RATTITUDE = 9
uint8 MAIN_STATE_AUTO_TAKEOFF = 10
uint8 MAIN_STATE_AUTO_LAND = 11
uint8 MAIN_STATE_MAX = 12

# If you change the order, add or remove arming_state_t states make sure to update the arrays
# in state_machine_helper.cpp as well.
uint8 ARMING_STATE_INIT = 0
uint8 ARMING_STATE_STANDBY = 1
uint8 ARMING_STATE_ARMED = 2
uint8 ARMING_STATE_ARMED_ERROR = 3
uint8 ARMING_STATE_STANDBY_ERROR = 4
uint8 ARMING_STATE_REBOOT = 5
uint8 ARMING_STATE_IN_AIR_RESTORE = 6
uint8 ARMING_STATE_MAX = 7

uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

# Navigation state, i.e. "what should vehicle do".
uint8 NAVIGATION_STATE_MANUAL = 0		# Manual mode
uint8 NAVIGATION_STATE_ALTCTL = 1		# Altitude control mode
uint8 NAVIGATION_STATE_POSCTL = 2		# Position control mode
uint8 NAVIGATION_STATE_AUTO_MISSION = 3		# Auto mission mode
uint8 NAVIGATION_STATE_AUTO_LOITER = 4		# Auto loiter mode
uint8 NAVIGATION_STATE_AUTO_RTL = 5		# Auto return to launch mode
uint8 NAVIGATION_STATE_AUTO_RCRECOVER = 6	# RC recover mode
uint8 NAVIGATION_STATE_AUTO_RTGS = 7		# Auto return to groundstation on data link loss
uint8 NAVIGATION_STATE_AUTO_LANDENGFAIL = 8 	# Auto land on engine failure
uint8 NAVIGATION_STATE_AUTO_LANDGPSFAIL = 9	# Auto land on gps failure (e.g. open loop loiter down)
uint8 NAVIGATION_STATE_ACRO = 10		# Acro mode
uint8 NAVIGATION_STATE_LAND = 11		# Land mode
uint8 NAVIGATION_STATE_DESCEND = 12		# Descend mode (no position control)
uint8 NAVIGATION_STATE_TERMINATION = 13		# Termination mode
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15		# Stabilized mode
uint8 NAVIGATION_STATE_RATTITUDE = 16		# Rattitude (aka "flip") mode
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17	# Takeoff
uint8 NAVIGATION_STATE_MAX = 18

uint8 RC_IN_MODE_DEFAULT = 0
uint8 RC_IN_MODE_OFF = 1
uint8 RC_IN_MODE_GENERATED = 2

# state machine / state of vehicle.
# Encodes the complete system state and is set by the commander app.
uint16 counter   # incremented by the writing thread everytime new data is stored
uint64 timestamp # in microseconds since system start, is set whenever the writing thread stores new data

uint8 main_state		    	# main state machine
uint8 main_state_prev       # previous main state
uint8 nav_state				# set navigation state machine to specified value
uint8 arming_state			# current arming state
uint8 hil_state				# current hil state
bool failsafe				# true if system is in failsafe state
bool calibration_enabled		# true if current calibrating parts of the system. Also sets the system to ARMING_STATE_INIT.

uint8 system_type			# system type, contains mavlink MAV_TYPE
uint32 system_id			# system id, contains MAVLink's system ID field
uint32 component_id			# subsystem / component id, contains MAVLink's component ID field

bool is_rotary_wing			# True if system is in rotary wing configuration, so for a VTOL this is only true while flying as a multicopter
bool is_vtol				# True if the system is VTOL capable
bool vtol_fw_permanent_stab		# True if vtol should stabilize attitude for fw in manual mode
bool in_transition_mode			# True if VTOL is doing a transition

bool condition_battery_voltage_valid
bool condition_system_in_air_restore		# true if we can restore in mid air
bool condition_system_sensors_initialized
bool condition_system_prearm_error_reported	# true if errors have already been reported
bool condition_system_hotplug_timeout		# true if the hotplug sensor search is over
bool condition_system_returned_to_home
bool condition_auto_mission_available
bool condition_global_position_valid		# set to true by the commander app if the quality of the position estimate is good enough to use it for navigation
bool condition_home_position_valid		# indicates a valid home position (a valid home position is not always a valid launch)
bool condition_local_position_valid
bool condition_local_altitude_valid
bool condition_airspeed_valid			# set to true by the commander app if there is a valid airspeed measurement available
bool condition_landed				# true if vehicle is landed, always true if disarmed
bool condition_power_input_valid		# set if input power is valid
float32 avionics_power_rail_voltage		# voltage of the avionics power rail
bool usb_connected				# status of the USB power supply

bool rc_signal_found_once
bool rc_signal_lost				# true if RC reception lost
uint64 rc_signal_lost_timestamp			# Time at which the RC reception was lost
bool rc_signal_lost_cmd				# true if RC lost mode is commanded
bool rc_input_blocked				# set if RC input should be ignored temporarily
uint8 rc_input_mode				# set to 1 to disable the RC input, 2 to enable manual control to RC in mapping.

bool data_link_lost				# datalink to GCS lost
bool data_link_lost_cmd				# datalink to GCS lost mode commanded
uint8 data_link_lost_counter			# counts unique data link lost events
bool engine_failure				# Set to true if an engine failure is detected
bool engine_failure_cmd				# Set to true if an engine failure mode is commanded
bool vtol_transition_failure		# Set to true if vtol transition failed
bool vtol_transition_failure_cmd	# Set to true if vtol transition failure mode is commanded
bool gps_failure				# Set to true if a gps failure is detected
bool gps_failure_cmd				# Set to true if a gps failure mode is commanded
bool mission_failure				# Set to true if mission could not continue/finish
bool barometer_failure				# Set to true if a barometer failure is detected

bool offboard_control_signal_found_once
bool offboard_control_signal_lost
bool offboard_control_signal_weak
uint64 offboard_control_signal_lost_interval	# interval in microseconds without an offboard control message
bool offboard_control_set_by_command		# true if the offboard mode was set by a mavlink command and should not be overridden by RC

# see SYS_STATUS mavlink message for the following
uint32 onboard_control_sensors_present
uint32 onboard_control_sensors_enabled
uint32 onboard_control_sensors_health

float32 load					# processor load from 0 to 1
float32 battery_voltage
float32 battery_current
float32 battery_remaining
float32 battery_discharged_mah
uint32 battery_cell_count

uint8 battery_warning    # current battery warning mode, as defined by VEHICLE_BATTERY_WARNING enum

