# Main state, i.e. what user wants. Controlled by RC or from ground station via telemetry link.
uint8 MAIN_STATE_MANUAL = 0
uint8 MAIN_STATE_ALTCTL = 1
uint8 MAIN_STATE_POSCTL = 2
uint8 MAIN_STATE_AUTO_MISSION = 3
uint8 MAIN_STATE_AUTO_LOITER = 4
uint8 MAIN_STATE_AUTO_RTL = 5
uint8 MAIN_STATE_ACRO = 6
uint8 MAIN_STATE_OFFBOARD = 7
uint8 MAIN_STATE_MAX = 8

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
uint8 NAVIGATION_STATE_MAX = 15

# VEHICLE_MODE_FLAG, same as MAV_MODE_FLAG of MAVLink 1.0 protocol
uint8 VEHICLE_MODE_FLAG_SAFETY_ARMED = 128
uint8 VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
uint8 VEHICLE_MODE_FLAG_HIL_ENABLED = 32
uint8 VEHICLE_MODE_FLAG_STABILIZED_ENABLED = 16
uint8 VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8
uint8 VEHICLE_MODE_FLAG_AUTO_ENABLED = 4
uint8 VEHICLE_MODE_FLAG_TEST_ENABLED = 2
uint8 VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

# VEHICLE_TYPE, should match 1:1 MAVLink's MAV_TYPE ENUM
uint8 VEHICLE_TYPE_GENERIC = 0		    # Generic micro air vehicle.
uint8 VEHICLE_TYPE_FIXED_WING = 1	    # Fixed wing aircraft.
uint8 VEHICLE_TYPE_QUADROTOR = 2	    # Quadrotor
uint8 VEHICLE_TYPE_COAXIAL = 3		    # Coaxial helicopter
uint8 VEHICLE_TYPE_HELICOPTER = 4	    # Normal helicopter with tail rotor.
uint8 VEHICLE_TYPE_ANTENNA_TRACKER = 5	    # Ground installation
uint8 VEHICLE_TYPE_GCS = 6		    # Operator control unit / ground control station
uint8 VEHICLE_TYPE_AIRSHIP = 7		    # Airship, controlled
uint8 VEHICLE_TYPE_FREE_BALLOON = 8	    # Free balloon, uncontrolled
uint8 VEHICLE_TYPE_ROCKET = 9		    # Rocket
uint8 VEHICLE_TYPE_GROUND_ROVER = 10	    # Ground rover
uint8 VEHICLE_TYPE_SURFACE_BOAT = 11	    # Surface vessel, boat, ship
uint8 VEHICLE_TYPE_SUBMARINE = 12	    # Submarine
uint8 VEHICLE_TYPE_HEXAROTOR = 13	    # Hexarotor
uint8 VEHICLE_TYPE_OCTOROTOR = 14	    # Octorotor
uint8 VEHICLE_TYPE_TRICOPTER = 15	    # Octorotor
uint8 VEHICLE_TYPE_FLAPPING_WING = 16	    # Flapping wing
uint8 VEHICLE_TYPE_KITE = 17		    # Kite
uint8 VEHICLE_TYPE_ONBOARD_CONTROLLER=18    # Onboard companion controller
uint8 VEHICLE_TYPE_VTOL_DUOROTOR = 19	    # Vtol with two engines
uint8 VEHICLE_TYPE_VTOL_QUADROTOR = 20	    # Vtol with four engines*/
uint8 VEHICLE_TYPE_ENUM_END = 21

uint8 VEHICLE_BATTERY_WARNING_NONE = 0	    # no battery low voltage warning active
uint8 VEHICLE_BATTERY_WARNING_LOW = 1	    # warning of low voltage
uint8 VEHICLE_BATTERY_WARNING_CRITICAL = 2  # alerting of critical voltage

# state machine / state of vehicle.
# Encodes the complete system state and is set by the commander app.
uint16 counter   # incremented by the writing thread everytime new data is stored
uint64 timestamp # in microseconds since system start, is set whenever the writing thread stores new data

uint8 main_state		    	# main state machine
uint8 nav_state				# set navigation state machine to specified value
uint8 arming_state			# current arming state
uint8 hil_state				# current hil state
bool failsafe				# true if system is in failsafe state

int32 system_type			# system type, inspired by MAVLink's VEHICLE_TYPE enum
int32 system_id				# system id, inspired by MAVLink's system ID field
int32 component_id			# subsystem / component id, inspired by MAVLink's component ID field

bool is_rotary_wing			# True if system is in rotary wing configuration, so for a VTOL this is only true while flying as a multicopter
bool is_vtol				# True if the system is VTOL capable
bool vtol_fw_permanent_stab		# True if vtol should stabilize attitude for fw in manual mode

bool condition_battery_voltage_valid
bool condition_system_in_air_restore		# true if we can restore in mid air
bool condition_system_sensors_initialized
bool condition_system_returned_to_home
bool condition_auto_mission_available
bool condition_global_position_valid		# set to true by the commander app if the quality of the position estimate is good enough to use it for navigation
bool condition_launch_position_valid		# indicates a valid launch position
bool condition_home_position_valid		# indicates a valid home position (a valid home position is not always a valid launch)
bool condition_local_position_valid
bool condition_local_altitude_valid
bool condition_airspeed_valid			# set to true by the commander app if there is a valid airspeed measurement available
bool condition_landed				# true if vehicle is landed, always true if disarmed
bool condition_power_input_valid		# set if input power is valid
float32 avionics_power_rail_voltage		# voltage of the avionics power rail

bool rc_signal_found_once
bool rc_signal_lost				# true if RC reception lost
uint64 rc_signal_lost_timestamp			# Time at which the RC reception was lost
bool rc_signal_lost_cmd				# true if RC lost mode is commanded
bool rc_input_blocked				# set if RC input should be ignored

bool data_link_lost				# datalink to GCS lost
bool data_link_lost_cmd				# datalink to GCS lost mode commanded
uint8 data_link_lost_counter			# counts unique data link lost events
bool engine_failure				# Set to true if an engine failure is detected
bool engine_failure_cmd				# Set to true if an engine failure mode is commanded
bool gps_failure				# Set to true if a gps failure is detected
bool gps_failure_cmd				# Set to true if a gps failure mode is commanded

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

uint8 battery_warning    # current battery warning mode, as defined by VEHICLE_BATTERY_WARNING enum
uint16 drop_rate_comm
uint16 errors_comm
uint16 errors_count1
uint16 errors_count2
uint16 errors_count3
uint16 errors_count4

bool circuit_breaker_engaged_power_check
bool circuit_breaker_engaged_airspd_check
bool circuit_breaker_engaged_enginefailure_check
bool circuit_breaker_engaged_gpsfailure_check
