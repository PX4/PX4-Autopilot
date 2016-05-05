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
uint8 NAVIGATION_STATE_UNUSED = 11		# Free slot
uint8 NAVIGATION_STATE_DESCEND = 12		# Descend mode (no position control)
uint8 NAVIGATION_STATE_TERMINATION = 13		# Termination mode
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15		# Stabilized mode
uint8 NAVIGATION_STATE_RATTITUDE = 16		# Rattitude (aka "flip") mode
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17	# Takeoff
uint8 NAVIGATION_STATE_AUTO_LAND = 18		# Land
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19	# Auto Follow
uint8 NAVIGATION_STATE_MAX = 20

uint8 RC_IN_MODE_DEFAULT = 0
uint8 RC_IN_MODE_OFF = 1
uint8 RC_IN_MODE_GENERATED = 2

# state machine / state of vehicle.
# Encodes the complete system state and is set by the commander app.
uint64 timestamp # in microseconds since system start, is set whenever the writing thread stores new data

uint8 nav_state				# set navigation state machine to specified value
uint8 arming_state			# current arming state
uint8 hil_state				# current hil state
bool failsafe				# true if system is in failsafe state

uint8 system_type			# system type, contains mavlink MAV_TYPE
uint32 system_id			# system id, contains MAVLink's system ID field
uint32 component_id			# subsystem / component id, contains MAVLink's component ID field

bool is_rotary_wing			# True if system is in rotary wing configuration, so for a VTOL this is only true while flying as a multicopter
bool is_vtol				# True if the system is VTOL capable
bool vtol_fw_permanent_stab		# True if vtol should stabilize attitude for fw in manual mode
bool in_transition_mode			# True if VTOL is doing a transition

bool rc_signal_lost				# true if RC reception lost
uint8 rc_input_mode				# set to 1 to disable the RC input, 2 to enable manual control to RC in mapping.

bool data_link_lost				# datalink to GCS lost
uint8 data_link_lost_counter			# counts unique data link lost events
bool engine_failure				# Set to true if an engine failure is detected
bool engine_failure_cmd				# Set to true if an engine failure mode is commanded
bool mission_failure				# Set to true if mission could not continue/finish

# see SYS_STATUS mavlink message for the following
uint32 onboard_control_sensors_present
uint32 onboard_control_sensors_enabled
uint32 onboard_control_sensors_health

float32 load					# processor load from 0 to 1

