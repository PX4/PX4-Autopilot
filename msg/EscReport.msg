uint64 timestamp					# time since system start (microseconds)
uint32 esc_errorcount					# Number of reported errors by ESC - if supported
int32 esc_rpm						# Motor RPM, negative for reverse rotation [RPM] - if supported
float32 esc_voltage					# Voltage measured from current ESC [V] - if supported
float32 esc_current					# Current measured from current ESC [A] - if supported
float32 esc_temperature					# Temperature measured from current ESC [degC] - if supported
uint8 esc_address					# Address of current ESC (in most cases 1-8 / must be set by driver)
uint8 esc_cmdcount					# Counter of number of commands

uint8 esc_state						# State of ESC - depend on Vendor

uint8 ESC_STATE_NOT_SPINNING = 0
uint8 ESC_STATE_SPINNING_UP = 1			# Startup burst
uint8 ESC_STATE_SPINUP_START_NO_CONTROL = 4	# ESC Start up spinning no user control
uint8 ESC_STATE_SPINNING = 5			# ESC spinning under user control
uint8 ESC_STATE_TURTLE_MODE = 16		# Turtle Mode
uint8 ESC_STATE_UNKNOWN = 17			# Unknown
uint8 ESC_STATE_COUNT = 18 			# Counter - keep it as last element!

uint8 actuator_function				# actuator output function (one of Motor1...MotorN)

uint16 failures					# Bitmask to indicate the internal ESC faults
int8 esc_power					# Applied power 0-100 in % (negative values reserved)

uint8 FAILURE_OVER_CURRENT = 0 			# (1 << 0)
uint8 FAILURE_OVER_VOLTAGE = 1 			# (1 << 1)
uint8 FAILURE_MOTOR_OVER_TEMPERATURE = 2 	# (1 << 2)
uint8 FAILURE_OVER_RPM = 3			# (1 << 3)
uint8 FAILURE_INCONSISTENT_CMD = 4 		# (1 << 4)  Set if ESC received an inconsistent command (i.e out of boundaries)
uint8 FAILURE_MOTOR_STUCK = 5			# (1 << 5)
uint8 FAILURE_GENERIC = 6			# (1 << 6)
uint8 FAILURE_MOTOR_WARN_TEMPERATURE = 7	# (1 << 7)
uint8 FAILURE_WARN_ESC_TEMPERATURE = 8		# (1 << 8)
uint8 FAILURE_OVER_ESC_TEMPERATURE = 9		# (1 << 9)
uint8 ESC_FAILURE_COUNT = 10 			# Counter - keep it as last element!
