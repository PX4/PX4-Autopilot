uint64 timestamp				# time since system start (microseconds)
uint32 servo_errorcount				# Number of reported errors by servo - if supported
float32 servo_voltage				# Voltage measured from current servo [V] - if supported
float32 servo_current				# Current measured from current servo [A] - if supported
uint8 servo_temperature				# Temperature measured from current servo [degC] - if supported
uint8 servo_address				# Address of current servo (in most cases 1-8 / must be set by driver)

uint8 servo_state				# State of servo - depends on Vendor

uint8 failures					# Bitmask to indicate the internal servo faults

uint8 FAILURE_NONE = 0
uint8 FAILURE_OVER_CURRENT_MASK = 1 		# (1 << 0)
uint8 FAILURE_OVER_VOLTAGE_MASK = 2 		# (1 << 1)
uint8 FAILURE_OVER_TEMPERATURE_MASK = 4 	# (1 << 2)
uint8 FAILURE_OVER_RPM_MASK = 8			# (1 << 3)
uint8 FAILURE_INCONSISTENT_CMD_MASK = 16 	# (1 << 4)  Set if servo received an inconsistent command (i.e out of boundaries)
uint8 FAILURE_MOTOR_STUCK_MASK = 32		# (1 << 5)
uint8 FAILURE_GENERIC_MASK = 64			# (1 << 6)
