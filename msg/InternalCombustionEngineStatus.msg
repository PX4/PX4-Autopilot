uint64 timestamp					# [us] Time since system start

uint8 STATE_STOPPED = 0					# The engine is not running. This is the default state.
uint8 STATE_STARTING = 1				# The engine is starting. This is a transient state.
uint8 STATE_RUNNING = 2					# The engine is running normally.
uint8 STATE_FAULT = 3					# The engine can no longer function.
uint8 state						# [@enum STATE] Engine state

uint8 SUBSTATE_RUN = 0					# The engine is running. This is the default state.
uint8 SUBSTATE_IDLE = 1					# The engine idle rpm controller is running.
uint8 SUBSTATE_REST = 2					# The engine is at rest.
uint8 substate						# [@enum SUBSTATE] Engine substate

uint32 FLAG_GENERAL_ERROR = 1				# General error.

uint32 FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED = 2	# Error of the crankshaft sensor. This flag is optional.
uint32 FLAG_CRANKSHAFT_SENSOR_ERROR = 4

uint32 FLAG_TEMPERATURE_SUPPORTED = 8			# Temperature levels. These flags are optional
uint32 FLAG_TEMPERATURE_BELOW_NOMINAL = 16      	# Under-temperature warning
uint32 FLAG_TEMPERATURE_ABOVE_NOMINAL = 32      	# Over-temperature warning
uint32 FLAG_TEMPERATURE_OVERHEATING = 64      		# Critical overheating
uint32 FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL = 128     	# Exhaust gas over-temperature warning

uint32 FLAG_FUEL_PRESSURE_SUPPORTED = 256		# Fuel pressure. These flags are optional
uint32 FLAG_FUEL_PRESSURE_BELOW_NOMINAL  = 512     	# Under-pressure warning
uint32 FLAG_FUEL_PRESSURE_ABOVE_NOMINAL = 1024   	# Over-pressure warning

uint32 FLAG_DETONATION_SUPPORTED = 2048			# Detonation warning. This flag is optional.
uint32 FLAG_DETONATION_OBSERVED = 4096    		# Detonation condition observed warning

uint32 FLAG_MISFIRE_SUPPORTED = 8192			# Misfire warning. This flag is optional.
uint32 FLAG_MISFIRE_OBSERVED = 16384   			# Misfire condition observed warning

uint32 FLAG_OIL_PRESSURE_SUPPORTED = 32768		# Oil pressure. These flags are optional
uint32 FLAG_OIL_PRESSURE_BELOW_NOMINAL = 65536   	# Under-pressure warning
uint32 FLAG_OIL_PRESSURE_ABOVE_NOMINAL = 131072  	# Over-pressure warning

uint32 FLAG_DEBRIS_SUPPORTED = 262144			# Debris warning. This flag is optional
uint32 FLAG_DEBRIS_DETECTED = 524288  			# Detection of debris warning
uint32 flags						# [@enum FLAG] Engine status flags bitmap

uint8 engine_load_percent				# [%] [@range 0, 127] Engine load estimate
uint32 engine_speed_rpm					# [rpm] [@range 0, inf] Engine speed
float32 spark_dwell_time_ms 				# [ms] [@range 0, inf] Spark dwell time
float32 atmospheric_pressure_kpa			# [kPa] [@range 0, inf] Atmospheric (barometric) pressure
float32 intake_manifold_pressure_kpa			# [kPa] [@range 0, inf] Engine intake manifold pressure
float32 intake_manifold_temperature			# [K] [@range 0, inf] Engine intake manifold temperature
float32 coolant_temperature				# [K] [@range 0, inf] Engine coolant temperature
float32 oil_pressure					# [kPa] [@range 0, inf] Oil pressure
float32 oil_temperature					# [K] [@range 0, inf] Oil temperature
float32 fuel_pressure					# [kPa] [@range 0, inf] Fuel pressure
float32 fuel_consumption_rate_cm3pm			# [cm^3/min] [@range 0, inf] Instant fuel consumption estimate
float32 estimated_consumed_fuel_volume_cm3		# [cm^3] [@range 0, inf] Estimate of the consumed fuel since the start of the engine
uint8 throttle_position_percent				# [%] [@range 0, 100] Throttle position
uint8 ecu_index						# [-] [@range 0, 255] The index of the publishing ECU


uint8 SPARK_PLUG_SINGLE         = 0
uint8 SPARK_PLUG_FIRST_ACTIVE   = 1
uint8 SPARK_PLUG_SECOND_ACTIVE  = 2
uint8 SPARK_PLUG_BOTH_ACTIVE    = 3
uint8 spark_plug_usage					# [@enum SPARK_PLUG] Spark plug activity report

float32 ignition_timing_deg				# [deg] [@range -inf, inf] Cylinder ignition timing, angular degrees of the crankshaft
float32 injection_time_ms				# [ms] [@range 0, inf] Fuel injection time
float32 cylinder_head_temperature			# [K] [@range 0, inf] Cylinder head temperature (CHT)
float32 exhaust_gas_temperature				# [K] [@range 0, inf] Exhaust gas temperature (EGT)
float32 lambda_coefficient				# [-] [@range 0, inf] Estimated lambda coefficient, dimensionless ratio

float32 pid_idle_rpm_integral				# [-] [@range -1, 1] Integral of the PID for the closed loop idle RPM controller
