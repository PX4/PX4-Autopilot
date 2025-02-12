uint64 timestamp				# time since system start (microseconds)

# General vehicle data
uint16 format_version				# format version, to track in case record format needs to be updated
uint32 boot_cycles          			# total number of times the system has booted
uint64 init_timestamp_utc_s			# timestamp for when this module first started running on the vehicle
uint32 on_duration_s          			# total on time in seconds
uint32 flight_duration_s      			# total flight time in seconds
uint32 arm_cycles         			# total number of arm cycles

# Flight history
uint32[3] altitude_duration_s 			# duration vehicle spent at altitude (meters) thresholds [medium (1200-2400), high (2400-3600), very high (> 3600)]
uint32[2] ambient_temperature_duration_s 	# duration vehicle spent at ambient temperature (celsius) thresholds [cold ( < -20), hot (> 50)]

# Structural history
uint64[3] vibration_severity_duration_s 	# duration vehicle spent at vibration severity thresholds [low (0-10), medium (10-20), high (20-30)]

# PER MOTOR DATA
# Rotation data
uint64[4] revs_per_motor   				# total revolutions for each motor
uint32[4] med_rpm_duration_s    			# duration spent at medium RPM (4000-6000) per motor
uint32[4] high_rpm_duration_s    			# duration spent at high RPM (>6000) per motor

# Prop dynamics history
# prop flop load for a motor is defined as (angular velocity of propeller (rad/s) * angular velocity magnitude of drone in x-y (rad/s))
uint32[4] med_prop_flop_duration_s 			# total duration of moderate severity prop flop (40-80) per motor in seconds
uint32[4] high_prop_flop_duration_s 			# total duration of high severity prop flop (> 80) per motor in seconds
# asymmetric lift load for a motor is defined as (angular velocity of propeller (rad/s) * air speed magnitude (m/s))
uint32[4] med_prop_asymmetric_lift_duration_s 		# total duration of moderate severity prop asymmetric lift (1500-2500) per motor in seconds
uint32[4] high_prop_asymmetric_lift_duration_s 		# total duration of high severity prop asymmetric lift (> 2500) per motor in seconds

# ESC electrical history
float32[4] esc_voltage_max 			# maximum ESC voltage per motor
float32[4] esc_voltage_min 			# minimum ESC voltage per motor
float32[4] esc_current_max 			# maximum ESC current per motor
float32[4] esc_current_min 			# minimum ESC current per motor
uint16[4] esc_current_warn_events 		# number of unique boot instances with ESC current warning event per motor
uint16[4] esc_current_max_events 		# number of unique boot instances with ESC current max event per motor
uint32[4] esc_current_warn_duration_s 		# total duration of ESC current warning event per motor in seconds
uint32[4] esc_current_max_duration_s 		# total duration of ESC current max event per motor in seconds

# ESC temperature history
float32[4] esc_temp_max 			# maximum ESC temperature per motor
uint16[4] esc_temp_warn_events     		# number of unique boot instances with ESC temperature warning event per motor
uint16[4] esc_temp_max_events      		# number of unique boot instances with ESC temperature max event per motor
uint32[4] esc_temp_warn_duration_s 		# total duration of ESC temperature warning event per motor in seconds
uint32[4] esc_temp_max_duration_s  		# total duration of ESC temperature max event per motor in seconds

# Motor temperature history
float32[4] motor_temp_max 			# maximum motor temperature per motor
uint16[4] motor_temp_warn_events     		# number of unique boot instances with motor temperature warning event per motor
uint16[4] motor_temp_max_events      		# number of unique boot instances with motor temperature max event per motor
uint32[4] motor_temp_warn_duration_s 		# total duration of motor temperature warning event per motor in seconds
uint32[4] motor_temp_max_duration_s  		# total duration of motor temperature max event per motor in seconds
