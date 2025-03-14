uint64 timestamp				# time since system start (microseconds)

float32 indicated_airspeed_m_s			# indicated airspeed in m/s (IAS), set to NAN if invalid
float32 calibrated_airspeed_m_s     		# calibrated airspeed in m/s (CAS, accounts for instrumentation errors), set to NAN if invalid
float32 true_airspeed_m_s			# true filtered airspeed in m/s (TAS), set to NAN if invalid

float32 calibrated_ground_minus_wind_m_s 	# CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid

float32 cal_airspeed_synth_m_s			# synthetic airspeed in m/s (CAS, accounts for instrumentation errors), set to NAN if invalid

int8 airspeed_source				# -1: Disabled, 0: Ground-Relative, 1: Pitot Tube 1, 2: Pitot Tube 2, 3: Pitot Tube 3, 4: Synthetic

float32 airspeed_derivative_filtered		# filtered indicated airspeed derivative [m/s/s]
float32 throttle_filtered			# filtered fixed-wing throttle [-]
float32 pitch_filtered				# filtered pitch [rad]
