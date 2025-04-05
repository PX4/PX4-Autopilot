uint32 MESSAGE_VERSION = 1

uint64 timestamp				# time since system start (microseconds)

float32 indicated_airspeed_m_s			# [m/s] Indicated airspeed (IAS), set to NAN if invalid
float32 calibrated_airspeed_m_s     		# [m/s] Calibrated airspeed (CAS), set to NAN if invalid
float32 true_airspeed_m_s			# [m/s] True airspeed (TAS), set to NAN if invalid

int8 airspeed_source				# Source of currently published airspeed values
int8 DISABLED = -1
int8 GROUND_MINUS_WIND = 0
int8 SENSOR_1 = 1
int8 SENSOR_2 = 2
int8 SENSOR_3 = 3
int8 SYNTHETIC = 4

# debug states
float32 calibrated_ground_minus_wind_m_s 	# CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid
float32 calibraded_airspeed_synth_m_s		# synthetic airspeed in m/s, set to NAN if invalid
float32 airspeed_derivative_filtered		# filtered indicated airspeed derivative [m/s/s]
float32 throttle_filtered			# filtered fixed-wing throttle [-]
float32 pitch_filtered				# filtered pitch [rad]
