uint64 timestamp			# time since system start (microseconds)

float32 indicated_airspeed_m_s		# indicated airspeed in m/s (IAS), set to NAN if invalid (if equal to equivalent_ground_minus_wind_m_s, then airspeed_sensor_data_valid = false)
float32 equivalent_airspeed_m_s     # equivalent airspeed in m/s (accounts for instrumentation errors) (EAS), set to NAN if invalid (if equal to equivalent_ground_minus_wind_m_s, then airspeed_sensor_data_valid = false)
float32 true_airspeed_m_s			# true filtered airspeed in m/s (TAS), set to NAN if invalid (if equal to true_ground_minus_wind_m_s, then airspeed_sensor_data_valid = false)

float32 equivalent_ground_minus_wind_m_s # EAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid
float32 true_ground_minus_wind_m_s 		# TAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid

bool airspeed_sensor_measurement_valid # True if data from at least one airspeed sensor is declared valid. If invalid, IAS, EAS and TAS are approximated by ground-windspeed.

int8 selected_airspeed_index # 0-2: airspeed sensor index, -1: groundspeed-windspeed, -2: airspeed invalid
