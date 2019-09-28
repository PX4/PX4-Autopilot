uint64 timestamp			# time since system start (microseconds)

float32 indicated_airspeed_m_s		# indicated airspeed in m/s (IAS), set to NAN if invalid
float32 equivalent_airspeed_m_s     # equivalent airspeed in m/s (accounts for instrumentation errors) (EAS), set to NAN if invalid
float32 true_airspeed_m_s			# true filtered airspeed in m/s (TAS), set to NAN if invalid

float32 true_ground_minus_wind_m_s 		# TAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid
float32 indicated_ground_minus_wind_m_s # IAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid

int8 selected_airspeed_index # 0-2: airspeed sensor index, -1: groundspeed-windspeed, -2: airspeed invalid
