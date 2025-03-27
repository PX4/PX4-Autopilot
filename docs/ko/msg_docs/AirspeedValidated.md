# AirspeedValidated (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/AirspeedValidated.msg)

```c
uint64 timestamp				# time since system start (microseconds)

float32 indicated_airspeed_m_s			# indicated airspeed in m/s (IAS), set to NAN if invalid
float32 calibrated_airspeed_m_s     		# calibrated airspeed in m/s (CAS, accounts for instrumentation errors), set to NAN if invalid
float32 true_airspeed_m_s			# true filtered airspeed in m/s (TAS), set to NAN if invalid

float32 calibrated_ground_minus_wind_m_s 	# CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid
float32 true_ground_minus_wind_m_s 		# TAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid

bool airspeed_sensor_measurement_valid 		# True if data from at least one airspeed sensor is declared valid.

int8 selected_airspeed_index 			# 1-3: airspeed sensor index, 0: groundspeed-windspeed, -1: airspeed invalid

float32 airspeed_derivative_filtered		# filtered indicated airspeed derivative [m/s/s]
float32 throttle_filtered			# filtered fixed-wing throttle [-]
float32 pitch_filtered				# filtered pitch [rad]

```
