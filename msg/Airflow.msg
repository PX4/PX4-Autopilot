# Airspeed data from sensors
#
# This is published by airspeed sensor drivers, CAN airspeed sensors, simulators.
# It is subscribed by the airspeed selector module, which validates the data from multiple sensors and passes on a single estimation to the EKF, controllers and telemetry providers.

uint64 timestamp                 # [us] Time since system start
uint64 timestamp_sample          # [us] Timestamp of the raw data
float32 u   			 # [m/s] Body-relative air relative velocity component / X direction
float32 v        		 # [m/s] Body-relative air relative velocity component / Y direction
float32 w               	 # [m/s] Body-relative air relative velocity component / Z direction
float32 windspeed_north	# [m/s] Wind component in north / X direction
float32 windspeed_east # [m/s] Wind component in east / Y direction
float32 windspeed_down # [m/s] Wind component in down / Z direction

# TOPICS airflow airflow_groundtruth
