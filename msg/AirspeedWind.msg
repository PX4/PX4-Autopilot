# Wind estimate (from airspeed_selector)
#
# Contains wind estimation and airspeed innovation information estimated by the WindEstimator
# in the airspeed selector module.
#
# This message is published by the airspeed selector for debugging purposes, and is not
# subscribed to by any other modules.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

float32 windspeed_north	# [m/s] Wind component in north / X direction
float32 windspeed_east # [m/s] Wind component in east / Y direction

float32 variance_north # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in north / X direction
float32 variance_east # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in east / Y direction

float32 tas_innov # [m/s] True airspeed innovation
float32 tas_innov_var # [m/s] True airspeed innovation variance

float32 tas_scale_raw # Estimated true airspeed scale factor (not validated)
float32 tas_scale_raw_var # True airspeed scale factor variance

float32 tas_scale_validated # Estimated true airspeed scale factor after validation

float32 beta_innov # [rad] Sideslip measurement innovation
float32 beta_innov_var # [rad^2] Sideslip measurement innovation variance

uint8 source # source of wind estimate

uint8 SOURCE_AS_BETA_ONLY = 0 # Wind estimate only based on synthetic sideslip fusion
uint8 SOURCE_AS_SENSOR_1 = 1 # Combined synthetic sideslip and airspeed fusion (data from first airspeed sensor)
uint8 SOURCE_AS_SENSOR_2 = 2 # Combined synthetic sideslip and airspeed fusion (data from second airspeed sensor)
uint8 SOURCE_AS_SENSOR_3 = 3 # Combined synthetic sideslip and airspeed fusion (data from third airspeed sensor)
