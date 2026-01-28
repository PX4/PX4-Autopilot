# Wind estimate (from EKF2)
#
# Contains the system-wide estimate of horizontal wind velocity and its variance.
# Published by the navigation filter (EKF2) for use by other flight modules and libraries.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

float32 windspeed_north # [m/s] Wind component in north / X direction
float32 windspeed_east # [m/s] Wind component in east / Y direction

float32 variance_north # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in north / X direction
float32 variance_east # [(m/s)^2] [@invalid 0 if not estimated] Wind estimate error variance in east / Y direction

float32 tas_innov # [m/s] True airspeed innovation
float32 tas_innov_var # [(m/s)^2] True airspeed innovation variance

float32 beta_innov # [rad] Sideslip measurement innovation
float32 beta_innov_var # [rad^2] Sideslip measurement innovation variance

# TOPICS wind estimator_wind
