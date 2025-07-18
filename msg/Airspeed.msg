# Airspeed data from sensors
#
# This is published by airspeed sensor drivers, CAN airspeed sensors, simulators.
# It is subscribed by EKF for estimation,
# along with a number of other modules such as the one for HOTT telemetry.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data
float32 indicated_airspeed_m_s # [m/s] Indicated airspeed
float32 true_airspeed_m_s # [m/s] True filtered airspeed
float32 confidence # [@range 0,1] Confidence value for this sensor
