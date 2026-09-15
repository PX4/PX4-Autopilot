# GNSS receiver check status
#
# Result of the quality checks run by the sensors module on a GNSS receiver output (fix type,
# satellite count, DOP, accuracy, drift, speed, spoofing and jamming).
# Status topic with latest-wins semantics, like vehicle_imu_status: consumers copy() the most
# recent message. It is not synchronized with an individual sensor_gps sample.

uint64 timestamp # [us] Time since system start

uint32 device_id # [-] Unique device ID for the sensor that does not change between power cycles

float32 position_drift_rate_horizontal_m_s # [m/s] Horizontal position drift rate magnitude
float32 position_drift_rate_vertical_m_s # [m/s] Vertical position drift rate magnitude
float32 filtered_horizontal_speed_m_s # [m/s] Filtered horizontal velocity magnitude

uint16 flags # [@enum GNSS] Bitmask of failed checks, GNSS_OK (0) when all checks pass
uint16 GNSS_OK = 0 # No failures
uint16 GNSS_NSATS_FAIL = 1 # (1 << 0)
uint16 GNSS_PDOP_FAIL = 2 # (1 << 1)
uint16 GNSS_HACC_FAIL = 4 # (1 << 2)
uint16 GNSS_VACC_FAIL = 8 # (1 << 3)
uint16 GNSS_SACC_FAIL = 16 # (1 << 4)
uint16 GNSS_HDRIFT_FAIL = 32 # (1 << 5)
uint16 GNSS_VDRIFT_FAIL = 64 # (1 << 6)
uint16 GNSS_HSPEED_FAIL = 128 # (1 << 7)
uint16 GNSS_VSPEED_FAIL = 256 # (1 << 8)
uint16 GNSS_SPOOFED_FAIL = 512 # (1 << 9)
uint16 GNSS_FIX_FAIL = 1024 # (1 << 10)
uint16 GNSS_JAMMED_FAIL = 2048 # (1 << 11)

uint16 enabled_checks # [@enum GNSS] Bitmask of the checks required by GNSS_CHECK; flags is masked with it

bool checks_passed # All checks pass and have done so for the minimum health time

# TOPICS sensor_gps_status vehicle_gps_position_status
