# GNSS health flag indicators
#
# Result of the quality checks run on a GNSS receiver output (fix type, satellite count, DOP,
# accuracy, drift, speed, spoofing and jamming). The flags bitmask reports which required checks currently fail.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

uint32 device_id # [-] Unique device ID for the sensor that does not change between power cycles

float32 position_drift_rate_horizontal_m_s # [m/s] Horizontal position drift rate magnitude
float32 position_drift_rate_vertical_m_s # [m/s] Vertical position drift rate magnitude
float32 filtered_horizontal_speed_m_s # [m/s] Filtered horizontal velocity magnitude

uint16 flags # [@enum GNSS] Bitmask of failed checks, GNSS_OK (0) when all checks pass
uint16 GNSS_OK = 0 # No failures
uint16 GNSS_FIX_FAIL = 1 # (1 << 0)
uint16 GNSS_NSATS_FAIL = 2 # (1 << 1)
uint16 GNSS_PDOP_FAIL = 4 # (1 << 2)
uint16 GNSS_VACC_FAIL = 16 # (1 << 4)
uint16 GNSS_SACC_FAIL = 32 # (1 << 5)
uint16 GNSS_HDRIFT_FAIL = 64 # (1 << 6)
uint16 GNSS_VDRIFT_FAIL = 128 # (1 << 7)
uint16 GNSS_HSPEED_FAIL = 256 # (1 << 8)
uint16 GNSS_VSPEED_FAIL = 512 # (1 << 9)
uint16 GNSS_SPOOFED_FAIL = 1024 # (1 << 10)
uint16 GNSS_JAMMED_FAIL = 2048 # (1 << 11)

uint64 time_last_pass # [us] Timestamp of last passing sample
uint64 time_last_fail # [us] Timestamp of last failing sample
bool checks_passed # All checks pass and have done so for the minimum health time
bool initial_checks_passed # Strict initial checks passed, simplified checks run afterwards

# TOPICS sensor_gps_checks vehicle_gps_position_checks
