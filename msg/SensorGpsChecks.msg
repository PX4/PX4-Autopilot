# GNSS health flag indicators
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id                # unique device ID for the sensor that does not change between power cycles

float32 position_drift_rate_horizontal_m_s # Horizontal position rate magnitude (m/s)
float32 position_drift_rate_vertical_m_s   # Vertical position rate magnitude (m/s)
float32 filtered_horizontal_speed_m_s      # Filtered horizontal velocity magnitude (m/s)

uint32 GNSS_OK             = 0        # default, no failures
uint32 GNSS_FIX_FAIL       = 1        # 1 << 0
uint32 GNSS_NSATS_FAIL     = 2        # 1 << 1
uint32 GNSS_HDOP_FAIL      = 4        # 1 << 2
uint32 GNSS_VDOP_FAIL      = 8        # 1 << 3
uint32 GNSS_HACC_FAIL      = 16       # 1 << 4
uint32 GNSS_VACC_FAIL      = 32       # 1 << 5
uint32 GNSS_SACC_FAIL      = 64       # 1 << 6
uint32 GNSS_HDRIFT_FAIL    = 128      # 1 << 7
uint32 GNSS_VDRIFT_FAIL    = 256      # 1 << 8
uint32 GNSS_HSPEED_FAIL    = 512      # 1 << 9
uint32 GNSS_VSPEED_FAIL    = 1024     # 1 << 10
uint32 GNSS_SPOOFED_FAIL   = 2048     # 1 << 11
uint32 GNSS_JAMMED_FAIL    = 4096     # 1 << 12

uint32 flags                            # OR-combination of the above GNSS_*_FAIL bits, 0 = all checks passed

bool checks_passed
bool initial_checks_passed

# TOPICS sensor_gps_checks vehicle_gps_position_checks
