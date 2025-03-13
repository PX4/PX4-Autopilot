# GPS position in WGS84 coordinates.
uint64 timestamp              # Time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id              # Unique device ID for the sensor that does not change between power cycles

float64 latitude              # Latitude in degrees (allows centimeter level RTK precision)
float64 longitude             # Longitude in degrees (allows centimeter level RTK precision)
float64 altitude_msl          # Altitude above MSL (meters) (as calculated by GNSS system)
float64 altitude_ellipsoid    # Altitude above Ellipsoid (WGS84)(meters)

float32 speed_accuracy        # GPS speed accuracy estimate (meters/sec)
float32 course_accuracy       # GPS course accuracy estimate (radians)
uint8 FIX_TYPE_NONE                   = 1
uint8 FIX_TYPE_2D                     = 2
uint8 FIX_TYPE_3D                     = 3
uint8 FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4
uint8 FIX_TYPE_RTK_FLOAT              = 5
uint8 FIX_TYPE_RTK_FIXED              = 6
uint8 FIX_TYPE_EXTRAPOLATED           = 8
uint8 fix_type                # Value 0 is also valid to represent no fix

float32 eph                   # GPS horizontal position accuracy (meters)
float32 epv                   # GPS vertical position accuracy (meters)

float32 hdop                  # Horizontal dilution of precision
float32 vdop                  # Vertical dilution of precision

int32 noise                   # GPS noise per millisecond
uint16 automatic_gain_control # Automatic gain control monitor

uint8 JAMMING_STATE_UNKNOWN  = 0
uint8 JAMMING_STATE_OK       = 1
uint8 JAMMING_STATE_WARNING  = 2
uint8 JAMMING_STATE_CRITICAL = 3
uint8 jamming_state           # Indicates whether jamming has been detected or suspected by the receivers (ublox only)
uint8 jamming_indicator       # Value from 0-255 where 0 indicates no jamming and 255 indicates strong CW jamming (ublox only)

uint8 SPOOFING_STATE_UNKNOWN   = 0
uint8 SPOOFING_STATE_NONE      = 1
uint8 SPOOFING_STATE_INDICATED = 2
uint8 SPOOFING_STATE_MULTIPLE  = 3
uint8 spoofing_state          # Indicates whether spoofing has been detected or suspected by the receivers

float32 ground_speed          # GPS ground speed (meters/sec)
float32 vel_north             # GPS North velocity (meters/sec)
float32 vel_east              # GPS East velocity (meters/sec)
float32 vel_down              # GPS Down velocity (meters/sec)
float32 course                # Course over ground (NOT heading, but direction of movement) in radians [-PI..PI]
bool vel_ned_valid            # True if NED velocity is valid

int32 timestamp_time_relative # timestamp + timestamp_time_relative = Time of the UTC timestamp since system start (microseconds)
uint64 time_utc_usec          # Timestamp in microseconds (UTC), might be unavailable right after cold start (indicated by value of 0)

uint8 satellites_used         # Number of satellites used

float32 heading               # Heading angle of XYZ body frame relative to NED (radians [-PI, PI]). Set to NaN if not available. (used for dual antenna GPS)
float32 heading_offset        # Heading offset of dual antenna array in body frame (radians [-PI, PI]). Set to NaN if not applicable
float32 heading_accuracy      # Heading accuracy (radians [0, 2PI])

float32 rtcm_injection_rate   # RTCM message injection rate (Hz)
uint8 selected_rtcm_instance  # uORB instance that is being used for RTCM corrections

bool rtcm_crc_failed          # RTCM message CRC failure detected

uint8 RTCM_MSG_USED_UNKNOWN = 0
uint8 RTCM_MSG_USED_NOT_USED = 1
uint8 RTCM_MSG_USED_USED = 2
uint8 rtcm_msg_used           # Indicates if the RTCM message was used successfully by the receiver

# TOPICS sensor_gps vehicle_gps_position
