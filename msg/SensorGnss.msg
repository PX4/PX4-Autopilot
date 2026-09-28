# GNSS receiver report
#
# One instance per receiver, published by its driver: the receiver's own solution, unmodified.
# The sensors module publishes the selected receiver on vehicle_gnss.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Measurement time if the driver knows it, else 0. vehicle_gnss carries the corrected value

uint32 device_id # [-] Unique device ID for the sensor that does not change between power cycles

float64 latitude           # [deg] Latitude, allows centimeter level RTK precision
float64 longitude          # [deg] Longitude, allows centimeter level RTK precision
float64 altitude_msl       # [m] Altitude above MSL, from the receiver's own geoid model
float64 altitude_ellipsoid # [m] Altitude above the ellipsoid

float32 speed_accuracy  # [m/s] Speed accuracy estimate
float32 course_accuracy # [rad] Course accuracy estimate

uint8 fix_type                             # [@enum FIX_TYPE] Value 0 is also valid to represent no fix
uint8 FIX_TYPE_NONE                   = 1
uint8 FIX_TYPE_2D                     = 2
uint8 FIX_TYPE_3D                     = 3
uint8 FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4
uint8 FIX_TYPE_RTK_FLOAT              = 5
uint8 FIX_TYPE_RTK_FIXED              = 6
uint8 FIX_TYPE_EXTRAPOLATED           = 8

float32 eph  # [m] Horizontal position accuracy
float32 epv  # [m] Vertical position accuracy
float32 hdop # [-] Horizontal dilution of precision
float32 vdop # [-] Vertical dilution of precision

int32 noise                   # [-] Noise level per millisecond
uint16 automatic_gain_control # [-] Automatic gain control monitor

uint8 jamming_state           # [@enum JAMMING_STATE]
uint8 JAMMING_STATE_UNKNOWN   = 0
uint8 JAMMING_STATE_OK        = 1
uint8 JAMMING_STATE_MITIGATED = 2
uint8 JAMMING_STATE_DETECTED  = 3
int32 jamming_indicator       # [-] Jamming indicator

uint8 spoofing_state           # [@enum SPOOFING_STATE]
uint8 SPOOFING_STATE_UNKNOWN   = 0
uint8 SPOOFING_STATE_OK        = 1
uint8 SPOOFING_STATE_MITIGATED = 2
uint8 SPOOFING_STATE_DETECTED  = 3

uint8 authentication_state              # [@enum AUTHENTICATION_STATE] Combined signal authentication state (e.g. Galileo OSNMA)
uint8 AUTHENTICATION_STATE_UNKNOWN      = 0
uint8 AUTHENTICATION_STATE_INITIALIZING = 1
uint8 AUTHENTICATION_STATE_ERROR        = 2
uint8 AUTHENTICATION_STATE_OK           = 3
uint8 AUTHENTICATION_STATE_DISABLED     = 4

float32 ground_speed # [m/s] Ground speed
float32 vel_north    # [m/s] North velocity
float32 vel_east     # [m/s] East velocity
float32 vel_down     # [m/s] Down velocity
float32 course       # [rad] [@range -PI, PI] Course over ground, not heading
bool vel_ned_valid   # NED velocity is valid

int32 timestamp_time_relative # [us] timestamp + timestamp_time_relative = time of the UTC timestamp since system start
uint64 time_utc_usec          # [us] UTC time from the receiver, 0 until known

uint8 satellites_used # [-] Satellites used in the solution

uint32 system_error                      # [@enum SYSTEM_ERROR] Bitmask of receiver errors
uint32 SYSTEM_ERROR_OK                   = 0
uint32 SYSTEM_ERROR_INCOMING_CORRECTIONS = 1
uint32 SYSTEM_ERROR_CONFIGURATION        = 2
uint32 SYSTEM_ERROR_SOFTWARE             = 4
uint32 SYSTEM_ERROR_ANTENNA              = 8
uint32 SYSTEM_ERROR_EVENT_CONGESTION     = 16
uint32 SYSTEM_ERROR_CPU_OVERLOAD         = 32
uint32 SYSTEM_ERROR_OUTPUT_CONGESTION    = 64

float32 rtcm_injection_rate  # [Hz] Correction injection rate
uint8 selected_rtcm_instance # [-] uORB instance used for corrections

uint8 corrections_protocol         # [@enum CORRECTIONS_PROTOCOL] Protocol of the last correction message the receiver parsed
uint8 CORRECTIONS_PROTOCOL_UNKNOWN = 0
uint8 CORRECTIONS_PROTOCOL_RTCM3   = 1
uint8 CORRECTIONS_PROTOCOL_SPARTN  = 2
uint8 CORRECTIONS_PROTOCOL_HAS     = 3 # Galileo High Accuracy Service, received on E6
uint8 CORRECTIONS_PROTOCOL_PMP     = 4 # SPARTN over L-band (u-blox NEO-D9S)
uint8 CORRECTIONS_PROTOCOL_QZSS_L6 = 5 # QZSS CLAS
bool corrections_crc_failed        # Last correction message failed its CRC or content check

uint8 corrections_msg_used          # [@enum CORRECTIONS_MSG_USED] Whether the receiver used the last correction message
uint8 CORRECTIONS_MSG_USED_UNKNOWN  = 0
uint8 CORRECTIONS_MSG_USED_NOT_USED = 1
uint8 CORRECTIONS_MSG_USED_USED     = 2

# TOPICS sensor_gnss
