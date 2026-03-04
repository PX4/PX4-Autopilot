---
pageClass: is-wide-page
---

# SensorGps (UORB message)

GPS position in WGS84 coordinates. the field 'timestamp' is for the position & velocity (microseconds).

**TOPICS:** sensor_gps vehicle_gps_position

## Fields

| 参数名                                                                           | 类型        | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                                                                                          |
| ----------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                     | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                   |
| timestamp_sample                                         | `uint64`  |                                                                  |            |                                                                                                                                                                                                                                             |
| device_id                                                | `uint32`  |                                                                  |            | unique device ID for the sensor that does not change between power cycles                                                                                                                                                                   |
| latitude_deg                                             | `float64` |                                                                  |            | Latitude in degrees, allows centimeter level RTK precision                                                                                                                                                                                  |
| longitude_deg                                            | `float64` |                                                                  |            | Longitude in degrees, allows centimeter level RTK precision                                                                                                                                                                                 |
| altitude_msl_m                      | `float64` |                                                                  |            | Altitude above MSL, meters                                                                                                                                                                                                                  |
| altitude_ellipsoid_m                | `float64` |                                                                  |            | Altitude above Ellipsoid, meters                                                                                                                                                                                                            |
| s_variance_m_s | `float32` |                                                                  |            | GPS speed accuracy estimate, (metres/sec)                                                                                                                                                                                |
| c_variance_rad                      | `float32` |                                                                  |            | GPS course accuracy estimate, (radians)                                                                                                                                                                                  |
| fix_type                                                 | `uint8`   |                                                                  |            | Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.                                                                                                      |
| eph                                                                           | `float32` |                                                                  |            | GPS horizontal position accuracy (metres)                                                                                                                                                                                |
| epv                                                                           | `float32` |                                                                  |            | GPS vertical position accuracy (metres)                                                                                                                                                                                  |
| hdop                                                                          | `float32` |                                                                  |            | Horizontal dilution of precision                                                                                                                                                                                                            |
| vdop                                                                          | `float32` |                                                                  |            | Vertical dilution of precision                                                                                                                                                                                                              |
| noise_per_ms                        | `int32`   |                                                                  |            | GPS noise per millisecond                                                                                                                                                                                                                   |
| automatic_gain_control              | `uint16`  |                                                                  |            | Automatic gain control monitor                                                                                                                                                                                                              |
| jamming_state                                            | `uint8`   |                                                                  |            | indicates whether jamming has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Mitigated, 3: Detected                                     |
| jamming_indicator                                        | `int32`   |                                                                  |            | indicates jamming is occurring                                                                                                                                                                                                              |
| spoofing_state                                           | `uint8`   |                                                                  |            | indicates whether spoofing has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Mitigated, 3: Detected                                    |
| authentication_state                                     | `uint8`   |                                                                  |            | GPS signal authentication state                                                                                                                                                                                                             |
| vel_m_s                             | `float32` |                                                                  |            | GPS ground speed, (metres/sec)                                                                                                                                                                                           |
| vel_n_m_s      | `float32` |                                                                  |            | GPS North velocity, (metres/sec)                                                                                                                                                                                         |
| vel_e_m_s      | `float32` |                                                                  |            | GPS East velocity, (metres/sec)                                                                                                                                                                                          |
| vel_d_m_s      | `float32` |                                                                  |            | GPS Down velocity, (metres/sec)                                                                                                                                                                                          |
| cog_rad                                                  | `float32` |                                                                  |            | Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)                                                                                       |
| vel_ned_valid                       | `bool`    |                                                                  |            | True if NED velocity is valid                                                                                                                                                                                                               |
| timestamp_time_relative             | `int32`   |                                                                  |            | timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)                                                                             |
| time_utc_usec                       | `uint64`  |                                                                  |            | Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0                                          |
| satellites_used                                          | `uint8`   |                                                                  |            | Number of satellites used                                                                                                                                                                                                                   |
| system_error                                             | `uint32`  |                                                                  |            | General errors with the connected GPS receiver                                                                                                                                                                                              |
| heading                                                                       | `float32` |                                                                  |            | heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI]) |
| heading_offset                                           | `float32` |                                                                  |            | heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])                                   |
| heading_accuracy                                         | `float32` |                                                                  |            | heading accuracy (rad, [0, 2PI])                                                                                                                                     |
| rtcm_injection_rate                 | `float32` |                                                                  |            | RTCM message injection rate Hz                                                                                                                                                                                                              |
| selected_rtcm_instance              | `uint8`   |                                                                  |            | uorb instance that is being used for RTCM corrections                                                                                                                                                                                       |
| rtcm_crc_failed                     | `bool`    |                                                                  |            | RTCM message CRC failure detected                                                                                                                                                                                                           |
| rtcm_msg_used                       | `uint8`   |                                                                  |            | Indicates if the RTCM message was used successfully by the receiver                                                                                                                                                                         |

## Constants

| 参数名                                                                                                                                                                 | 类型       | 值  | 描述                                                         |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | -- | ---------------------------------------------------------- |
| <a href="#FIX_TYPE_NONE"></a> FIX_TYPE_NONE                                                                               | `uint8`  | 1  | Value 0 is also valid to represent no fix. |
| <a href="#FIX_TYPE_2D"></a> FIX_TYPE_2D                                                                                   | `uint8`  | 2  |                                                            |
| <a href="#FIX_TYPE_3D"></a> FIX_TYPE_3D                                                                                   | `uint8`  | 3  |                                                            |
| <a href="#FIX_TYPE_RTCM_CODE_DIFFERENTIAL"></a> FIX_TYPE_RTCM_CODE_DIFFERENTIAL | `uint8`  | 4  |                                                            |
| <a href="#FIX_TYPE_RTK_FLOAT"></a> FIX_TYPE_RTK_FLOAT                                                | `uint8`  | 5  |                                                            |
| <a href="#FIX_TYPE_RTK_FIXED"></a> FIX_TYPE_RTK_FIXED                                                | `uint8`  | 6  |                                                            |
| <a href="#FIX_TYPE_EXTRAPOLATED"></a> FIX_TYPE_EXTRAPOLATED                                                               | `uint8`  | 8  |                                                            |
| <a href="#JAMMING_STATE_UNKNOWN"></a> JAMMING_STATE_UNKNOWN                                                               | `uint8`  | 0  | default                                                    |
| <a href="#JAMMING_STATE_OK"></a> JAMMING_STATE_OK                                                                         | `uint8`  | 1  |                                                            |
| <a href="#JAMMING_STATE_MITIGATED"></a> JAMMING_STATE_MITIGATED                                                           | `uint8`  | 2  |                                                            |
| <a href="#JAMMING_STATE_DETECTED"></a> JAMMING_STATE_DETECTED                                                             | `uint8`  | 3  |                                                            |
| <a href="#SPOOFING_STATE_UNKNOWN"></a> SPOOFING_STATE_UNKNOWN                                                             | `uint8`  | 0  | default                                                    |
| <a href="#SPOOFING_STATE_OK"></a> SPOOFING_STATE_OK                                                                       | `uint8`  | 1  |                                                            |
| <a href="#SPOOFING_STATE_MITIGATED"></a> SPOOFING_STATE_MITIGATED                                                         | `uint8`  | 2  |                                                            |
| <a href="#SPOOFING_STATE_DETECTED"></a> SPOOFING_STATE_DETECTED                                                           | `uint8`  | 3  |                                                            |
| <a href="#AUTHENTICATION_STATE_UNKNOWN"></a> AUTHENTICATION_STATE_UNKNOWN                                                 | `uint8`  | 0  | default                                                    |
| <a href="#AUTHENTICATION_STATE_INITIALIZING"></a> AUTHENTICATION_STATE_INITIALIZING                                       | `uint8`  | 1  |                                                            |
| <a href="#AUTHENTICATION_STATE_ERROR"></a> AUTHENTICATION_STATE_ERROR                                                     | `uint8`  | 2  |                                                            |
| <a href="#AUTHENTICATION_STATE_OK"></a> AUTHENTICATION_STATE_OK                                                           | `uint8`  | 3  |                                                            |
| <a href="#AUTHENTICATION_STATE_DISABLED"></a> AUTHENTICATION_STATE_DISABLED                                               | `uint8`  | 4  |                                                            |
| <a href="#SYSTEM_ERROR_OK"></a> SYSTEM_ERROR_OK                                                                           | `uint32` | 0  | default                                                    |
| <a href="#SYSTEM_ERROR_INCOMING_CORRECTIONS"></a> SYSTEM_ERROR_INCOMING_CORRECTIONS                  | `uint32` | 1  |                                                            |
| <a href="#SYSTEM_ERROR_CONFIGURATION"></a> SYSTEM_ERROR_CONFIGURATION                                                     | `uint32` | 2  |                                                            |
| <a href="#SYSTEM_ERROR_SOFTWARE"></a> SYSTEM_ERROR_SOFTWARE                                                               | `uint32` | 4  |                                                            |
| <a href="#SYSTEM_ERROR_ANTENNA"></a> SYSTEM_ERROR_ANTENNA                                                                 | `uint32` | 8  |                                                            |
| <a href="#SYSTEM_ERROR_EVENT_CONGESTION"></a> SYSTEM_ERROR_EVENT_CONGESTION                          | `uint32` | 16 |                                                            |
| <a href="#SYSTEM_ERROR_CPU_OVERLOAD"></a> SYSTEM_ERROR_CPU_OVERLOAD                                  | `uint32` | 32 |                                                            |
| <a href="#SYSTEM_ERROR_OUTPUT_CONGESTION"></a> SYSTEM_ERROR_OUTPUT_CONGESTION                        | `uint32` | 64 |                                                            |
| <a href="#RTCM_MSG_USED_UNKNOWN"></a> RTCM_MSG_USED_UNKNOWN                                          | `uint8`  | 0  |                                                            |
| <a href="#RTCM_MSG_USED_NOT_USED"></a> RTCM_MSG_USED_NOT_USED                   | `uint8`  | 1  |                                                            |
| <a href="#RTCM_MSG_USED_USED"></a> RTCM_MSG_USED_USED                                                | `uint8`  | 2  |                                                            |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGps.msg)

:::details
Click here to see original file

```c
# GPS position in WGS84 coordinates.
# the field 'timestamp' is for the position & velocity (microseconds)
uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id                # unique device ID for the sensor that does not change between power cycles

float64 latitude_deg		# Latitude in degrees, allows centimeter level RTK precision
float64 longitude_deg		# Longitude in degrees, allows centimeter level RTK precision
float64 altitude_msl_m		# Altitude above MSL, meters
float64 altitude_ellipsoid_m	# Altitude above Ellipsoid, meters

float32 s_variance_m_s		# GPS speed accuracy estimate, (metres/sec)
float32 c_variance_rad		# GPS course accuracy estimate, (radians)
uint8 FIX_TYPE_NONE                   = 1       # Value 0 is also valid to represent no fix.
uint8 FIX_TYPE_2D                     = 2
uint8 FIX_TYPE_3D                     = 3
uint8 FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4
uint8 FIX_TYPE_RTK_FLOAT              = 5
uint8 FIX_TYPE_RTK_FIXED              = 6
uint8 FIX_TYPE_EXTRAPOLATED           = 8
uint8 fix_type                  # Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

float32 eph			# GPS horizontal position accuracy (metres)
float32 epv			# GPS vertical position accuracy (metres)

float32 hdop			# Horizontal dilution of precision
float32 vdop			# Vertical dilution of precision

int32 noise_per_ms		# GPS noise per millisecond
uint16 automatic_gain_control   # Automatic gain control monitor

uint8 JAMMING_STATE_UNKNOWN   = 0 #default
uint8 JAMMING_STATE_OK        = 1
uint8 JAMMING_STATE_MITIGATED = 2
uint8 JAMMING_STATE_DETECTED  = 3
uint8 jamming_state	      # indicates whether jamming has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Mitigated, 3: Detected
int32 jamming_indicator	      # indicates jamming is occurring

uint8 SPOOFING_STATE_UNKNOWN   = 0 #default
uint8 SPOOFING_STATE_OK        = 1
uint8 SPOOFING_STATE_MITIGATED = 2
uint8 SPOOFING_STATE_DETECTED  = 3
uint8 spoofing_state	       # indicates whether spoofing has been detected or suspected by the receivers. O: Unknown, 1: OK, 2: Mitigated, 3: Detected

# Combined authentication state (e.g. Galileo OSNMA)
uint8 AUTHENTICATION_STATE_UNKNOWN      = 0 #default
uint8 AUTHENTICATION_STATE_INITIALIZING = 1
uint8 AUTHENTICATION_STATE_ERROR        = 2
uint8 AUTHENTICATION_STATE_OK           = 3
uint8 AUTHENTICATION_STATE_DISABLED     = 4
uint8 authentication_state              # GPS signal authentication state

float32 vel_m_s			# GPS ground speed, (metres/sec)
float32 vel_n_m_s		# GPS North velocity, (metres/sec)
float32 vel_e_m_s		# GPS East velocity, (metres/sec)
float32 vel_d_m_s		# GPS Down velocity, (metres/sec)
float32 cog_rad			# Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
bool vel_ned_valid		# True if NED velocity is valid

int32 timestamp_time_relative	# timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
uint64 time_utc_usec		# Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0

uint8 satellites_used		# Number of satellites used

uint32 SYSTEM_ERROR_OK                   = 0 #default
uint32 SYSTEM_ERROR_INCOMING_CORRECTIONS = 1
uint32 SYSTEM_ERROR_CONFIGURATION        = 2
uint32 SYSTEM_ERROR_SOFTWARE             = 4
uint32 SYSTEM_ERROR_ANTENNA              = 8
uint32 SYSTEM_ERROR_EVENT_CONGESTION     = 16
uint32 SYSTEM_ERROR_CPU_OVERLOAD         = 32
uint32 SYSTEM_ERROR_OUTPUT_CONGESTION    = 64
uint32 system_error                      # General errors with the connected GPS receiver

float32 heading			# heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
float32 heading_offset		# heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
float32 heading_accuracy	# heading accuracy (rad, [0, 2PI])

float32 rtcm_injection_rate	# RTCM message injection rate Hz
uint8 selected_rtcm_instance	# uorb instance that is being used for RTCM corrections

bool rtcm_crc_failed		# RTCM message CRC failure detected

uint8 RTCM_MSG_USED_UNKNOWN = 0
uint8 RTCM_MSG_USED_NOT_USED = 1
uint8 RTCM_MSG_USED_USED = 2
uint8 rtcm_msg_used		# Indicates if the RTCM message was used successfully by the receiver

# TOPICS sensor_gps vehicle_gps_position
```

:::
