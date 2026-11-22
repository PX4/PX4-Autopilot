---
pageClass: is-wide-page
---

# SensorGnss (UORB message)

GNSS receiver report, position in WGS84.

One instance per receiver, published by its driver: the receiver's own solution, unmodified.
The sensors module publishes the selected receiver on vehicle_gnss.

**TOPICS:** sensor_gnss

## Fields

| Name                                                            | Type      | Unit [Frame] | Range/Enum                                    | Description                                                                                                     |
| --------------------------------------------------------------- | --------- | ------------ | --------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                             | `uint64`  | us           |                                               | Time since system start                                                                                         |
| <a id="fld_timestamp_sample"></a>timestamp_sample               | `uint64`  | us           |                                               | Measurement time if the driver knows it, else 0 or equal to timestamp. vehicle_gnss carries the corrected value |
| <a id="fld_device_id"></a>device_id                             | `uint32`  |              |                                               | Unique device ID for the sensor that does not change between power cycles                                       |
| <a id="fld_latitude"></a>latitude                               | `float64` | deg          |                                               | Latitude, allows centimeter level RTK precision                                                                 |
| <a id="fld_longitude"></a>longitude                             | `float64` | deg          |                                               | Longitude, allows centimeter level RTK precision                                                                |
| <a id="fld_altitude_msl"></a>altitude_msl                       | `float64` | m            |                                               | Altitude above MSL, from the receiver's own geoid model                                                         |
| <a id="fld_altitude_ellipsoid"></a>altitude_ellipsoid           | `float64` | m            |                                               | Altitude above the WGS84 ellipsoid                                                                              |
| <a id="fld_speed_accuracy"></a>speed_accuracy                   | `float32` | m/s          |                                               | Speed accuracy estimate                                                                                         |
| <a id="fld_course_accuracy"></a>course_accuracy                 | `float32` | rad          |                                               | Course accuracy estimate                                                                                        |
| <a id="fld_fix_type"></a>fix_type                               | `uint8`   |              | [FIX_TYPE](#FIX_TYPE)                         | Value 0 is also valid to represent no fix                                                                       |
| <a id="fld_eph"></a>eph                                         | `float32` | m            |                                               | Horizontal position accuracy                                                                                    |
| <a id="fld_epv"></a>epv                                         | `float32` | m            |                                               | Vertical position accuracy                                                                                      |
| <a id="fld_hdop"></a>hdop                                       | `float32` |              |                                               | Horizontal dilution of precision                                                                                |
| <a id="fld_vdop"></a>vdop                                       | `float32` |              |                                               | Vertical dilution of precision                                                                                  |
| <a id="fld_noise"></a>noise                                     | `int32`   |              |                                               | Noise level per millisecond                                                                                     |
| <a id="fld_automatic_gain_control"></a>automatic_gain_control   | `uint16`  |              |                                               | Automatic gain control monitor                                                                                  |
| <a id="fld_jamming_state"></a>jamming_state                     | `uint8`   |              | [JAMMING_STATE](#JAMMING_STATE)               |
| <a id="fld_jamming_indicator"></a>jamming_indicator             | `int32`   |              |                                               | Jamming indicator                                                                                               |
| <a id="fld_spoofing_state"></a>spoofing_state                   | `uint8`   |              | [SPOOFING_STATE](#SPOOFING_STATE)             |
| <a id="fld_authentication_state"></a>authentication_state       | `uint8`   |              | [AUTHENTICATION_STATE](#AUTHENTICATION_STATE) | Combined signal authentication state (e.g. Galileo OSNMA)                                                       |
| <a id="fld_ground_speed"></a>ground_speed                       | `float32` | m/s          |                                               | Ground speed                                                                                                    |
| <a id="fld_vel_north"></a>vel_north                             | `float32` | m/s          |                                               | North velocity                                                                                                  |
| <a id="fld_vel_east"></a>vel_east                               | `float32` | m/s          |                                               | East velocity                                                                                                   |
| <a id="fld_vel_down"></a>vel_down                               | `float32` | m/s          |                                               | Down velocity                                                                                                   |
| <a id="fld_course"></a>course                                   | `float32` | rad          | [-PI : PI]                                    | Course over ground, not heading                                                                                 |
| <a id="fld_vel_ned_valid"></a>vel_ned_valid                     | `bool`    |              |                                               | NED velocity is valid                                                                                           |
| <a id="fld_timestamp_time_relative"></a>timestamp_time_relative | `int32`   | us           |                                               | timestamp + timestamp_time_relative = time of the UTC timestamp since system start                              |
| <a id="fld_time_utc_usec"></a>time_utc_usec                     | `uint64`  | us           |                                               | UTC time from the receiver, 0 until known                                                                       |
| <a id="fld_satellites_used"></a>satellites_used                 | `uint8`   |              |                                               | Satellites used in the solution                                                                                 |
| <a id="fld_system_error"></a>system_error                       | `uint32`  |              | [SYSTEM_ERROR](#SYSTEM_ERROR)                 | Bitmask of receiver errors                                                                                      |
| <a id="fld_rtcm_injection_rate"></a>rtcm_injection_rate         | `float32` | Hz           |                                               | Correction injection rate                                                                                       |
| <a id="fld_selected_rtcm_instance"></a>selected_rtcm_instance   | `uint8`   |              |                                               | uORB instance used for corrections                                                                              |
| <a id="fld_corrections_protocol"></a>corrections_protocol       | `uint8`   |              | [CORRECTIONS_PROTOCOL](#CORRECTIONS_PROTOCOL) | Protocol of the last correction message the receiver parsed                                                     |
| <a id="fld_corrections_crc_failed"></a>corrections_crc_failed   | `bool`    |              |                                               | Last correction message failed its CRC or content check                                                         |
| <a id="fld_corrections_msg_used"></a>corrections_msg_used       | `uint8`   |              | [CORRECTIONS_MSG_USED](#CORRECTIONS_MSG_USED) | Whether the receiver used the last correction message                                                           |

## Enums

### FIX_TYPE {#FIX_TYPE}

Used in field(s): [fix_type](#fld_fix_type)

| Name                                                                          | Type    | Value | Description |
| ----------------------------------------------------------------------------- | ------- | ----- | ----------- |
| <a id="#FIX_TYPE_NONE"></a> FIX_TYPE_NONE                                     | `uint8` | 1     |
| <a id="#FIX_TYPE_2D"></a> FIX_TYPE_2D                                         | `uint8` | 2     |
| <a id="#FIX_TYPE_3D"></a> FIX_TYPE_3D                                         | `uint8` | 3     |
| <a id="#FIX_TYPE_RTCM_CODE_DIFFERENTIAL"></a> FIX_TYPE_RTCM_CODE_DIFFERENTIAL | `uint8` | 4     |
| <a id="#FIX_TYPE_RTK_FLOAT"></a> FIX_TYPE_RTK_FLOAT                           | `uint8` | 5     |
| <a id="#FIX_TYPE_RTK_FIXED"></a> FIX_TYPE_RTK_FIXED                           | `uint8` | 6     |
| <a id="#FIX_TYPE_EXTRAPOLATED"></a> FIX_TYPE_EXTRAPOLATED                     | `uint8` | 8     |

### JAMMING_STATE {#JAMMING_STATE}

Used in field(s): [jamming_state](#fld_jamming_state)

| Name                                                          | Type    | Value | Description |
| ------------------------------------------------------------- | ------- | ----- | ----------- |
| <a id="#JAMMING_STATE_UNKNOWN"></a> JAMMING_STATE_UNKNOWN     | `uint8` | 0     |
| <a id="#JAMMING_STATE_OK"></a> JAMMING_STATE_OK               | `uint8` | 1     |
| <a id="#JAMMING_STATE_MITIGATED"></a> JAMMING_STATE_MITIGATED | `uint8` | 2     |
| <a id="#JAMMING_STATE_DETECTED"></a> JAMMING_STATE_DETECTED   | `uint8` | 3     |

### SPOOFING_STATE {#SPOOFING_STATE}

Used in field(s): [spoofing_state](#fld_spoofing_state)

| Name                                                            | Type    | Value | Description |
| --------------------------------------------------------------- | ------- | ----- | ----------- |
| <a id="#SPOOFING_STATE_UNKNOWN"></a> SPOOFING_STATE_UNKNOWN     | `uint8` | 0     |
| <a id="#SPOOFING_STATE_OK"></a> SPOOFING_STATE_OK               | `uint8` | 1     |
| <a id="#SPOOFING_STATE_MITIGATED"></a> SPOOFING_STATE_MITIGATED | `uint8` | 2     |
| <a id="#SPOOFING_STATE_DETECTED"></a> SPOOFING_STATE_DETECTED   | `uint8` | 3     |

### AUTHENTICATION_STATE {#AUTHENTICATION_STATE}

Used in field(s): [authentication_state](#fld_authentication_state)

| Name                                                                              | Type    | Value | Description |
| --------------------------------------------------------------------------------- | ------- | ----- | ----------- |
| <a id="#AUTHENTICATION_STATE_UNKNOWN"></a> AUTHENTICATION_STATE_UNKNOWN           | `uint8` | 0     |
| <a id="#AUTHENTICATION_STATE_INITIALIZING"></a> AUTHENTICATION_STATE_INITIALIZING | `uint8` | 1     |
| <a id="#AUTHENTICATION_STATE_ERROR"></a> AUTHENTICATION_STATE_ERROR               | `uint8` | 2     |
| <a id="#AUTHENTICATION_STATE_OK"></a> AUTHENTICATION_STATE_OK                     | `uint8` | 3     |
| <a id="#AUTHENTICATION_STATE_DISABLED"></a> AUTHENTICATION_STATE_DISABLED         | `uint8` | 4     |

### SYSTEM_ERROR {#SYSTEM_ERROR}

Used in field(s): [system_error](#fld_system_error)

| Name                                                                              | Type     | Value | Description |
| --------------------------------------------------------------------------------- | -------- | ----- | ----------- |
| <a id="#SYSTEM_ERROR_OK"></a> SYSTEM_ERROR_OK                                     | `uint32` | 0     |
| <a id="#SYSTEM_ERROR_INCOMING_CORRECTIONS"></a> SYSTEM_ERROR_INCOMING_CORRECTIONS | `uint32` | 1     |
| <a id="#SYSTEM_ERROR_CONFIGURATION"></a> SYSTEM_ERROR_CONFIGURATION               | `uint32` | 2     |
| <a id="#SYSTEM_ERROR_SOFTWARE"></a> SYSTEM_ERROR_SOFTWARE                         | `uint32` | 4     |
| <a id="#SYSTEM_ERROR_ANTENNA"></a> SYSTEM_ERROR_ANTENNA                           | `uint32` | 8     |
| <a id="#SYSTEM_ERROR_EVENT_CONGESTION"></a> SYSTEM_ERROR_EVENT_CONGESTION         | `uint32` | 16    |
| <a id="#SYSTEM_ERROR_CPU_OVERLOAD"></a> SYSTEM_ERROR_CPU_OVERLOAD                 | `uint32` | 32    |
| <a id="#SYSTEM_ERROR_OUTPUT_CONGESTION"></a> SYSTEM_ERROR_OUTPUT_CONGESTION       | `uint32` | 64    |

### CORRECTIONS_PROTOCOL {#CORRECTIONS_PROTOCOL}

Used in field(s): [corrections_protocol](#fld_corrections_protocol)

| Name                                                                    | Type    | Value | Description                                   |
| ----------------------------------------------------------------------- | ------- | ----- | --------------------------------------------- |
| <a id="#CORRECTIONS_PROTOCOL_UNKNOWN"></a> CORRECTIONS_PROTOCOL_UNKNOWN | `uint8` | 0     |
| <a id="#CORRECTIONS_PROTOCOL_RTCM3"></a> CORRECTIONS_PROTOCOL_RTCM3     | `uint8` | 1     |
| <a id="#CORRECTIONS_PROTOCOL_SPARTN"></a> CORRECTIONS_PROTOCOL_SPARTN   | `uint8` | 2     |
| <a id="#CORRECTIONS_PROTOCOL_HAS"></a> CORRECTIONS_PROTOCOL_HAS         | `uint8` | 3     | Galileo High Accuracy Service, received on E6 |
| <a id="#CORRECTIONS_PROTOCOL_PMP"></a> CORRECTIONS_PROTOCOL_PMP         | `uint8` | 4     | SPARTN over L-band (u-blox NEO-D9S)           |
| <a id="#CORRECTIONS_PROTOCOL_QZSS_L6"></a> CORRECTIONS_PROTOCOL_QZSS_L6 | `uint8` | 5     | QZSS CLAS                                     |

### CORRECTIONS_MSG_USED {#CORRECTIONS_MSG_USED}

Used in field(s): [corrections_msg_used](#fld_corrections_msg_used)

| Name                                                                      | Type    | Value | Description |
| ------------------------------------------------------------------------- | ------- | ----- | ----------- |
| <a id="#CORRECTIONS_MSG_USED_UNKNOWN"></a> CORRECTIONS_MSG_USED_UNKNOWN   | `uint8` | 0     |
| <a id="#CORRECTIONS_MSG_USED_NOT_USED"></a> CORRECTIONS_MSG_USED_NOT_USED | `uint8` | 1     |
| <a id="#CORRECTIONS_MSG_USED_USED"></a> CORRECTIONS_MSG_USED_USED         | `uint8` | 2     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGnss.msg)

::: details Click here to see original file

```c
# GNSS receiver report, position in WGS84
#
# One instance per receiver, published by its driver: the receiver's own solution, unmodified.
# The sensors module publishes the selected receiver on vehicle_gnss.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Measurement time if the driver knows it, else 0 or equal to timestamp. vehicle_gnss carries the corrected value

uint32 device_id # [-] Unique device ID for the sensor that does not change between power cycles

float64 latitude           # [deg] Latitude, allows centimeter level RTK precision
float64 longitude          # [deg] Longitude, allows centimeter level RTK precision
float64 altitude_msl       # [m] Altitude above MSL, from the receiver's own geoid model
float64 altitude_ellipsoid # [m] Altitude above the WGS84 ellipsoid

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
```

:::
