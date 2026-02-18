---
pageClass: is-wide-page
---

# TransponderReport (UORB message)

**TOPICS:** transponder_report

## Fields

| 명칭                                 | 형식          | Unit [Frame] | Range/Enum | 설명                                                                          |
| ---------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------- |
| timestamp                          | `uint64`    |                                                                  |            | time since system start (microseconds)                   |
| icao_address  | `uint32`    |                                                                  |            | ICAO address                                                                |
| lat                                | `float64`   |                                                                  |            | Latitude, expressed as degrees                                              |
| lon                                | `float64`   |                                                                  |            | Longitude, expressed as degrees                                             |
| altitude_type | `uint8`     |                                                                  |            | Type from ADSB_ALTITUDE_TYPE enum |
| altitude                           | `float32`   |                                                                  |            | Altitude(ASL) in meters                                  |
| heading                            | `float32`   |                                                                  |            | Course over ground in radians, 0 to 2pi, 0 is north                         |
| hor_velocity  | `float32`   |                                                                  |            | The horizontal velocity in m/s                                              |
| ver_velocity  | `float32`   |                                                                  |            | The vertical velocity in m/s, positive is up                                |
| callsign                           | `char[9]`   |                                                                  |            | The callsign, 8+null                                                        |
| emitter_type  | `uint8`     |                                                                  |            | Type from ADSB_EMITTER_TYPE enum  |
| tslc                               | `uint8`     |                                                                  |            | Time since last communication in seconds                                    |
| flags                              | `uint16`    |                                                                  |            | Flags to indicate various statuses including valid data fields              |
| squawk                             | `uint16`    |                                                                  |            | Squawk code                                                                 |
| uas_id        | `uint8[18]` |                                                                  |            | Unique UAS ID                                                               |

## Constants

| 명칭                                                                                                                                                                                               | 형식       | Value | 설명 |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------- | ----- | -- |
| <a href="#PX4_ADSB_FLAGS_VALID_COORDS"></a> PX4_ADSB_FLAGS_VALID_COORDS                                      | `uint16` | 1     |    |
| <a href="#PX4_ADSB_FLAGS_VALID_ALTITUDE"></a> PX4_ADSB_FLAGS_VALID_ALTITUDE                                  | `uint16` | 2     |    |
| <a href="#PX4_ADSB_FLAGS_VALID_HEADING"></a> PX4_ADSB_FLAGS_VALID_HEADING                                    | `uint16` | 4     |    |
| <a href="#PX4_ADSB_FLAGS_VALID_VELOCITY"></a> PX4_ADSB_FLAGS_VALID_VELOCITY                                  | `uint16` | 8     |    |
| <a href="#PX4_ADSB_FLAGS_VALID_CALLSIGN"></a> PX4_ADSB_FLAGS_VALID_CALLSIGN                                  | `uint16` | 16    |    |
| <a href="#PX4_ADSB_FLAGS_VALID_SQUAWK"></a> PX4_ADSB_FLAGS_VALID_SQUAWK                                      | `uint16` | 32    |    |
| <a href="#PX4_ADSB_FLAGS_RETRANSLATE"></a> PX4_ADSB_FLAGS_RETRANSLATE                                                             | `uint16` | 256   |    |
| <a href="#ADSB_EMITTER_TYPE_NO_INFO"></a> ADSB_EMITTER_TYPE_NO_INFO                                          | `uint16` | 0     |    |
| <a href="#ADSB_EMITTER_TYPE_LIGHT"></a> ADSB_EMITTER_TYPE_LIGHT                                                                   | `uint16` | 1     |    |
| <a href="#ADSB_EMITTER_TYPE_SMALL"></a> ADSB_EMITTER_TYPE_SMALL                                                                   | `uint16` | 2     |    |
| <a href="#ADSB_EMITTER_TYPE_LARGE"></a> ADSB_EMITTER_TYPE_LARGE                                                                   | `uint16` | 3     |    |
| <a href="#ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE"></a> ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE | `uint16` | 4     |    |
| <a href="#ADSB_EMITTER_TYPE_HEAVY"></a> ADSB_EMITTER_TYPE_HEAVY                                                                   | `uint16` | 5     |    |
| <a href="#ADSB_EMITTER_TYPE_HIGHLY_MANUV"></a> ADSB_EMITTER_TYPE_HIGHLY_MANUV                                | `uint16` | 6     |    |
| <a href="#ADSB_EMITTER_TYPE_ROTOCRAFT"></a> ADSB_EMITTER_TYPE_ROTOCRAFT                                                           | `uint16` | 7     |    |
| <a href="#ADSB_EMITTER_TYPE_UNASSIGNED"></a> ADSB_EMITTER_TYPE_UNASSIGNED                                                         | `uint16` | 8     |    |
| <a href="#ADSB_EMITTER_TYPE_GLIDER"></a> ADSB_EMITTER_TYPE_GLIDER                                                                 | `uint16` | 9     |    |
| <a href="#ADSB_EMITTER_TYPE_LIGHTER_AIR"></a> ADSB_EMITTER_TYPE_LIGHTER_AIR                                  | `uint16` | 10    |    |
| <a href="#ADSB_EMITTER_TYPE_PARACHUTE"></a> ADSB_EMITTER_TYPE_PARACHUTE                                                           | `uint16` | 11    |    |
| <a href="#ADSB_EMITTER_TYPE_ULTRA_LIGHT"></a> ADSB_EMITTER_TYPE_ULTRA_LIGHT                                  | `uint16` | 12    |    |
| <a href="#ADSB_EMITTER_TYPE_UNASSIGNED2"></a> ADSB_EMITTER_TYPE_UNASSIGNED2                                                       | `uint16` | 13    |    |
| <a href="#ADSB_EMITTER_TYPE_UAV"></a> ADSB_EMITTER_TYPE_UAV                                                                       | `uint16` | 14    |    |
| <a href="#ADSB_EMITTER_TYPE_SPACE"></a> ADSB_EMITTER_TYPE_SPACE                                                                   | `uint16` | 15    |    |
| <a href="#ADSB_EMITTER_TYPE_UNASSGINED3"></a> ADSB_EMITTER_TYPE_UNASSGINED3                                                       | `uint16` | 16    |    |
| <a href="#ADSB_EMITTER_TYPE_EMERGENCY_SURFACE"></a> ADSB_EMITTER_TYPE_EMERGENCY_SURFACE                      | `uint16` | 17    |    |
| <a href="#ADSB_EMITTER_TYPE_SERVICE_SURFACE"></a> ADSB_EMITTER_TYPE_SERVICE_SURFACE                          | `uint16` | 18    |    |
| <a href="#ADSB_EMITTER_TYPE_POINT_OBSTACLE"></a> ADSB_EMITTER_TYPE_POINT_OBSTACLE                            | `uint16` | 19    |    |
| <a href="#ADSB_EMITTER_TYPE_ENUM_END"></a> ADSB_EMITTER_TYPE_ENUM_END                                        | `uint16` | 20    |    |
| <a href="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                                                                                      | `uint8`  | 16    |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TransponderReport.msg)

:::details
Click here to see original file

```c
uint64 timestamp	# time since system start (microseconds)
uint32 icao_address 	# ICAO address
float64 lat 		# Latitude, expressed as degrees
float64 lon 		# Longitude, expressed as degrees
uint8 altitude_type	# Type from ADSB_ALTITUDE_TYPE enum
float32 altitude 	# Altitude(ASL) in meters
float32 heading 	# Course over ground in radians, 0 to 2pi, 0 is north
float32 hor_velocity	# The horizontal velocity in m/s
float32 ver_velocity 	# The vertical velocity in m/s, positive is up
char[9] callsign	# The callsign, 8+null
uint8 emitter_type 	# Type from ADSB_EMITTER_TYPE enum
uint8 tslc 		# Time since last communication in seconds
uint16 flags 		# Flags to indicate various statuses including valid data fields
uint16 squawk 		# Squawk code
uint8[18] uas_id	# Unique UAS ID

# ADSB flags
uint16 PX4_ADSB_FLAGS_VALID_COORDS = 1
uint16 PX4_ADSB_FLAGS_VALID_ALTITUDE = 2
uint16 PX4_ADSB_FLAGS_VALID_HEADING = 4
uint16 PX4_ADSB_FLAGS_VALID_VELOCITY = 8
uint16 PX4_ADSB_FLAGS_VALID_CALLSIGN = 16
uint16 PX4_ADSB_FLAGS_VALID_SQUAWK = 32
uint16 PX4_ADSB_FLAGS_RETRANSLATE = 256

#ADSB Emitter Data:
#from mavlink/v2.0/common/common.h
uint16 ADSB_EMITTER_TYPE_NO_INFO=0
uint16 ADSB_EMITTER_TYPE_LIGHT=1
uint16 ADSB_EMITTER_TYPE_SMALL=2
uint16 ADSB_EMITTER_TYPE_LARGE=3
uint16 ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE=4
uint16 ADSB_EMITTER_TYPE_HEAVY=5
uint16 ADSB_EMITTER_TYPE_HIGHLY_MANUV=6
uint16 ADSB_EMITTER_TYPE_ROTOCRAFT=7
uint16 ADSB_EMITTER_TYPE_UNASSIGNED=8
uint16 ADSB_EMITTER_TYPE_GLIDER=9
uint16 ADSB_EMITTER_TYPE_LIGHTER_AIR=10
uint16 ADSB_EMITTER_TYPE_PARACHUTE=11
uint16 ADSB_EMITTER_TYPE_ULTRA_LIGHT=12
uint16 ADSB_EMITTER_TYPE_UNASSIGNED2=13
uint16 ADSB_EMITTER_TYPE_UAV=14
uint16 ADSB_EMITTER_TYPE_SPACE=15
uint16 ADSB_EMITTER_TYPE_UNASSGINED3=16
uint16 ADSB_EMITTER_TYPE_EMERGENCY_SURFACE=17
uint16 ADSB_EMITTER_TYPE_SERVICE_SURFACE=18
uint16 ADSB_EMITTER_TYPE_POINT_OBSTACLE=19
uint16 ADSB_EMITTER_TYPE_ENUM_END=20

uint8 ORB_QUEUE_LENGTH = 16
```

:::
