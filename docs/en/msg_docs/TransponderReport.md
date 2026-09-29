---
pageClass: is-wide-page
---

# TransponderReport (UORB message)

Transponder report.

ADSB report closely matching MAVLink's ADSB_VEHICLE (246) message with few internal extra fields at the end.
Populated by ADSB receivers, processed for user messaging and navigator, logging and republishing ADSB information.

**TOPICS:** transponder_report

## Fields

| Name                                        | Type        | Unit [Frame] | Range/Enum                              | Description                                                                           |
| ------------------------------------------- | ----------- | ------------ | --------------------------------------- | ------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp         | `uint64`    | us           |                                         | Time since system start                                                               |
| <a id="fld_icao_address"></a>icao_address   | `uint32`    |              |                                         | ICAO address                                                                          |
| <a id="fld_lat"></a>lat                     | `float64`   | deg          |                                         | Latitude, validity flag: PX4_ADSB_FLAGS_VALID_COORDS                                  |
| <a id="fld_lon"></a>lon                     | `float64`   | deg          |                                         | Longitude, validity flag: PX4_ADSB_FLAGS_VALID_COORDS                                 |
| <a id="fld_altitude_type"></a>altitude_type | `uint8`     |              |                                         | Type from ADSB_ALTITUDE_TYPE enum                                                     |
| <a id="fld_altitude"></a>altitude           | `float32`   | m            |                                         | Altitude (ASL), validity flag: PX4_ADSB_FLAGS_VALID_ALTITUDE                          |
| <a id="fld_heading"></a>heading             | `float32`   | rad          |                                         | Course over ground, 0 to 2pi, 0 is north, validity flag: PX4_ADSB_FLAGS_VALID_HEADING |
| <a id="fld_hor_velocity"></a>hor_velocity   | `float32`   | m/s          |                                         | Horizontal velocity, validity flag: PX4_ADSB_FLAGS_VALID_VELOCITY                     |
| <a id="fld_ver_velocity"></a>ver_velocity   | `float32`   | m/s          |                                         | Vertical velocity, positive is up, validity flag: PX4_ADSB_FLAGS_VALID_VELOCITY       |
| <a id="fld_callsign"></a>callsign           | `char[9]`   |              |                                         | The callsign, 8+null, validity flag: PX4_ADSB_FLAGS_VALID_CALLSIGN                    |
| <a id="fld_emitter_type"></a>emitter_type   | `uint8`     |              | [ADSB_EMITTER_TYPE](#ADSB_EMITTER_TYPE) | Type matching MAVLink's ADSB_EMITTER_TYPE enum                                        |
| <a id="fld_tslc"></a>tslc                   | `uint8`     | s            |                                         | Time since last communication                                                         |
| <a id="fld_flags"></a>flags                 | `uint16`    |              | [PX4_ADSB_FLAGS](#PX4_ADSB_FLAGS)       | Flags matching MAVLink's ADSB_FLAGS bitmask                                           |
| <a id="fld_squawk"></a>squawk               | `uint16`    |              |                                         | Squawk code, validity flag: PX4_ADSB_FLAGS_VALID_SQUAWK                               |
| <a id="fld_uas_id"></a>uas_id               | `uint8[18]` |              |                                         | Unique UAS ID, not part of ADSB_VEHICLE MAVLink message                               |

## Enums

### ADSB_EMITTER_TYPE {#ADSB_EMITTER_TYPE}

Used in field(s): [emitter_type](#fld_emitter_type)

| Name                                                                                  | Type     | Value | Description |
| ------------------------------------------------------------------------------------- | -------- | ----- | ----------- |
| <a id="#ADSB_EMITTER_TYPE_NO_INFO"></a> ADSB_EMITTER_TYPE_NO_INFO                     | `uint16` | 0     |
| <a id="#ADSB_EMITTER_TYPE_LIGHT"></a> ADSB_EMITTER_TYPE_LIGHT                         | `uint16` | 1     |
| <a id="#ADSB_EMITTER_TYPE_SMALL"></a> ADSB_EMITTER_TYPE_SMALL                         | `uint16` | 2     |
| <a id="#ADSB_EMITTER_TYPE_LARGE"></a> ADSB_EMITTER_TYPE_LARGE                         | `uint16` | 3     |
| <a id="#ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE"></a> ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE | `uint16` | 4     |
| <a id="#ADSB_EMITTER_TYPE_HEAVY"></a> ADSB_EMITTER_TYPE_HEAVY                         | `uint16` | 5     |
| <a id="#ADSB_EMITTER_TYPE_HIGHLY_MANUV"></a> ADSB_EMITTER_TYPE_HIGHLY_MANUV           | `uint16` | 6     |
| <a id="#ADSB_EMITTER_TYPE_ROTOCRAFT"></a> ADSB_EMITTER_TYPE_ROTOCRAFT                 | `uint16` | 7     |
| <a id="#ADSB_EMITTER_TYPE_UNASSIGNED"></a> ADSB_EMITTER_TYPE_UNASSIGNED               | `uint16` | 8     |
| <a id="#ADSB_EMITTER_TYPE_GLIDER"></a> ADSB_EMITTER_TYPE_GLIDER                       | `uint16` | 9     |
| <a id="#ADSB_EMITTER_TYPE_LIGHTER_AIR"></a> ADSB_EMITTER_TYPE_LIGHTER_AIR             | `uint16` | 10    |
| <a id="#ADSB_EMITTER_TYPE_PARACHUTE"></a> ADSB_EMITTER_TYPE_PARACHUTE                 | `uint16` | 11    |
| <a id="#ADSB_EMITTER_TYPE_ULTRA_LIGHT"></a> ADSB_EMITTER_TYPE_ULTRA_LIGHT             | `uint16` | 12    |
| <a id="#ADSB_EMITTER_TYPE_UNASSIGNED2"></a> ADSB_EMITTER_TYPE_UNASSIGNED2             | `uint16` | 13    |
| <a id="#ADSB_EMITTER_TYPE_UAV"></a> ADSB_EMITTER_TYPE_UAV                             | `uint16` | 14    |
| <a id="#ADSB_EMITTER_TYPE_SPACE"></a> ADSB_EMITTER_TYPE_SPACE                         | `uint16` | 15    |
| <a id="#ADSB_EMITTER_TYPE_UNASSGINED3"></a> ADSB_EMITTER_TYPE_UNASSGINED3             | `uint16` | 16    |
| <a id="#ADSB_EMITTER_TYPE_EMERGENCY_SURFACE"></a> ADSB_EMITTER_TYPE_EMERGENCY_SURFACE | `uint16` | 17    |
| <a id="#ADSB_EMITTER_TYPE_SERVICE_SURFACE"></a> ADSB_EMITTER_TYPE_SERVICE_SURFACE     | `uint16` | 18    |
| <a id="#ADSB_EMITTER_TYPE_POINT_OBSTACLE"></a> ADSB_EMITTER_TYPE_POINT_OBSTACLE       | `uint16` | 19    |
| <a id="#ADSB_EMITTER_TYPE_ENUM_END"></a> ADSB_EMITTER_TYPE_ENUM_END                   | `uint16` | 20    |

### PX4_ADSB_FLAGS {#PX4_ADSB_FLAGS}

Used in field(s): [flags](#fld_flags)

| Name                                                                                        | Type     | Value | Description |
| ------------------------------------------------------------------------------------------- | -------- | ----- | ----------- |
| <a id="#PX4_ADSB_FLAGS_VALID_COORDS"></a> PX4_ADSB_FLAGS_VALID_COORDS                       | `uint16` | 1     |
| <a id="#PX4_ADSB_FLAGS_VALID_ALTITUDE"></a> PX4_ADSB_FLAGS_VALID_ALTITUDE                   | `uint16` | 2     |
| <a id="#PX4_ADSB_FLAGS_VALID_HEADING"></a> PX4_ADSB_FLAGS_VALID_HEADING                     | `uint16` | 4     |
| <a id="#PX4_ADSB_FLAGS_VALID_VELOCITY"></a> PX4_ADSB_FLAGS_VALID_VELOCITY                   | `uint16` | 8     |
| <a id="#PX4_ADSB_FLAGS_VALID_CALLSIGN"></a> PX4_ADSB_FLAGS_VALID_CALLSIGN                   | `uint16` | 16    |
| <a id="#PX4_ADSB_FLAGS_VALID_SQUAWK"></a> PX4_ADSB_FLAGS_VALID_SQUAWK                       | `uint16` | 32    |
| <a id="#PX4_ADSB_FLAGS_SIMULATED"></a> PX4_ADSB_FLAGS_SIMULATED                             | `uint16` | 64    |
| <a id="#PX4_ADSB_FLAGS_VERTICAL_VELOCITY_VALID"></a> PX4_ADSB_FLAGS_VERTICAL_VELOCITY_VALID | `uint16` | 128   |
| <a id="#PX4_ADSB_FLAGS_BARO_VALID"></a> PX4_ADSB_FLAGS_BARO_VALID                           | `uint16` | 256   |
| <a id="#PX4_ADSB_FLAGS_SOURCE_UAT"></a> PX4_ADSB_FLAGS_SOURCE_UAT                           | `uint16` | 32768 |

## Constants

| Name                                            | Type    | Value | Description |
| ----------------------------------------------- | ------- | ----- | ----------- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 16    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TransponderReport.msg)

::: details Click here to see original file

```c
# Transponder report
#
# ADSB report closely matching MAVLink's ADSB_VEHICLE (246) message with few internal extra fields at the end.
# Populated by ADSB receivers, processed for user messaging and navigator, logging and republishing ADSB information.

uint64 timestamp # [us] Time since system start

uint32 icao_address # [-] ICAO address
float64 lat # [deg] Latitude, validity flag: PX4_ADSB_FLAGS_VALID_COORDS
float64 lon # [deg] Longitude, validity flag: PX4_ADSB_FLAGS_VALID_COORDS
uint8 altitude_type # Type from ADSB_ALTITUDE_TYPE enum
float32 altitude # [m] Altitude (ASL), validity flag: PX4_ADSB_FLAGS_VALID_ALTITUDE
float32 heading # [rad] Course over ground, 0 to 2pi, 0 is north, validity flag: PX4_ADSB_FLAGS_VALID_HEADING
float32 hor_velocity # [m/s] Horizontal velocity, validity flag: PX4_ADSB_FLAGS_VALID_VELOCITY
float32 ver_velocity # [m/s] Vertical velocity, positive is up, validity flag: PX4_ADSB_FLAGS_VALID_VELOCITY
char[9] callsign # The callsign, 8+null, validity flag: PX4_ADSB_FLAGS_VALID_CALLSIGN

uint8 emitter_type # [@enum ADSB_EMITTER_TYPE] Type matching MAVLink's ADSB_EMITTER_TYPE enum
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

uint8 tslc # [s] Time since last communication

uint16 flags # [@enum PX4_ADSB_FLAGS] Flags matching MAVLink's ADSB_FLAGS bitmask
uint16 PX4_ADSB_FLAGS_VALID_COORDS = 1
uint16 PX4_ADSB_FLAGS_VALID_ALTITUDE = 2
uint16 PX4_ADSB_FLAGS_VALID_HEADING = 4
uint16 PX4_ADSB_FLAGS_VALID_VELOCITY = 8
uint16 PX4_ADSB_FLAGS_VALID_CALLSIGN = 16
uint16 PX4_ADSB_FLAGS_VALID_SQUAWK = 32
uint16 PX4_ADSB_FLAGS_SIMULATED = 64
uint16 PX4_ADSB_FLAGS_VERTICAL_VELOCITY_VALID = 128
uint16 PX4_ADSB_FLAGS_BARO_VALID = 256
uint16 PX4_ADSB_FLAGS_SOURCE_UAT = 32768

uint16 squawk # [-] Squawk code, validity flag: PX4_ADSB_FLAGS_VALID_SQUAWK

uint8[18] uas_id # [-] Unique UAS ID, not part of ADSB_VEHICLE MAVLink message

uint8 ORB_QUEUE_LENGTH = 16
```

:::
