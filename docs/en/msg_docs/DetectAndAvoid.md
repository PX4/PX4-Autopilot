---
pageClass: is-wide-page
---

# DetectAndAvoid (UORB message)

Detect-and-avoid conflict assessment for one traffic aircraft.

Detailed per-traffic output from navigator's `DetectAndAvoid` component.
A sample is published when an evaluated report indicates a conflict or updates
an aircraft already tracked as a conflict. New reports assessed at NONE are omitted.

Unlike `detect_and_avoid_most_urgent`, this topic does not summarize the single
active conflict driving DAA actions or prearm checks. It reports the conflict level
and geometry for the specific traffic aircraft that was just evaluated.

Published by: `navigator` (`DetectAndAvoid`)
Used by: logging and tests

**TOPICS:** detect_and_avoid

## Fields

| Name                                                          | Type      | Unit [Frame] | Range/Enum | Description                                                                                                                               |
| ------------------------------------------------------------- | --------- | ------------ | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                           | `uint64`  | us           |            | Time since system start                                                                                                                   |
| <a id="fld_unique_id"></a>unique_id                           | `uint64`  |              |            | Encoded traffic identifier selected in priority order: ICAO address, ADS-B callsign, then reduced UAS ID tail bytes                       |
| <a id="fld_unique_id_encoding"></a>unique_id_encoding         | `uint8`   |              |            | Namespace used to decode `unique_id`                                                                                                      |
| <a id="fld_conflict_level"></a>conflict_level                 | `uint8`   |              |            | Conflict level calculated for this traffic aircraft                                                                                       |
| <a id="fld_aircraft_dist"></a>aircraft_dist                   | `float32` | m            |            | Current 3D point-to-point range between ownship and the traffic aircraft                                                                  |
| <a id="fld_aircraft_dist_hor"></a>aircraft_dist_hor           | `float32` | m [NED]      |            | Horizontal separation metric. In Crosstrack mode this is the signed crosstrack distance when available, otherwise direct horizontal range |
| <a id="fld_aircraft_dist_vert"></a>aircraft_dist_vert         | `float32` | m [NED]      |            | Vertical separation between ownship (the current vehicle) and the traffic aircraft                                                        |
| <a id="fld_expected_min_dist_time"></a>expected_min_dist_time | `float32` | s            |            | Conservative collision-time estimate from current 3D separation and the sum of ownship and traffic speed magnitudes                       |

## Constants

| Name                                                                            | Type    | Value | Description                                                   |
| ------------------------------------------------------------------------------- | ------- | ----- | ------------------------------------------------------------- |
| <a id="#DAA_CONFLICT_LVL_NONE"></a> DAA_CONFLICT_LVL_NONE                       | `uint8` | 0     |
| <a id="#DAA_CONFLICT_LVL_LOW"></a> DAA_CONFLICT_LVL_LOW                         | `uint8` | 1     |
| <a id="#DAA_CONFLICT_LVL_MEDIUM"></a> DAA_CONFLICT_LVL_MEDIUM                   | `uint8` | 2     |
| <a id="#DAA_CONFLICT_LVL_HIGH"></a> DAA_CONFLICT_LVL_HIGH                       | `uint8` | 3     |
| <a id="#DAA_CONFLICT_LVL_CRITICAL"></a> DAA_CONFLICT_LVL_CRITICAL               | `uint8` | 4     |
| <a id="#UNIQUE_ID_ENCODING_ICAO"></a> UNIQUE_ID_ENCODING_ICAO                   | `uint8` | 0     | `unique_id` contains an ICAO address                          |
| <a id="#UNIQUE_ID_ENCODING_ADSB_CALLSIGN"></a> UNIQUE_ID_ENCODING_ADSB_CALLSIGN | `uint8` | 1     | `unique_id` contains an ADS-B callsign packed into a `uint64` |
| <a id="#UNIQUE_ID_ENCODING_UAS_ID"></a> UNIQUE_ID_ENCODING_UAS_ID               | `uint8` | 2     | `unique_id` contains the reduced tail bytes of a UAS ID       |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH                                 | `uint8` | 16    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DetectAndAvoid.msg)

::: details Click here to see original file

```c
# Detect-and-avoid conflict assessment for one traffic aircraft.
#
# Detailed per-traffic output from navigator's `DetectAndAvoid` component.
# A sample is published when an evaluated report indicates a conflict or updates
# an aircraft already tracked as a conflict. New reports assessed at NONE are omitted.
#
# Unlike `detect_and_avoid_most_urgent`, this topic does not summarize the single
# active conflict driving DAA actions or prearm checks. It reports the conflict level
# and geometry for the specific traffic aircraft that was just evaluated.
#
# Published by: `navigator` (`DetectAndAvoid`)
# Used by: logging and tests

uint8 DAA_CONFLICT_LVL_NONE = 0
uint8 DAA_CONFLICT_LVL_LOW = 1
uint8 DAA_CONFLICT_LVL_MEDIUM = 2
uint8 DAA_CONFLICT_LVL_HIGH = 3
uint8 DAA_CONFLICT_LVL_CRITICAL = 4

uint8 UNIQUE_ID_ENCODING_ICAO = 0 # `unique_id` contains an ICAO address
uint8 UNIQUE_ID_ENCODING_ADSB_CALLSIGN = 1 # `unique_id` contains an ADS-B callsign packed into a `uint64`
uint8 UNIQUE_ID_ENCODING_UAS_ID = 2 # `unique_id` contains the reduced tail bytes of a UAS ID

uint8 ORB_QUEUE_LENGTH = 16

uint64 timestamp # [us] Time since system start
uint64 unique_id # [-] Encoded traffic identifier selected in priority order: ICAO address, ADS-B callsign, then reduced UAS ID tail bytes
uint8 unique_id_encoding # [-] Namespace used to decode `unique_id`

uint8 conflict_level # [-] Conflict level calculated for this traffic aircraft
float32 aircraft_dist # [m] Current 3D point-to-point range between ownship and the traffic aircraft
float32 aircraft_dist_hor # [m] [@frame NED] Horizontal separation metric. In Crosstrack mode this is the signed crosstrack distance when available, otherwise direct horizontal range
float32 aircraft_dist_vert # [m] [@frame NED] Vertical separation between ownship (the current vehicle) and the traffic aircraft
float32 expected_min_dist_time # [s] Conservative collision-time estimate from current 3D separation and the sum of ownship and traffic speed magnitudes
```

:::
