---
pageClass: is-wide-page
---

# DetectAndAvoidMostUrgent (UORB message)

Detect-and-avoid summary for the most urgent active conflict.

This is the aggregated DAA status from navigator's `DetectAndAvoid` component.
It publishes the single active conflict that currently has the highest urgency
after the per-traffic conflict buffer has been updated.

Unlike `detect_and_avoid`, this topic is not published for every processed
traffic report and does not include the detailed horizontal and vertical geometry.
It is the topic used for overall DAA status, automatic-action decisions, and
prearm checks.

Published by: `navigator` (`DetectAndAvoid`)
Used by: `commander` DAA arming checks, logging, and test


**TOPICS:** detect_and_avoid_most_urgent

## Fields

Name | Type | Unit [Frame] | Range/Enum | Description
--- | --- | --- | --- | ---
timestamp | `uint64` | us | | Time since system start
unique_id | `uint64` |  | | Encoded identifier of the current most urgent traffic aircraft, selected in priority order: ICAO address, ADS-B callsign, then reduced UAS ID tail bytes
unique_id_encoding | `uint8` |  | | Namespace used to decode `unique_id`
has_action | `bool` |  | | True if the configured DAA response for this most urgent conflict is stronger than Warn only
conflict_level | `uint8` |  | | Conflict level of the current most urgent active conflict
aircraft_dist | `float32` | m | | Approximate 3D range to the most urgent traffic aircraft

## Constants

Name | Type | Value | Description
--- | --- | --- |---
<a id="#UNIQUE_ID_ENCODING_ICAO"></a> UNIQUE_ID_ENCODING_ICAO | `uint8` | 0 | `unique_id` contains an ICAO address
<a id="#UNIQUE_ID_ENCODING_ADSB_CALLSIGN"></a> UNIQUE_ID_ENCODING_ADSB_CALLSIGN | `uint8` | 1 | `unique_id` contains an ADS-B callsign packed into a `uint64`
<a id="#UNIQUE_ID_ENCODING_UAS_ID"></a> UNIQUE_ID_ENCODING_UAS_ID | `uint8` | 2 | `unique_id` contains the reduced tail bytes of a UAS ID


## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DetectAndAvoidMostUrgent.msg)

::: details Click here to see original file

```c
# Detect-and-avoid summary for the most urgent active conflict.
#
# This is the aggregated DAA status from navigator's `DetectAndAvoid` component.
# It publishes the single active conflict that currently has the highest urgency
# after the per-traffic conflict buffer has been updated.
#
# Unlike `detect_and_avoid`, this topic is not published for every processed
# traffic report and does not include the detailed horizontal and vertical geometry.
# It is the topic used for overall DAA status, automatic-action decisions, and
# prearm checks.
#
# Published by: `navigator` (`DetectAndAvoid`)
# Used by: `commander` DAA arming checks, logging, and test

uint8 UNIQUE_ID_ENCODING_ICAO = 0 # `unique_id` contains an ICAO address
uint8 UNIQUE_ID_ENCODING_ADSB_CALLSIGN = 1 # `unique_id` contains an ADS-B callsign packed into a `uint64`
uint8 UNIQUE_ID_ENCODING_UAS_ID = 2 # `unique_id` contains the reduced tail bytes of a UAS ID

uint64 timestamp # [us] Time since system start
uint64 unique_id # [-] Encoded identifier of the current most urgent traffic aircraft, selected in priority order: ICAO address, ADS-B callsign, then reduced UAS ID tail bytes
uint8 unique_id_encoding # [-] Namespace used to decode `unique_id`

bool has_action # [-] True if the configured DAA response for this most urgent conflict is stronger than Warn only
uint8 conflict_level # [-] Conflict level of the current most urgent active conflict
float32 aircraft_dist # [m] Approximate 3D range to the most urgent traffic aircraft
```

:::
