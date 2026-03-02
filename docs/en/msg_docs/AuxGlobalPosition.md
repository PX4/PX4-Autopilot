---
pageClass: is-wide-page
---

# AuxGlobalPosition (UORB message)

Auxiliary global position.

This message provides global position data from an external source such as
pseudolites, visual navigation, or other positioning system.

**TOPICS:** aux_global_position

## Fields

| Name                  | Type      | Unit [Frame] | Range/Enum        | Description                                                                 |
| --------------------- | --------- | ------------ | ----------------- | --------------------------------------------------------------------------- |
| timestamp             | `uint64`  | us           |                   | Time since system start                                                     |
| timestamp_sample      | `uint64`  | us           |                   | Timestamp of the raw data                                                   |
| id                    | `uint8`   |              |                   | Unique identifier for the AGP source                                        |
| source                | `uint8`   |              | [SOURCE](#SOURCE) | Source type of the position data (based on mavlink::GLOBAL_POSITION_SRC)    |
| lat                   | `float64` | deg          |                   | Latitude in WGS84                                                           |
| lon                   | `float64` | deg          |                   | Longitude in WGS84                                                          |
| alt                   | `float32` | m            |                   | Altitude above mean sea level (AMSL) (Invalid: NaN)                         |
| eph                   | `float32` | m            |                   | Std dev of horizontal position, lower bounded by NOISE param (Invalid: NaN) |
| epv                   | `float32` | m            |                   | Std dev of vertical position, lower bounded by NOISE param (Invalid: NaN)   |
| lat_lon_reset_counter | `uint8`   |              |                   | Counter for reset events on horizontal position coordinates                 |

## Enums

### SOURCE {#SOURCE}

| Name                                                | Type    | Value | Description    |
| --------------------------------------------------- | ------- | ----- | -------------- |
| <a id="#SOURCE_UNKNOWN"></a> SOURCE_UNKNOWN         | `uint8` | 0     | Unknown source |
| <a id="#SOURCE_GNSS"></a> SOURCE_GNSS               | `uint8` | 1     | GNSS           |
| <a id="#SOURCE_VISION"></a> SOURCE_VISION           | `uint8` | 2     | Vision         |
| <a id="#SOURCE_PSEUDOLITES"></a> SOURCE_PSEUDOLITES | `uint8` | 3     | Pseudolites    |
| <a id="#SOURCE_TERRAIN"></a> SOURCE_TERRAIN         | `uint8` | 4     | Terrain        |
| <a id="#SOURCE_MAGNETIC"></a> SOURCE_MAGNETIC       | `uint8` | 5     | Magnetic       |
| <a id="#SOURCE_ESTIMATOR"></a> SOURCE_ESTIMATOR     | `uint8` | 6     | Estimator      |

## Constants

| Name                                          | Type     | Value | Description |
| --------------------------------------------- | -------- | ----- | ----------- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 1     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/AuxGlobalPosition.msg)

::: details Click here to see original file

```c
# Auxiliary global position
#
# This message provides global position data from an external source such as
# pseudolites, visual navigation, or other positioning system.

uint32 MESSAGE_VERSION = 1

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

uint8 id # [-] Unique identifier for the AGP source
uint8 source # [@enum SOURCE] Source type of the position data (based on mavlink::GLOBAL_POSITION_SRC)
uint8 SOURCE_UNKNOWN = 0 # Unknown source
uint8 SOURCE_GNSS = 1 # GNSS
uint8 SOURCE_VISION = 2 # Vision
uint8 SOURCE_PSEUDOLITES = 3 # Pseudolites
uint8 SOURCE_TERRAIN = 4 # Terrain
uint8 SOURCE_MAGNETIC = 5 # Magnetic
uint8 SOURCE_ESTIMATOR = 6 # Estimator

# lat, lon: required for horizontal position fusion, alt: required for vertical position fusion
float64 lat # [deg] Latitude in WGS84
float64 lon # [deg] Longitude in WGS84
float32 alt # [m] [@invalid NaN] Altitude above mean sea level (AMSL)

float32 eph # [m] [@invalid NaN] Std dev of horizontal position, lower bounded by NOISE param
float32 epv # [m] [@invalid NaN] Std dev of vertical position, lower bounded by NOISE param

uint8 lat_lon_reset_counter # [-] Counter for reset events on horizontal position coordinates

# TOPICS aux_global_position
```

:::
