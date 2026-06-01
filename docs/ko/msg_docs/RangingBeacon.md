---
pageClass: is-wide-page
---

# RangingBeacon (UORB message)

Ranging beacon measurement data (e.g. LoRa, UWB).

**TOPICS:** ranging_beacon

## Fields

| 명칭                                    | 형식        | Unit [Frame] | Range/Enum                                 | 설명                                                                                  |
| ------------------------------------- | --------- | ---------------------------------------------------------------- | ------------------------------------------ | ----------------------------------------------------------------------------------- |
| timestamp                             | `uint64`  | us                                                               |                                            | time since system start                                                             |
| timestamp_sample | `uint64`  | us                                                               |                                            | the timestamp of the raw data                                                       |
| beacon_id        | `uint8`   |                                                                  |                                            |                                                                                     |
| range                                 | `float32` | m                                                                |                                            | Range measurement                                                                   |
| lat                                   | `float64` | deg                                                              |                                            | Latitude                                                                            |
| lon                                   | `float64` | deg                                                              |                                            | Longitude                                                                           |
| alt                                   | `float32` | m                                                                |                                            | Beacon altitude (frame defined in alt_type) |
| alt_type         | `uint8`   |                                                                  | [ALT_TYPE](#ALT_TYPE) | Altitude frame for alt field                                                        |
| hacc                                  | `float32` | m                                                                |                                            | Groundbeacon horizontal accuracy                                                    |
| vacc                                  | `float32` | m                                                                |                                            | Groundbeacon vertical accuracy                                                      |
| sequence_nr      | `uint8`   |                                                                  |                                            |                                                                                     |
| status                                | `uint8`   |                                                                  |                                            |                                                                                     |
| carrier_freq     | `uint16`  | MHz                                                              |                                            | Carrier frequency                                                                   |
| range_accuracy   | `float32` | m                                                                |                                            | Range accuracy estimate                                                             |

## Enums

### ALT_TYPE {#ALT_TYPE}

| 명칭                                                                                    | 형식      | Value | 설명                                                      |
| ------------------------------------------------------------------------------------- | ------- | ----- | ------------------------------------------------------- |
| <a id="#ALT_TYPE_WGS84"></a> ALT_TYPE_WGS84 | `uint8` | 0     | Altitude above WGS84 ellipsoid                          |
| <a id="#ALT_TYPE_MSL"></a> ALT_TYPE_MSL     | `uint8` | 1     | Altitude above Mean Sea Level (AMSL) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RangingBeacon.msg)

:::details
Click here to see original file

```c
# Ranging beacon measurement data (e.g. LoRa, UWB)

uint64 timestamp                # [us] time since system start
uint64 timestamp_sample         # [us] the timestamp of the raw data
uint8 beacon_id
float32 range                   # [m] Range measurement

float64 lat                     # [deg] Latitude
float64 lon                     # [deg] Longitude
float32 alt                     # [m] Beacon altitude (frame defined in alt_type)
uint8 alt_type                  # [@enum ALT_TYPE] Altitude frame for alt field
uint8 ALT_TYPE_WGS84 = 0        # Altitude above WGS84 ellipsoid
uint8 ALT_TYPE_MSL   = 1        # Altitude above Mean Sea Level (AMSL)

float32 hacc                    # [m] Groundbeacon horizontal accuracy
float32 vacc                    # [m] Groundbeacon vertical accuracy

uint8 sequence_nr
uint8 status
uint16 carrier_freq             # [MHz] Carrier frequency
float32 range_accuracy          # [m] Range accuracy estimate


# TOPICS ranging_beacon
```

:::
