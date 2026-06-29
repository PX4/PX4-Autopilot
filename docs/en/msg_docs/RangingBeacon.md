---
pageClass: is-wide-page
---

# RangingBeacon (UORB message)

Ranging beacon measurement data (e.g. LoRa, UWB).

**TOPICS:** ranging_beacon

## Fields

| Name                                              | Type      | Unit [Frame] | Range/Enum            | Description                                 |
| ------------------------------------------------- | --------- | ------------ | --------------------- | ------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`  | us           |                       | time since system start                     |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`  | us           |                       | the timestamp of the raw data               |
| <a id="fld_beacon_id"></a>beacon_id               | `uint8`   |              |                       |
| <a id="fld_range"></a>range                       | `float32` | m            |                       | Range measurement                           |
| <a id="fld_lat"></a>lat                           | `float64` | deg          |                       | Latitude                                    |
| <a id="fld_lon"></a>lon                           | `float64` | deg          |                       | Longitude                                   |
| <a id="fld_alt"></a>alt                           | `float32` | m            |                       | Beacon altitude (frame defined in alt_type) |
| <a id="fld_alt_type"></a>alt_type                 | `uint8`   |              | [ALT_TYPE](#ALT_TYPE) | Altitude frame for alt field                |
| <a id="fld_hacc"></a>hacc                         | `float32` | m            |                       | Groundbeacon horizontal accuracy            |
| <a id="fld_vacc"></a>vacc                         | `float32` | m            |                       | Groundbeacon vertical accuracy              |
| <a id="fld_sequence_nr"></a>sequence_nr           | `uint8`   |              |                       |
| <a id="fld_status"></a>status                     | `uint8`   |              |                       |
| <a id="fld_carrier_freq"></a>carrier_freq         | `uint16`  | MHz          |                       | Carrier frequency                           |
| <a id="fld_range_accuracy"></a>range_accuracy     | `float32` | m            |                       | Range accuracy estimate                     |

## Enums

### ALT_TYPE {#ALT_TYPE}

Used in field(s): [alt_type](#fld_alt_type)

| Name                                        | Type    | Value | Description                          |
| ------------------------------------------- | ------- | ----- | ------------------------------------ |
| <a id="#ALT_TYPE_WGS84"></a> ALT_TYPE_WGS84 | `uint8` | 0     | Altitude above WGS84 ellipsoid       |
| <a id="#ALT_TYPE_MSL"></a> ALT_TYPE_MSL     | `uint8` | 1     | Altitude above Mean Sea Level (AMSL) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RangingBeacon.msg)

::: details Click here to see original file

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
