---
pageClass: is-wide-page
---

# VehicleGnss (UORB message)

Selected GNSS solution.

Published by the sensors module for every sample of the selected receiver, usable or not, with the selection state and the check result for that sample.
Heading is on vehicle_gnss_heading.

**TOPICS:** vehicle_gnss

## Fields

| 명칭                                                                       | 형식           | Unit [Frame] | Range/Enum      | 설명                                                                                                                                                                 |
| ------------------------------------------------------------------------ | ------------ | ---------------------------------------------------------------- | --------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                                      | `uint64`     | us                                                               |                 | Time since system start                                                                                                                                            |
| <a id="fld_timestamp_sample"></a>timestamp_sample   | `uint64`     | us                                                               |                 | Measurement time, delay corrected. Use this rather than receiver.timestamp_sample, which holds the same value |
| <a id="fld_receiver"></a>receiver                                        | `SensorGnss` |                                                                  |                 | The selected receiver's sensor_gnss sample with the corrected timestamp_sample                                           |
| <a id="fld_antenna_offset"></a>antenna_offset       | `float32[3]` | m [FRD]      |                 | Antenna position of the selected receiver                                                                                                                          |
| <a id="fld_selected_instance"></a>selected_instance | `uint8`      |                                                                  |                 | sensor_gnss instance of the selected receiver                                                                                                 |
| <a id="fld_selection_count"></a>selection_count     | `uint8`      |                                                                  |                 | Increments when the selected receiver changes                                                                                                                      |
| <a id="fld_usable"></a>usable                                            | `bool`       |                                                                  |                 | Sample may be used: the receiver has passed its checks for long enough                                                                             |
| <a id="fld_failed_checks"></a>failed_checks         | `uint16`     |                                                                  | [CHECK](#CHECK) | Bitmask of the enabled checks that failed                                                                                                                          |

## Enums

### CHECK {#CHECK}

Used in field(s): [failed_checks](#fld_failed_checks)

| 명칭                                                             | 형식       | Value | 설명                                         |
| -------------------------------------------------------------- | -------- | ----- | ------------------------------------------ |
| <a id="#CHECK_NSATS"></a> CHECK_NSATS     | `uint16` | 1     | Too few satellites                         |
| <a id="#CHECK_PDOP"></a> CHECK_PDOP       | `uint16` | 2     | Position dilution of precision too high    |
| <a id="#CHECK_EPH"></a> CHECK_EPH         | `uint16` | 4     | Horizontal position accuracy too low       |
| <a id="#CHECK_EPV"></a> CHECK_EPV         | `uint16` | 8     | Vertical position accuracy too low         |
| <a id="#CHECK_SACC"></a> CHECK_SACC       | `uint16` | 16    | Speed accuracy too low                     |
| <a id="#CHECK_HDRIFT"></a> CHECK_HDRIFT   | `uint16` | 32    | Horizontal position drift at rest too high |
| <a id="#CHECK_VDRIFT"></a> CHECK_VDRIFT   | `uint16` | 64    | Vertical position drift at rest too high   |
| <a id="#CHECK_HSPEED"></a> CHECK_HSPEED   | `uint16` | 128   | Horizontal speed at rest too high          |
| <a id="#CHECK_VSPEED"></a> CHECK_VSPEED   | `uint16` | 256   | Vertical speed at rest too high            |
| <a id="#CHECK_SPOOFED"></a> CHECK_SPOOFED | `uint16` | 512   | Receiver reports spoofing                  |
| <a id="#CHECK_FIX"></a> CHECK_FIX         | `uint16` | 1024  | Fix type too low                           |
| <a id="#CHECK_JAMMED"></a> CHECK_JAMMED   | `uint16` | 2048  | Receiver reports jamming                   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleGnss.msg)

:::details
Click here to see original file

```c
# Selected GNSS solution
#
# Published by the sensors module for every sample of the selected receiver, usable or not, with the selection state and the check result for that sample.
# Heading is on vehicle_gnss_heading.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Measurement time, delay corrected. Use this rather than receiver.timestamp_sample, which holds the same value

SensorGnss receiver # The selected receiver's sensor_gnss sample with the corrected timestamp_sample

float32[3] antenna_offset # [m] [@frame FRD] Antenna position of the selected receiver

# Selection state
uint8 selected_instance # [-] sensor_gnss instance of the selected receiver
uint8 selection_count   # [-] Increments when the selected receiver changes

# Check result for this sample
bool usable                 # Sample may be used: the receiver has passed its checks for long enough
uint16 failed_checks        # [@enum CHECK] Bitmask of the enabled checks that failed
uint16 CHECK_NSATS   = 1    # Too few satellites
uint16 CHECK_PDOP    = 2    # Position dilution of precision too high
uint16 CHECK_EPH     = 4    # Horizontal position accuracy too low
uint16 CHECK_EPV     = 8    # Vertical position accuracy too low
uint16 CHECK_SACC    = 16   # Speed accuracy too low
uint16 CHECK_HDRIFT  = 32   # Horizontal position drift at rest too high
uint16 CHECK_VDRIFT  = 64   # Vertical position drift at rest too high
uint16 CHECK_HSPEED  = 128  # Horizontal speed at rest too high
uint16 CHECK_VSPEED  = 256  # Vertical speed at rest too high
uint16 CHECK_SPOOFED = 512  # Receiver reports spoofing
uint16 CHECK_FIX     = 1024 # Fix type too low
uint16 CHECK_JAMMED  = 2048 # Receiver reports jamming
```

:::
