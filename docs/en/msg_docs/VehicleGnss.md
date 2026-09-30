---
pageClass: is-wide-page
---

# VehicleGnss (UORB message)

Selected GNSS solution.

Published by the sensors module for every sample of the selected receiver.
Heading is on vehicle_gnss_heading.

**TOPICS:** vehicle_gnss

## Fields

| Name                                              | Type         | Unit [Frame] | Range/Enum | Description                                                                                                                                                               |
| ------------------------------------------------- | ------------ | ------------ | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`     | us           |            | Time since system start                                                                                                                                                   |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`     | us           |            | Measurement time, corrected by SENS_GNSSn_DELAY or PPS. Use this, not receiver.timestamp_sample: same value, but only top-level timestamps are time-synced over uXRCE-DDS |
| <a id="fld_receiver"></a>receiver                 | `SensorGnss` |              |            | The selected receiver's sensor_gnss sample with the corrected timestamp_sample                                                                                            |
| <a id="fld_antenna_offset"></a>antenna_offset     | `float32[3]` | m [FRD]      |            | Antenna position of the selected receiver (SENS_GNSSn_OFFX/Y/Z)                                                                                                           |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleGnss.msg)

::: details Click here to see original file

```c
# Selected GNSS solution
#
# Published by the sensors module for every sample of the selected receiver.
# Heading is on vehicle_gnss_heading.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Measurement time, corrected by SENS_GNSSn_DELAY or PPS. Use this, not receiver.timestamp_sample: same value, but only top-level timestamps are time-synced over uXRCE-DDS

SensorGnss receiver # The selected receiver's sensor_gnss sample with the corrected timestamp_sample

float32[3] antenna_offset # [m] [@frame FRD] Antenna position of the selected receiver (SENS_GNSSn_OFFX/Y/Z)
```

:::
