---
pageClass: is-wide-page
---

# SensorsStatusGnss (UORB message)

Per-receiver GNSS health and check diagnostics.

Published by the sensors module. Describes the latest sample of each receiver: to gate on a sample, use vehicle_gnss.usable.

**TOPICS:** sensors_status_gnss

## Fields

| 명칭                                                                                                            | 형식           | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                                                                                 |
| ------------------------------------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                                           | `uint64`     | us                                                               |            | Time since system start                                                                                                                                                                                                            |
| <a id="fld_device_id_selected"></a>device_id_selected               | `uint32`     |                                                                  |            | Receiver that vehicle_gnss carries                                                                                                                                                                            |
| <a id="fld_device_ids"></a>device_ids                                                    | `uint32[4]`  |                                                                  |            | One entry per receiver, 0 where unused                                                                                                                                                                                             |
| <a id="fld_order"></a>order                                                                                   | `int8[4]`    |                                                                  |            | Fixed position of the receiver: 0 for the preferred one, otherwise for the first to publish, then in order of first publication. Changes only with the preference, -1 until the receiver publishes |
| <a id="fld_healthy"></a>healthy                                                                               | `bool[4]`    |                                                                  |            | Publishing, and has passed its checks for long enough                                                                                                                                                                              |
| <a id="fld_failed_checks"></a>failed_checks                                              | `uint16[4]`  |                                                                  |            | Bitmask of the enabled checks that failed, VehicleGnss CHECK\_\* bits                                                                                                                                        |
| <a id="fld_strict"></a>strict                                                                                 | `bool[4]`    |                                                                  |            | The pre-flight thresholds apply rather than the relaxed in-flight ones                                                                                                                                                             |
| <a id="fld_drift_rate_horizontal"></a>drift_rate_horizontal         | `float32[4]` | m/s                                                              |            | Horizontal position drift rate at rest                                                                                                                                                                                             |
| <a id="fld_drift_rate_vertical"></a>drift_rate_vertical             | `float32[4]` | m/s                                                              |            | Vertical position drift rate at rest                                                                                                                                                                                               |
| <a id="fld_speed_horizontal_filtered"></a>speed_horizontal_filtered | `float32[4]` | m/s                                                              |            | Filtered horizontal speed at rest                                                                                                                                                                                                  |

## Constants

| 명칭                                                             | 형식      | Value | 설명                                |
| -------------------------------------------------------------- | ------- | ----- | --------------------------------- |
| <a id="#MAX_RECEIVERS"></a> MAX_RECEIVERS | `uint8` | 4     | Length of the per-receiver arrays |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorsStatusGnss.msg)

:::details
Click here to see original file

```c
# Per-receiver GNSS health and check diagnostics
#
# Published by the sensors module. Describes the latest sample of each receiver: to gate on a sample, use vehicle_gnss.usable.

uint64 timestamp # [us] Time since system start

uint32 device_id_selected # [-] Receiver that vehicle_gnss carries

uint8 MAX_RECEIVERS = 4 # Length of the per-receiver arrays

uint32[4] device_ids                 # [-] One entry per receiver, 0 where unused
int8[4] order                        # [-] Fixed position of the receiver: 0 for the preferred one, otherwise for the first to publish, then in order of first publication. Changes only with the preference, -1 until the receiver publishes
bool[4] healthy                      # Publishing, and has passed its checks for long enough
uint16[4] failed_checks              # [-] Bitmask of the enabled checks that failed, VehicleGnss CHECK_* bits
bool[4] strict                       # The pre-flight thresholds apply rather than the relaxed in-flight ones
float32[4] drift_rate_horizontal     # [m/s] Horizontal position drift rate at rest
float32[4] drift_rate_vertical       # [m/s] Vertical position drift rate at rest
float32[4] speed_horizontal_filtered # [m/s] Filtered horizontal speed at rest

# TOPICS sensors_status_gnss
```

:::
