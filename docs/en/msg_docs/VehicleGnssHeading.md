---
pageClass: is-wide-page
---

# VehicleGnssHeading (UORB message)

GNSS heading from a dual-antenna or moving-baseline receiver.

Published by the sensors module from sensor_gnss_relative. Consumed by EKF2 independently from position and velocity.

**TOPICS:** vehicle_gnss_heading

## Fields

| Name                                              | Type      | Unit [Frame] | Range/Enum | Description                                                                                                                          |
| ------------------------------------------------- | --------- | ------------ | ---------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp               | `uint64`  |              |            | time since system start (microseconds)                                                                                               |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`  |              |            | time since system start (microseconds) - actual measurement time                                                                     |
| <a id="fld_device_id"></a>device_id               | `uint32`  |              |            | unique device ID for the sensor that does not change between power cycles                                                            |
| <a id="fld_heading"></a>heading                   | `float32` |              |            | body-frame heading rel to NED from dual antenna array (rad, [-PI, PI])                                                               |
| <a id="fld_heading_accuracy"></a>heading_accuracy | `float32` |              |            | 1-sigma heading accuracy (rad)                                                                                                       |
| <a id="fld_heading_offset"></a>heading_offset     | `float32` |              |            | yaw of the configured antenna baseline in the body frame; heading + heading_offset is the measured baseline heading (rad, [-PI, PI]) |
| <a id="fld_baseline_length"></a>baseline_length   | `float32` |              |            | antenna baseline length reported by the receiver, NaN if it doesn't report one (m)                                                   |
| <a id="fld_jamming_state"></a>jamming_state       | `uint8`   |              |            | jamming_state of the receiver providing the heading, values as in SensorGnss (0: Unknown, 1: OK, 2: Mitigated, 3: Detected)          |
| <a id="fld_spoofing_state"></a>spoofing_state     | `uint8`   |              |            | spoofing_state of the receiver providing the heading, values as in SensorGnss (0: Unknown, 1: OK, 2: Mitigated, 3: Detected)         |
| <a id="fld_usable"></a>usable                     | `bool`    |              |            | heading may be used: its receiver reports no spoofing or jamming that the enabled checks reject                                      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleGnssHeading.msg)

::: details Click here to see original file

```c
# GNSS heading from a dual-antenna or moving-baseline receiver
#
# Published by the sensors module from sensor_gnss_relative. Consumed by EKF2 independently from position and velocity.

uint64 timestamp              # time since system start (microseconds)
uint64 timestamp_sample       # time since system start (microseconds) - actual measurement time

uint32 device_id              # unique device ID for the sensor that does not change between power cycles

float32 heading               # body-frame heading rel to NED from dual antenna array (rad, [-PI, PI])
float32 heading_accuracy      # 1-sigma heading accuracy (rad)
float32 heading_offset        # yaw of the configured antenna baseline in the body frame; heading + heading_offset is the measured baseline heading (rad, [-PI, PI])
float32 baseline_length       # antenna baseline length reported by the receiver, NaN if it doesn't report one (m)

uint8 jamming_state           # jamming_state of the receiver providing the heading, values as in SensorGnss (0: Unknown, 1: OK, 2: Mitigated, 3: Detected)
uint8 spoofing_state          # spoofing_state of the receiver providing the heading, values as in SensorGnss (0: Unknown, 1: OK, 2: Mitigated, 3: Detected)
bool usable                   # heading may be used: its receiver reports no spoofing or jamming that the enabled checks reject
```

:::
