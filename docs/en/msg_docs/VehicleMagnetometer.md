---
pageClass: is-wide-page
---

# VehicleMagnetometer (UORB message)

**TOPICS:** vehicle_magnetometer

## Fields

| Name                                                | Type         | Unit [Frame] | Range/Enum | Description                                                                        |
| --------------------------------------------------- | ------------ | ------------ | ---------- | ---------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                 | `uint64`     |              |            | time since system start (microseconds)                                             |
| <a id="fld_timestamp_sample"></a>timestamp_sample   | `uint64`     |              |            | the timestamp of the raw data (microseconds)                                       |
| <a id="fld_device_id"></a>device_id                 | `uint32`     |              |            | unique device ID for the selected magnetometer                                     |
| <a id="fld_magnetometer_ga"></a>magnetometer_ga     | `float32[3]` |              |            | Magnetic field in the FRD body frame XYZ-axis in Gauss                             |
| <a id="fld_calibration_count"></a>calibration_count | `uint8`      |              |            | Calibration changed counter. Monotonically increases whenever calibration changes. |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleMagnetometer.msg)

::: details Click here to see original file

```c
uint64 timestamp            # time since system start (microseconds)

uint64 timestamp_sample     # the timestamp of the raw data (microseconds)

uint32 device_id            # unique device ID for the selected magnetometer

float32[3] magnetometer_ga  # Magnetic field in the FRD body frame XYZ-axis in Gauss

uint8 calibration_count     # Calibration changed counter. Monotonically increases whenever calibration changes.
```

:::
