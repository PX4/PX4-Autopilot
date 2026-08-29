---
pageClass: is-wide-page
---

# SensorGyroFifo (UORB message)

**TOPICS:** sensor_gyro_fifo

## Fields

| Name                                              | Type        | Unit [Frame] | Range/Enum | Description                                                               |
| ------------------------------------------------- | ----------- | ------------ | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp               | `uint64`    |              |            | time since system start (microseconds)                                    |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`    |              |            |
| <a id="fld_device_id"></a>device_id               | `uint32`    |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_is_external"></a>is_external           | `bool`      |              |            | true if the sensor is not mounted on the flight controller board itself   |
| <a id="fld_dt"></a>dt                             | `float32`   |              |            | delta time between samples (microseconds)                                 |
| <a id="fld_scale"></a>scale                       | `float32`   |              |            | rad/s per count                                                           |
| <a id="fld_samples"></a>samples                   | `uint8`     |              |            | number of valid samples                                                   |
| <a id="fld_x"></a>x                               | `int16[32]` |              |            | FRD board frame X-axis raw counts (rad/s = x \* scale)                    |
| <a id="fld_y"></a>y                               | `int16[32]` |              |            | FRD board frame Y-axis raw counts (rad/s = y \* scale)                    |
| <a id="fld_z"></a>z                               | `int16[32]` |              |            | FRD board frame Z-axis raw counts (rad/s = z \* scale)                    |

## Constants

| Name                                            | Type    | Value | Description |
| ----------------------------------------------- | ------- | ----- | ----------- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 4     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGyroFifo.msg)

::: details Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles
bool is_external          # true if the sensor is not mounted on the flight controller board itself

float32 dt                # delta time between samples (microseconds)
float32 scale             # rad/s per count

uint8 samples             # number of valid samples

int16[32] x               # FRD board frame X-axis raw counts (rad/s = x * scale)
int16[32] y               # FRD board frame Y-axis raw counts (rad/s = y * scale)
int16[32] z               # FRD board frame Z-axis raw counts (rad/s = z * scale)

uint8 ORB_QUEUE_LENGTH = 4
```

:::
