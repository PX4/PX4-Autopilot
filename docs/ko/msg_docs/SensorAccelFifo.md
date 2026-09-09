---
pageClass: is-wide-page
---

# SensorAccelFifo (UORB message)

**TOPICS:** sensor_accel_fifo

## Fields

| 명칭                                                                     | 형식          | Unit [Frame] | Range/Enum | 설명                                                                        |
| ---------------------------------------------------------------------- | ----------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                    | `uint64`    |                                                                  |            | time since system start (microseconds)                 |
| <a id="fld_timestamp_sample"></a>timestamp_sample | `uint64`    |                                                                  |            |                                                                           |
| <a id="fld_device_id"></a>device_id               | `uint32`    |                                                                  |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_is_external"></a>is_external           | `bool`      |                                                                  |            | true if the sensor is not mounted on the flight controller board itself   |
| <a id="fld_dt"></a>dt                                                  | `float32`   |                                                                  |            | delta time between samples (microseconds)              |
| <a id="fld_scale"></a>scale                                            | `float32`   |                                                                  |            | m/s^2 per count                                                           |
| <a id="fld_samples"></a>samples                                        | `uint8`     |                                                                  |            | number of valid samples                                                   |
| <a id="fld_x"></a>x                                                    | `int16[32]` |                                                                  |            | FRD board frame X-axis raw counts (m/s^2 = x \* scale) |
| <a id="fld_y"></a>y                                                    | `int16[32]` |                                                                  |            | FRD board frame Y-axis raw counts (m/s^2 = y \* scale) |
| <a id="fld_z"></a>z                                                    | `int16[32]` |                                                                  |            | FRD board frame Z-axis raw counts (m/s^2 = z \* scale) |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorAccelFifo.msg)

:::details
Click here to see original file

```c
uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles
bool is_external          # true if the sensor is not mounted on the flight controller board itself

float32 dt                # delta time between samples (microseconds)
float32 scale             # m/s^2 per count

uint8 samples             # number of valid samples

int16[32] x               # FRD board frame X-axis raw counts (m/s^2 = x * scale)
int16[32] y               # FRD board frame Y-axis raw counts (m/s^2 = y * scale)
int16[32] z               # FRD board frame Z-axis raw counts (m/s^2 = z * scale)
```

:::
