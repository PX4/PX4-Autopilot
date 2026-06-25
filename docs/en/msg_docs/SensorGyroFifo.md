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
| <a id="fld_dt"></a>dt                             | `float32`   |              |            | delta time between samples (microseconds)                                 |
| <a id="fld_scale"></a>scale                       | `float32`   |              |            |
| <a id="fld_samples"></a>samples                   | `uint8`     |              |            | number of valid samples                                                   |
| <a id="fld_x"></a>x                               | `int16[32]` |              |            | angular velocity in the FRD board frame X-axis in rad/s                   |
| <a id="fld_y"></a>y                               | `int16[32]` |              |            | angular velocity in the FRD board frame Y-axis in rad/s                   |
| <a id="fld_z"></a>z                               | `int16[32]` |              |            | angular velocity in the FRD board frame Z-axis in rad/s                   |

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

float32 dt                # delta time between samples (microseconds)
float32 scale

uint8 samples             # number of valid samples

int16[32] x               # angular velocity in the FRD board frame X-axis in rad/s
int16[32] y               # angular velocity in the FRD board frame Y-axis in rad/s
int16[32] z               # angular velocity in the FRD board frame Z-axis in rad/s

uint8 ORB_QUEUE_LENGTH = 4
```

:::
