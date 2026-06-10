---
pageClass: is-wide-page
---

# GpsInjectData (UORB message)

**TOPICS:** gps_inject_data

## Fields

| Name                                | Type         | Unit [Frame] | Range/Enum | Description                                                               |
| ----------------------------------- | ------------ | ------------ | ---------- | ------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp | `uint64`     |              |            | time since system start (microseconds)                                    |
| <a id="fld_device_id"></a>device_id | `uint32`     |              |            | unique device ID for the sensor that does not change between power cycles |
| <a id="fld_len"></a>len             | `uint16`     |              |            | length of data                                                            |
| <a id="fld_flags"></a>flags         | `uint8`      |              |            | LSB: 1=fragmented across multiple uORB publications                       |
| <a id="fld_data"></a>data           | `uint8[300]` |              |            | data chunk to write to GPS device (RTCM message)                          |

## Constants

| Name                                            | Type    | Value | Description |
| ----------------------------------------------- | ------- | ----- | ----------- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 8     |
| <a id="#MAX_INSTANCES"></a> MAX_INSTANCES       | `uint8` | 2     |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/GpsInjectData.msg)

::: details Click here to see original file

```c
uint64 timestamp		# time since system start (microseconds)

uint32 device_id                # unique device ID for the sensor that does not change between power cycles

uint16 len                       # length of data
uint8 flags                     # LSB: 1=fragmented across multiple uORB publications
uint8[300] data                 # data chunk to write to GPS device (RTCM message)

uint8 ORB_QUEUE_LENGTH = 8

uint8 MAX_INSTANCES = 2
```

:::
