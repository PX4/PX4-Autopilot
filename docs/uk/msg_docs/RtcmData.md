---
pageClass: is-wide-page
---

# RtcmData (UORB message)

RTCM3 data exchanged with GNSS receivers.

Published under two topic names that share this definition (see TOPICS below):

rtcm_corrections - external fixed-base corrections fed into the vehicle (MAVLink
GPS_RTCM_DATA, UAVCAN RTCMStream, GPS drivers in dump mode). Multiple
sources are allowed, one uORB instance each; consumers select an instance
via their stale-link logic.

rtcm_moving_baseline - moving-base GPS output (RTCM 4072 or equivalent) intended for a rover.
Single publisher per vehicle (on-board moving base, or a CANnode
forwarding MovingBaselineData); consumers only read instance 0.

**TOPICS:** rtcm_corrections rtcm_moving_baseline

## Fields

| Назва                                                    | Тип          | Unit [Frame] | Range/Enum | Опис                                                                                                    |
| -------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                      | `uint64`     | us                                                               |            | Time since system start                                                                                 |
| <a id="fld_device_id"></a>device_id | `uint32`     |                                                                  |            | Unique device ID of the publisher that produced this RTCM                                               |
| <a id="fld_len"></a>len                                  | `uint16`     |                                                                  |            | Length of data                                                                                          |
| <a id="fld_flags"></a>flags                              | `uint8`      |                                                                  |            | LSB: 1=fragmented                                                                       |
| <a id="fld_data"></a>data                                | `uint8[300]` |                                                                  |            | Correction payload (fixed-base RTCM3 and/or SPARTN frames, or moving-baseline RTCM3) |

## Constants

| Назва                                                                                     | Тип     | Значення | Опис |
| ----------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a id="#ORB_QUEUE_LENGTH"></a> ORB_QUEUE_LENGTH | `uint8` | 16       |      |
| <a id="#MAX_INSTANCES"></a> MAX_INSTANCES                            | `uint8` | 4        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RtcmData.msg)

:::details
Click here to see original file

```c
# RTCM3 data exchanged with GNSS receivers.
#
# Published under two topic names that share this definition (see TOPICS below):
#
# rtcm_corrections - external fixed-base corrections fed into the vehicle (MAVLink
# GPS_RTCM_DATA, UAVCAN RTCMStream, GPS drivers in dump mode). Multiple
# sources are allowed, one uORB instance each; consumers select an instance
# via their stale-link logic.
#
# rtcm_moving_baseline - moving-base GPS output (RTCM 4072 or equivalent) intended for a rover.
# Single publisher per vehicle (on-board moving base, or a CANnode
# forwarding MovingBaselineData); consumers only read instance 0.

uint64 timestamp # [us] Time since system start

uint32 device_id # [-] Unique device ID of the publisher that produced this RTCM

uint16 len # [-] Length of data
uint8 flags # [-] LSB: 1=fragmented
uint8[300] data # Correction payload (fixed-base RTCM3 and/or SPARTN frames, or moving-baseline RTCM3)

uint8 ORB_QUEUE_LENGTH = 16

# Sized for the fixed-base corrections case (up to four independent sources). The moving-baseline
# topic only uses instance 0 (single publisher per vehicle).
uint8 MAX_INSTANCES = 4

# TOPICS rtcm_corrections rtcm_moving_baseline
```

:::
