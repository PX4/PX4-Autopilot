---
pageClass: is-wide-page
---

# TimesyncStatus (UORB message)

**TOPICS:** timesync_status

## Fields

| Назва                                                     | Тип      | Unit [Frame] | Range/Enum | Опис                                                                                          |
| --------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ---------- | --------------------------------------------------------------------------------------------- |
| timestamp                                                 | `uint64` |                                                                  |            | time since system start (microseconds)                                     |
| source_protocol                      | `uint8`  |                                                                  |            | timesync source                                                                               |
| remote_timestamp                     | `uint64` |                                                                  |            | remote system timestamp (microseconds)                                     |
| observed_offset                      | `int64`  |                                                                  |            | raw time offset directly observed from this timesync packet (microseconds) |
| estimated_offset                     | `int64`  |                                                                  |            | smoothed time offset between companion system and PX4 (microseconds)       |
| round_trip_time | `uint32` |                                                                  |            | round trip time of this timesync packet (microseconds)                     |

## Constants

| Назва                                                                                                     | Тип     | Значення | Опис |
| --------------------------------------------------------------------------------------------------------- | ------- | -------- | ---- |
| <a href="#SOURCE_PROTOCOL_UNKNOWN"></a> SOURCE_PROTOCOL_UNKNOWN | `uint8` | 0        |      |
| <a href="#SOURCE_PROTOCOL_MAVLINK"></a> SOURCE_PROTOCOL_MAVLINK | `uint8` | 1        |      |
| <a href="#SOURCE_PROTOCOL_DDS"></a> SOURCE_PROTOCOL_DDS         | `uint8` | 2        |      |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TimesyncStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp			# time since system start (microseconds)

uint8 SOURCE_PROTOCOL_UNKNOWN = 0
uint8 SOURCE_PROTOCOL_MAVLINK = 1
uint8 SOURCE_PROTOCOL_DDS     = 2
uint8 source_protocol			# timesync source

uint64 remote_timestamp			# remote system timestamp (microseconds)
int64 observed_offset			# raw time offset directly observed from this timesync packet (microseconds)
int64 estimated_offset			# smoothed time offset between companion system and PX4 (microseconds)
uint32 round_trip_time			# round trip time of this timesync packet (microseconds)
```

:::
