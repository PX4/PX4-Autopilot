# TimesyncStatus (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TimesyncStatus.msg)

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
