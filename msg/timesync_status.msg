uint64 timestamp			# time since system start (microseconds)

uint8 SOURCE_PROTOCOL_MAVLINK = 0
uint8 SOURCE_PROTOCOL_RTPS = 1
uint8 source_protocol			# timesync source. Source can be MAVLink or the microRTPS bridge

uint64 remote_timestamp			# remote system timestamp (microseconds)
int64 observed_offset			# raw time offset directly observed from this timesync packet (microseconds)
int64 estimated_offset			# smoothed time offset between companion system and PX4 (microseconds)
uint32 round_trip_time			# round trip time of this timesync packet (microseconds)
