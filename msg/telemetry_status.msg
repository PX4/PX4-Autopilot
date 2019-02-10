uint64 timestamp			# time since system start (microseconds)

uint8 TELEMETRY_STATUS_RADIO_TYPE_GENERIC = 0
uint8 TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO = 1
uint8 TELEMETRY_STATUS_RADIO_TYPE_UBIQUITY_BULLET = 2
uint8 TELEMETRY_STATUS_RADIO_TYPE_WIRE = 3
uint8 TELEMETRY_STATUS_RADIO_TYPE_USB = 4
uint8 TELEMETRY_STATUS_RADIO_TYPE_IRIDIUM = 5

uint64 heartbeat_time			# Time of last received heartbeat from remote system

uint8 type				# type of the radio hardware

uint8 system_id				# system id of the remote system
uint8 component_id			# component id of the remote system

uint8 mode

bool flow_control
bool forwarding
bool mavlink_v2
bool ftp

uint8 streams

float32 data_rate

float32 rate_multiplier

float32 rate_rx

float32 rate_tx
float32 rate_txerr
