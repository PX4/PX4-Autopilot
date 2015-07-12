uint8 TELEMETRY_STATUS_RADIO_TYPE_GENERIC = 0
uint8 TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO = 1
uint8 TELEMETRY_STATUS_RADIO_TYPE_UBIQUITY_BULLET = 2
uint8 TELEMETRY_STATUS_RADIO_TYPE_WIRE = 3

uint64 timestamp
uint64 heartbeat_time			# Time of last received heartbeat from remote system
uint64 telem_time			# Time of last received telemetry status packet, 0 for none
uint8 type				# type of the radio hardware
uint8 rssi				# local signal strength
uint8 remote_rssi			# remote signal strength
uint16 rxerrors				# receive errors
uint16 fixed				# count of error corrected packets
uint8 noise				# background noise level
uint8 remote_noise			# remote background noise level
uint8 txbuf				# how full the tx buffer is as a percentage
uint8 system_id				# system id of the remote system
uint8 component_id			# component id of the remote system
