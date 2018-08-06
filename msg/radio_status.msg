
uint64 timestamp	# time since system start (microseconds)

uint8 RADIO_TYPE_GENERIC = 0
uint8 RADIO_TYPE_3DR_RADIO = 1
uint8 RADIO_TYPE_UBIQUITY_BULLET = 2
uint8 RADIO_TYPE_WIRE = 3
uint8 RADIO_TYPE_USB = 4
uint8 RADIO_TYPE_IRIDIUM = 5

uint8 type				# type of the radio hardware

uint8 rssi				# local signal strength
uint8 remote_rssi			# remote signal strength

uint8 txbuf				# how full the tx buffer is as a percentage
uint8 noise				# background noise level

uint8 remote_noise			# remote background noise level
uint16 rxerrors				# receive errors

uint16 fixed				# count of error corrected packets
