# Ranging beacon measurement data (e.g. LoRa, UWB)

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)
uint8 beacon_id
float32 range			# Range measurement (m)

float64 lat			# Latitude (degrees)
float64 lon			# Longitude (degrees)
float32 alt			# Altitude AMSL (m)
float32 alt_ellipsoid		# Altitude above ellipsoid (m)

float32 hacc			# Groundbeacon horizontal accuracy (m)
float32 vacc			# Groundbeacon vertical accuracy (m)

int8 rssi_node			# RSSI at node/vehicle side (dBm)
int8 rssi_beacon		# RSSI at beacon/station side (dBm)
uint8 battery_status		# Beacon battery (%, 255=unknown)
uint8 sequence
uint16 carrier_freq		# Carrier frequency (MHz)
float32 range_accuracy		# Range accuracy estimate (m)


# TOPICS ranging_beacon
