# Debug topic for ranging beacons
#

uint32 MESSAGE_VERSION = 0

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)
uint8 beacon_id
uint32 range

float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt			# Altitude AMSL, (meters)
float32 alt_ellipsoid		# Altitude above ellipsoid, (meters)

uint32 hacc
uint32 vacc

float32 rssi_node
float32 rssi_beacon
uint8 battery_status
uint8 sequence
uint32 carrier_freq
float32 range_accuracy


# TOPICS ranging_beacon
