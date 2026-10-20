uint64 timestamp # Local monotonic reception time (microseconds)
uint8 mavlink_channel # Input channel, used to avoid echoing configuration
uint8[20] id_or_mac	# Only used for drone ID data received from other UAs, no null termination, null filled if shorter
uint8 id_type		# MAV_ODID_ID_TYPE: indicates the format for the uas_id field
uint8 ua_type		# MAV_ODID_UA_TYPE: indicates the type of UA (Unmanned Aircraft)
uint8[20] uas_id	# UAS (Unmanned Aircraft System) ID following the format specified by id_type, no null termination, null filled if shorter
