uint64 timestamp	# time since system start (microseconds)

uint16 status 	# Status bitmap 1: Roaming is active
uint8 type 	# Cellular network radio type 0: none 1: gsm 2: cdma 3: wcdma 4: lte
uint8 quality	# Cellular network RSSI/RSRP in dBm, absolute value
uint16 mcc	# Mobile country code. If unknown, set to: UINT16_MAX
uint16 mnc	# Mobile network code. If unknown, set to: UINT16_MAX
uint16 lac	# Location area code. If unknown, set to: 0
uint32 cid	# Cell ID. If unknown, set to: UINT32_MAX
