uint8 len			# length of data
uint8 flags         		# LSB: 1=fragmented
uint8[182] data		# data to write to GPS device (RTCM message)
