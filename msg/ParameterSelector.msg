# This message is used to notify the system about the parameter selector state

uint64 timestamp		# time since system start (microseconds)

uint32 instance			# Instance count - constantly incrementing
uint8 state			# state

uint8 DISABLED = 0		# Disabled
uint8 ALT_FAST = 1		# Altitude Fast
uint8 ALT_MEDIUM = 2		# Altitude Medium
uint8 ALT_SLOW = 3		# Altitude Slow
uint8 ACRO_FAST = 4		# Acro Fast
uint8 RESERVED5 = 5		# Reserved
uint8 STATE_COUNT = 6		# Counter - keep it as last element!

char[8] short_string		# Short string that describes the parameter selector state. For example printing to an OSD with limited characters
