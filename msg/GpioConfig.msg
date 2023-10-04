# GPIO configuration

uint64 timestamp			# time since system start (microseconds)
uint32 device_id			# Device id

uint32 mask				# Pin mask
uint32 state				# Initial pin output state

# Configuration Mask
# Bit 0-3: Direction: 0=Input, 1=Output
# Bit 4-7: Input Config: 0=Floating, 1=PullUp, 2=PullDown
# Bit 8-12: Output Config: 0=PushPull, 1=OpenDrain
# Bit 13-31: Reserved
uint32 INPUT			= 0	# 0x0000
uint32 OUTPUT			= 1	# 0x0001
uint32 PULLUP			= 16	# 0x0010
uint32 PULLDOWN			= 32	# 0x0020
uint32 OPENDRAIN		= 256	# 0x0100

uint32 INPUT_FLOATING		= 0	# 0x0000
uint32 INPUT_PULLUP		= 16	# 0x0010
uint32 INPUT_PULLDOWN		= 32	# 0x0020

uint32 OUTPUT_PUSHPULL		= 0	# 0x0000
uint32 OUTPUT_OPENDRAIN		= 256	# 0x0100
uint32 OUTPUT_OPENDRAIN_PULLUP	= 272	# 0x0110

uint32 config
