# State of MAVLink CAN forwarding (MAV_CMD_CAN_FORWARD), published by the uavcan driver
# on every state change and at 1 Hz while active

uint64 timestamp		# [us] Time since system start

uint8 STATE_IDLE = 0		# No forwarding
uint8 STATE_ACTIVE = 1		# Bus to GCS and GCS to bus
uint8 STATE_MONITOR_ONLY = 2	# Bus to GCS only, vehicle armed or prearmed

uint8 state			# [@enum STATE]
uint8 bus			# CAN interface, 0-based, valid unless idle
uint8 num_buses			# Number of CAN interfaces
uint32 requester_sysid		# MAVLink system ID of the client that requested forwarding
uint8 requester_compid		# MAVLink component ID of the client that requested forwarding
uint16 num_filter_ids		# Number of DroneCAN data type IDs in the filter, 0 forwards everything

uint32 frames_to_gcs		# Frames forwarded to the GCS
uint32 frames_from_gcs		# Frames from the GCS put on the bus
uint32 dropped_blocked		# Frames from the GCS dropped: vehicle armed or prearmed, or no session
uint32 dropped_stale		# Frames from the GCS dropped: waited too long for transmission
uint32 dropped_queue		# Batches of frames from the GCS lost to a uORB queue overrun
uint32 dropped_inject		# Frames from the GCS not fed to the local node: queue full
