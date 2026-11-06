# DroneCAN data type filter for MAVLink CAN forwarding, from CAN_FILTER_MODIFY

uint64 timestamp		# [us] Time since system start

uint8 OPERATION_REPLACE = 0	# Replace the filter. An empty list forwards everything
uint8 OPERATION_ADD = 1		# Add the IDs to the filter
uint8 OPERATION_REMOVE = 2	# Remove the IDs from the filter

uint8 bus			# CAN interface, 0-based
uint8 operation			# [@enum OPERATION]
uint8 num_ids			# Number of valid entries in ids
uint16[16] ids			# DroneCAN data type IDs. 0 matches anonymous frames

uint8 ORB_QUEUE_LENGTH = 4
