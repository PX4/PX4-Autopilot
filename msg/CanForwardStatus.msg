# DroneCAN over MAVLink session (MAV_CMD_CAN_FORWARD), published by the uavcan driver on change

uint64 timestamp		# [us] Time since system start

bool active			# A client forwards a CAN bus. Arming is blocked meanwhile
uint8 bus			# CAN interface, 0-based
uint32 client_system		# MAVLink system ID of the client
uint16 client_component		# MAVLink component ID of the client
