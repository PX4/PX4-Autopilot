# DroneCAN over MAVLink session status
#
# State of the CAN forwarding session opened by a MAVLink client with MAV_CMD_CAN_FORWARD.
# Published by the uavcan driver on change. Subscribed by the mavlink module to route frames to the owning link, and by commander to block arming while a session is active.

uint64 timestamp # [us] Time since system start

bool active # A client forwards a CAN bus. Arming is blocked meanwhile
uint8 bus # [-] CAN interface, 0-based
uint32 client_system # [-] MAVLink system ID of the client
uint16 client_component # [-] MAVLink component ID of the client
