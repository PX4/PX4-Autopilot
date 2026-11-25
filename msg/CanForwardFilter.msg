# DroneCAN data type filter for MAVLink CAN forwarding
#
# Received from a MAVLink client via CAN_FILTER_MODIFY and published by the mavlink module.
# Subscribed by the uavcan driver to limit which bus frames are forwarded to the client.
# Until a filter is set, all frames on the forwarded bus are sent to the client.

uint64 timestamp # [us] Time since system start

uint8 bus # [-] CAN interface, 0-based

uint8 operation # [@enum OPERATION] How ids is applied to the current filter
uint8 OPERATION_REPLACE = 0 # Replace the filter. An empty list forwards everything
uint8 OPERATION_ADD = 1 # Add the IDs to the filter
uint8 OPERATION_REMOVE = 2 # Remove the IDs from the filter

uint8 num_ids # [-] [@range 0, 16] Number of valid entries in ids
uint16[16] ids # [-] DroneCAN data type IDs. 0 matches anonymous frames

uint8 ORB_QUEUE_LENGTH = 4
