# Batch of classic CAN frames tunneled over MAVLink
#
# Carries frames of a DroneCAN over MAVLink session (MAV_CMD_CAN_FORWARD, CAN_FRAME, CANFD_FRAME).
# can_forward_rx: frames from the CAN bus to the MAVLink client, published by the uavcan driver and subscribed by the mavlink module.
# can_forward_tx: frames from the MAVLink client to the CAN bus, published by the mavlink module and subscribed by the uavcan driver.
# Frames are batched to keep the publish rate low during a firmware update burst.

uint8 MAX_FRAMES = 16 # Maximum number of frames per batch

uint64 timestamp # [us] Time since system start

uint8 bus # [-] CAN interface, 0-based
uint8 count # [-] [@range 0, 16] Number of valid frames in this batch

uint32[16] id # [-] CAN ID. Bit 31 is set for an extended (29 bit) frame
uint8[16] len # [-] [@range 0, 8] Number of data bytes per frame
uint8[128] data # [-] Frame payloads, frame i uses data[i * 8 .. i * 8 + len[i] - 1]

uint8 ORB_QUEUE_LENGTH = 8

# TOPICS can_forward_rx can_forward_tx
