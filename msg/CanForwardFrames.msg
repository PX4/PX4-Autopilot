# Batch of classic CAN frames tunneled over MAVLink (CAN_FRAME / CANFD_FRAME, MAV_CMD_CAN_FORWARD)
#
# can_forward_rx: frames from the CAN bus to the GCS, published by the uavcan driver
# can_forward_tx: frames from the GCS to the CAN bus, published by the mavlink module
#
# Frames are batched to keep the publish rate low during a firmware update burst.

uint64 timestamp		# [us] Time since system start

uint8 MAX_FRAMES = 16

uint8 bus			# CAN interface, 0-based
uint8 count			# Number of valid frames in this batch

uint32[16] id			# CAN ID. Bit 31 is set for an extended (29 bit) frame
uint8[16] len			# Number of data bytes, 0..8
uint8[128] data			# Frame i uses data[i * 8 .. i * 8 + len[i] - 1]

uint8 ORB_QUEUE_LENGTH = 8

# TOPICS can_forward_rx can_forward_tx
