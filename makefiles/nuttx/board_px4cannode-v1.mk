#
# Board-specific definitions for the PX4CANNODE
#

#
# Configure the toolchain
#

CONFIG_ARCH			 = CORTEXM3
CONFIG_BOARD		 = PX4CANNODE_V1

WUSEPACKED = -Wno-packed
include $(PX4_MK_DIR)nuttx/toolchain_gnu-arm-eabi.mk

#
# Bring in common uavcan hardware version definitions
#
include $(PX4_MK_DIR)nuttx/uavcan_board_px4cannode-v1.mk

