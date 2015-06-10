#
# Board-specific definitions for the PX4CANNODE
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM3
CONFIG_BOARD		 = PX4CANNODE_V1

WUSEPACKED = -Wno-packed
include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
