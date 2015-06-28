#
# Board-specific definitions for the S2740VC
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4
CONFIG_BOARD		 = S2740VC_V1

WUSEPACKED = -Wno-packed
include $(PX4_MK_DIR)nuttx/toolchain_gnu-arm-eabi.mk

#
# Bring in common uavcan hardware version definitions
#
include $(PX4_MK_DIR)nuttx/uavcan_board_s2740vc-v1.mk
