#
# Board-specific definitions for the PX4ESC
#

#
# Configure the toolchain
#

CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD		 = PX4ESC_V1

WUSEPACKED = -Wno-packed
include $(PX4_MK_DIR)nuttx/toolchain_gnu-arm-eabi.mk

#
# Bring in common uavcan hardware version definitions
#
include $(PX4_MK_DIR)nuttx/uavcan_board_px4esc-v1.mk

