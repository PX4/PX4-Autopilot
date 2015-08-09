#
# Board-specific definitions for the ZUBAXGNSS
#

#
# Configure the toolchain
#

CONFIG_ARCH			 = CORTEXM3
CONFIG_BOARD		 = ZUBAXGNSS_V1

WUSEPACKED = -Wno-packed
include $(PX4_MK_DIR)nuttx/toolchain_gnu-arm-eabi.mk

#
# Bring in common uavcan hardware version definitions
#
include $(PX4_MK_DIR)nuttx/uavcan_board_zubaxgnss-v1.mk
