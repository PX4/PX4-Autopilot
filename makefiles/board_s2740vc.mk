#
# Board-specific definitions for the S2740VC
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4
CONFIG_BOARD		 = S2740VC

WUSEPACKED = -Wno-packed
include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
