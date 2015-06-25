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

export UAVCANBLID_HW_VERSION_MAJOR=1
export UAVCANBLID_HW_VERSION_MINOR=0
export UAVCANBLID_NAME= "\"com.thiemar.s2740vc-v1\""
