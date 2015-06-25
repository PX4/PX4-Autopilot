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

export UAVCANBLID_HW_VERSION_MAJOR=1
export UAVCANBLID_HW_VERSION_MINOR=0
export UAVCANBLID_NAME= "\"org.pixhawk.px4cannode-v1\""

