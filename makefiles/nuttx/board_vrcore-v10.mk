#
# Board-specific definitions for the VRCOREv10
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = VRCORE_V10

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
