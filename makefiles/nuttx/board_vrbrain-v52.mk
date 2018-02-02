#
# Board-specific definitions for the VRBRAINv52
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = VRBRAIN_V52

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
