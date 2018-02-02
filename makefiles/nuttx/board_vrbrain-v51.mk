#
# Board-specific definitions for the VRBRAINv51
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = VRBRAIN_V51

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
