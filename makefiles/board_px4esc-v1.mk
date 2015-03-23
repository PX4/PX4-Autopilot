#
# Board-specific definitions for the PX4ESC
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM3
CONFIG_BOARD			 = PX4ESC_V1

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
