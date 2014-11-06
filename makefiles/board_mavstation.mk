#
# Board-specific definitions for the mavstation
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM3
CONFIG_BOARD			 = MAVSTATION

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
