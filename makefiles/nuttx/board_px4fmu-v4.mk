#
# Board-specific definitions for the PX4FMUv4
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = PX4FMU_V4

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
