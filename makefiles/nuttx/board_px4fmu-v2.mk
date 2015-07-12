#
# Board-specific definitions for the PX4FMUv2
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = PX4FMU_V2

include $(PX4_MK_DIR)/nuttx/toolchain_gnu-arm-eabi.mk
