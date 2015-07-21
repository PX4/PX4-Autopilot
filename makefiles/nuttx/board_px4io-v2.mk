#
# Board-specific definitions for the PX4IOv2
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM3
CONFIG_BOARD			 = PX4IO_V2

include $(PX4_MK_DIR)/nuttx/toolchain_gnu-arm-eabi.mk
