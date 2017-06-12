#
# Board-specific definitions for the PX4FMUv4PRO
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = PX4FMU_V4PRO

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
