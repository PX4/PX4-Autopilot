#
# Board-specific definitions for the Gumstix AeroCore
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = AEROCORE

include $(PX4_MK_DIR)/nuttx/toolchain_gnu-arm-eabi.mk
