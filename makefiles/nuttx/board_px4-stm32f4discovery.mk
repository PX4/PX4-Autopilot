#
# Board-specific definitions for the PX4_STM32F4DISCOVERY
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = PX4_STM32F4DISCOVERY

include $(PX4_MK_DIR)/nuttx/toolchain_gnu-arm-eabi.mk
