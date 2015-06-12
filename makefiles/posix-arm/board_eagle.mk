#
# Board-specific definitions for the POSIX port of PX4
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXA8
CONFIG_BOARD			 = EAGLE

include $(PX4_MK_DIR)/posix-arm/toolchain_gnu-arm-linux-gnueabihf.mk
