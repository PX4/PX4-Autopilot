#
# Board-specific definitions for the POSIX port of PX4
# for use in SITL testing
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = NATIVE
CONFIG_BOARD			 = SITL

include $(PX4_MK_DIR)/posix/toolchain_native.mk
