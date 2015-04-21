#
# Board-specific definitions for the POSIX port of PX4
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = NATIVE
CONFIG_BOARD			 = POSIXTEST

include $(PX4_MK_DIR)/toolchain_native.mk
