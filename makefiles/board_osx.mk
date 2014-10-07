#
# Pseudo-board definition for native component build on Mac OS X.
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = x86_64
CONFIG_BOARD			 = OSX

include $(PX4_MK_DIR)/toolchain_clang-osx.mk
