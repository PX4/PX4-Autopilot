#
# Board-specific definitions for the Linux port of PX4
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = HEXAGON
CONFIG_BOARD			 = QURTTEST

include $(PX4_MK_DIR)/qurt/toolchain_hexagon.mk
