#
# Makefile for the s2740vc_bootloader configuration
#

#
# Board support modules
#
MODULES		+= drivers/boards/s2740vc/bootloader

SRC_SEARCH = 	can src common uavcan

INCLUDE_DIRS += $(addprefix $(PX4_MODULE_SRC)$(MODULES)/,$(SRC_SEARCH))

