#
# Makefile for the px4cannode_bootloader configuration
#

#
# Board support modules
#
MODULES		+= drivers/boards/px4cannode-v1/bootloader

SRC_SEARCH = 	can src common uavcan		

INCLUDE_DIRS += $(addprefix $(PX4_MODULE_SRC)$(MODULES)/,$(SRC_SEARCH))

 
