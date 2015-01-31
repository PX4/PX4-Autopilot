#
# Makefile for the mavstation_default configuration
#

#
# Use the configuration's ROMFS
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/mavstation_common

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/stm32
MODULES		+= drivers/boards/mavstation
MODULES		+= modules/mavstation_firmware
#MODULES		+= modules/uORB
#MODULES		+= examples/px4_simple_app

#MODULES		+= systemcmds/nshterm


define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

BUILTIN_COMMANDS := \
    $(call _B, hello, , 512, hello_main) \
    $(call _B, sercon, , 1024,  sercon_main) \
	$(call _B, serdis, , 1024,  serdis_main)
