#
# Makefile for the mavstation_default configuration
#

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/stm32
MODULES		+= drivers/boards/mavstation
MODULES		+= modules/mavstation_firmware
#MODULES		+= modules/systemlib
#MODULES		+= modules/controllib
MODULES		+= modules/uORB
#MODULES		+= modules/dataman
MODULES		+= examples/px4_simple_app


define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

BUILTIN_COMMANDS := \
    $(call _B, hello, , 512, hello_main)