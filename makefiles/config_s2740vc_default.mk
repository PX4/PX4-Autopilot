#
# Makefile for the s2740vc_default configuration
#

#
# Board support modules
#
MODULES		+= drivers/stm32
MODULES		+= drivers/stm32/adc
MODULES		+= drivers/led
MODULES		+= drivers/boards/s2740vc

#
# System commands
#
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
MODULES		+= systemcmds/config
MODULES		+= systemcmds/ver

#
# General system control
#
MODULES		+= modules/uavcannode

#
# Library modules
#
MODULES		+= modules/systemlib



#
# Unit tests
#
#MODULES 	+= modules/unit_test
#MODULES 	+= modules/commander/commander_tests

# Generate parameter XML file
GEN_PARAM_XML = 1


#
# Demo apps
#

#
# Transitional support - add commands from the NuttX export archive.
#
# In general, these should move to modules over time.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint> but we use a helper macro
# to make the table a bit more readable.
#
define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

BUILTIN_COMMANDS := \
    $(call _B, null, , 60, null_main)

