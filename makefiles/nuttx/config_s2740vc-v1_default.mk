#
# Makefile for the s2740vc_default configuration
#

INCLUDE_DIRS += $(PX4_BOOTLOADER_BASE)include


#
# UAVCAN boot loadable Module ID
#

export UAVCANBLID_SW_VERSION_MAJOR=0
export UAVCANBLID_SW_VERSION_MINOR=1

#
# Bring in common uavcan hardware version definitions
#
include $(PX4_MK_DIR)nuttx/uavcan_board_s2740vc-v1.mk

#
# Board support modules
#
MODULES		+= drivers/stm32
MODULES		+= drivers/led
MODULES		+= drivers/boards/s2740vc-v1

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
# Make this UAVCAN boot loadable
#
# N.B. this would be uncommented when there is an APP 
#MAKE_UAVCAN_BOOT_LOADABLE_ID=$(call MKUAVCANBLNAME,$(subst $\",,$(UAVCANBLID_NAME)))


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

