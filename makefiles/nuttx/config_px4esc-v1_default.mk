#
# Makefile for the px4esc-v1_default configuration
#

# WhenFLASH_BASED_PARAMS is defined PARAMETER_BUFFER_SIZE must be defined larger
# then the maximum parameter memory needed to commit the recored + ~20 bytes. 
# For the syslib's parameter this would be the size of the bson representations 
# of the data

EXTRADEFINES+= -DFLASH_BASED_PARAMS -DPARAM_NO_ORB -DPARAMETER_BUFFER_SIZE=1024
INCLUDE_DIRS += $(PX4_BOOTLOADER_BASE)include


#
# UAVCAN boot loadable Module ID
#

export UAVCANBLID_SW_VERSION_MAJOR=0
export UAVCANBLID_SW_VERSION_MINOR=1

#
# Bring in common uavcan hardware version definitions
#
include $(PX4_MK_DIR)nuttx/uavcan_board_px4esc-v1.mk

#
# Board support modules
#
MODULES		+= drivers/stm32
MODULES		+= drivers/stm32/adc
MODULES		+= drivers/led
MODULES		+= drivers/boards/px4esc-v1

#
# System commands
#
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
MODULES		+= systemcmds/config
MODULES		+= systemcmds/ver
MODULES		+= systemcmds/param

#
# General system control
#
MODULES		+= modules/uavcanesc
MODULES		+= modules/uavcanesc/nshterm
MODULES		+= modules/uavcanesc/commands/cfg
MODULES		+= modules/uavcanesc/commands/selftest
MODULES		+= modules/uavcanesc/commands/dc
MODULES		+= modules/uavcanesc/commands/rpm
MODULES		+= modules/uavcanesc/commands/stat

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/flashparams



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
MAKE_UAVCAN_BOOT_LOADABLE_ID=$(call MKUAVCANBLNAME,$(subst $\",,$(UAVCANBLID_NAME)))

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
	$(call _B, sercon,                 ,                          2048,  sercon_main                ) \
	$(call _B, serdis,                 ,                          2048,  serdis_main                )

