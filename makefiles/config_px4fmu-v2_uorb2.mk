#
# Makefile for the px4fmu_default configuration
#

#
# Use the configuration's ROMFS, copy the px4iov2 firmware into
# the ROMFS if it's available
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/px4fmu_common
ROMFS_OPTIONAL_FILES = $(PX4_BASE)/Images/px4io-v2_default.bin

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/stm32
MODULES		+= drivers/led
MODULES		+= drivers/boards/px4fmu-v2


# Needs to be burned to the ground and re-written; for now,
# just don't build it.
#MODULES		+= drivers/mkblctrl

#
# System commands
#
MODULES		+= systemcmds/bl_update
MODULES		+= systemcmds/boardinfo
MODULES		+= systemcmds/perf
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
MODULES		+= systemcmds/config
MODULES		+= systemcmds/nshterm

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/uORB
MODULES		+= modules/uORB2

#
# Libraries
#
LIBRARIES	+= lib/mathlib/CMSIS
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/ecl
MODULES		+= lib/external_lgpl
MODULES		+= lib/geo
MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion
MODULES		+= lib/launchdetection

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

#                  command                 priority                   stack  entrypoint
BUILTIN_COMMANDS := \
	$(call _B, sercon,                 ,                          2048,  sercon_main                ) \
	$(call _B, serdis,                 ,                          2048,  serdis_main                )
