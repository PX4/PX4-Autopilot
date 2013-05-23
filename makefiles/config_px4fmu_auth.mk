#
# Makefile for the px4fmu_default configuration
#

#
# Use the configuration's ROMFS.
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/px4fmu_common

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/stm32
MODULES		+= drivers/stm32/adc
MODULES		+= drivers/stm32/tone_alarm
MODULES		+= drivers/led
MODULES		+= drivers/px4fmu
MODULES		+= drivers/boards/px4fmu
MODULES		+= drivers/ardrone_interface
MODULES		+= drivers/mpu6000
MODULES		+= drivers/hmc5883
MODULES		+= drivers/ms5611
MODULES		+= drivers/mb12xx
MODULES		+= drivers/gps
MODULES		+= drivers/hil
MODULES		+= drivers/hott_telemetry
MODULES		+= modules/sensors

#
# System commands
#
MODULES		+= systemcmds/auth
MODULES		+= systemcmds/reboot

#
# General system control
#
MODULES		+= modules/commander
MODULES		+= modules/mavlink
MODULES		+= modules/mavlink_onboard

#
# Logging
#
MODULES		+= modules/sdlog

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/mathlib
MODULES		+= modules/controllib
MODULES		+= modules/libtomfastmath
MODULES		+= modules/libtomcrypt
MODULES		+= modules/uORB

#
# Libraries
#
LIBRARIES	+= modules/mathlib/CMSIS

#
# Demo apps
#
#MODULES		+= examples/math_demo
# Tutorial code from
# https://pixhawk.ethz.ch/px4/dev/hello_sky
#MODULES		+= examples/px4_simple_app

# Tutorial code from
# https://pixhawk.ethz.ch/px4/dev/daemon
#MODULES		+= examples/px4_daemon_app

# Tutorial code from
# https://pixhawk.ethz.ch/px4/dev/debug_values
#MODULES		+= examples/px4_mavlink_debug

# Tutorial code from
# https://pixhawk.ethz.ch/px4/dev/example_fixedwing_control
MODULES			+= examples/fixedwing_control

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
