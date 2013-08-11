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
MODULES		+= drivers/px4io
MODULES		+= drivers/boards/px4fmu-v2
MODULES		+= drivers/rgbled
MODULES		+= drivers/lsm303d
MODULES		+= drivers/l3gd20
MODULES		+= drivers/hmc5883
MODULES		+= drivers/ms5611
MODULES		+= drivers/mb12xx
MODULES		+= drivers/gps
MODULES		+= drivers/hil
MODULES		+= drivers/hott/hott_telemetry
MODULES		+= drivers/hott/hott_sensors
MODULES		+= drivers/blinkm
MODULES		+= drivers/airspeed
MODULES		+= drivers/ets_airspeed
MODULES		+= drivers/meas_airspeed
MODULES		+= modules/sensors

# Needs to be burned to the ground and re-written; for now,
# just don't build it.
#MODULES		+= drivers/mkblctrl

#
# System commands
#
MODULES		+= systemcmds/ramtron
MODULES		+= systemcmds/bl_update
MODULES		+= systemcmds/boardinfo
MODULES		+= systemcmds/mixer
MODULES		+= systemcmds/param
MODULES		+= systemcmds/perf
MODULES		+= systemcmds/preflight_check
MODULES		+= systemcmds/pwm
MODULES		+= systemcmds/reboot
MODULES		+= systemcmds/top
MODULES		+= systemcmds/tests

#
# General system control
#
MODULES		+= modules/commander
MODULES		+= modules/mavlink
MODULES		+= modules/mavlink_onboard

#
# Estimation modules (EKF / other filters)
#
MODULES		+= modules/attitude_estimator_ekf
MODULES		+= modules/position_estimator_mc
MODULES		+= modules/position_estimator
MODULES		+= modules/att_pos_estimator_ekf

#
# Vehicle Control
#
MODULES		+= modules/fixedwing_backside
MODULES		+= modules/fixedwing_att_control
MODULES		+= modules/fixedwing_pos_control
MODULES		+= modules/multirotor_att_control
MODULES		+= modules/multirotor_pos_control

#
# Logging
#
MODULES		+= modules/sdlog2

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/mathlib
MODULES		+= modules/mathlib/math/filter
MODULES		+= modules/controllib
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
