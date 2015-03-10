#
# Makefile for the Foo *default* configuration
#

#
# Use the configuration's ROMFS.
#
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/px4fmu_common

#
# Board support modules
#
#MODULES		+= drivers/dxl
#MODULES		+= drivers/i2c_pwm
#MODULES		+= drivers/lemonrx
#MODULES		+= drivers/mpu9x50
#MODULES		+= drivers/um7
MODULES		+= drivers/device
MODULES		+= modules/sensors

#
# System commands
#
#MODULES	+= systemcmds/boardinfo

#
# General system control
#
#MODULES	+= modules/mavlink
MODULES		+= modules/mavlink

#
# Estimation modules (EKF/ SO3 / other filters)
#
MODULES		+= modules/attitude_estimator_ekf

#
# Vehicle Control
#
MODULES		+= modules/mc_att_control

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/uORB
MODULES		+= modules/dataman

#
# Libraries
#
MODULES		+= lib/mathlib
MODULES		+= lib/geo
MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion
#MODULES		+= lib/utils

#
# Linux port
#
MODULES		+= platforms/linux/px4_layer
MODULES		+= platforms/linux/publisher

