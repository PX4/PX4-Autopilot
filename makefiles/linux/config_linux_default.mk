#
# Makefile for the Foo *default* configuration
#

#
# Use the configuration's ROMFS.
#
#ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/px4fmu_common

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/blinkm
MODULES		+= drivers/hil
MODULES		+= modules/sensors
#MODULES		+= drivers/ms5611

#
# System commands
#
MODULES	+= systemcmds/param

#
# General system control
#
MODULES		+= modules/mavlink

#
# Estimation modules (EKF/ SO3 / other filters)
#
MODULES		+= modules/attitude_estimator_ekf
MODULES		+= modules/ekf_att_pos_estimator

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
MODULES		+= modules/sdlog2
MODULES		+= modules/simulator

#
# Libraries
#
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/geo
MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion

#
# Linux port
#
MODULES		+= platforms/linux/px4_layer
MODULES		+= platforms/linux/drivers/accelsim
MODULES		+= platforms/linux/drivers/gyrosim
MODULES		+= platforms/linux/drivers/adcsim
MODULES		+= platforms/linux/drivers/barosim

#
# Unit tests
#
#MODULES		+= platforms/linux/tests/hello
#MODULES		+= platforms/linux/tests/vcdev_test
#MODULES		+= platforms/linux/tests/hrt_test
#MODULES		+= platforms/linux/tests/wqueue

