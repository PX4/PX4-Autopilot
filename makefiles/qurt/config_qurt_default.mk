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
MODULES		+= drivers/pwm_out_sim
MODULES		+= drivers/led
MODULES		+= drivers/rgbled
MODULES		+= modules/sensors
#MODULES		+= drivers/ms5611

#
# System commands
#
MODULES	+= systemcmds/param

#
# General system control
#
#MODULES		+= modules/mavlink

#
# Estimation modules (EKF/ SO3 / other filters)
#
#MODULES		+= modules/attitude_estimator_ekf
#MODULES		+= modules/ekf_att_pos_estimator

#
# Vehicle Control
#
#MODULES		+= modules/mc_att_control

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/uORB
#MODULES		+= modules/dataman
#MODULES		+= modules/sdlog2
#MODULES		+= modules/simulator
#MODULES		+= modules/commander

#
# Libraries
#
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
#MODULES		+= lib/geo
#MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion

#
# QuRT port
#
MODULES		+= platforms/qurt/px4_layer
MODULES		+= platforms/posix/work_queue
MODULES		+= platforms/posix/drivers/accelsim
MODULES		+= platforms/posix/drivers/gyrosim
MODULES		+= platforms/posix/drivers/adcsim
MODULES		+= platforms/posix/drivers/barosim

#
# Unit tests
#
MODULES		+= platforms/qurt/tests/hello
MODULES		+= platforms/posix/tests/vcdev_test
MODULES		+= platforms/posix/tests/hrt_test
MODULES		+= platforms/posix/tests/wqueue

#
# sources for muorb over fastrpc
#
#MODULES         += $(PX4_BASE)/../muorb_qurt/
