#
# Makefile for the EAGLE *default* configuration
#

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= drivers/blinkm
MODULES		+= drivers/pwm_out_sim
MODULES		+= drivers/rgbled
MODULES		+= drivers/led
MODULES		+= modules/sensors
#MODULES		+= drivers/ms5611

#
# System commands
#
MODULES	+= systemcmds/param
MODULES += systemcmds/mixer
MODULES += systemcmds/ver
#MODULES += systemcmds/topic_listener

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
#MODULES 	+= modules/navigator
MODULES 	+= modules/mc_pos_control
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
MODULES		+= modules/commander
MODULES 	+= modules/controllib

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
MODULES		+= platforms/posix/px4_layer
MODULES		+= platforms/posix/work_queue

#
# Unit tests
#
#MODULES		+= platforms/posix/tests/hello
#MODULES		+= platforms/posix/tests/vcdev_test
#MODULES		+= platforms/posix/tests/hrt_test
#MODULES		+= platforms/posix/tests/wqueue

#
# muorb fastrpc changes.
#
#MODULES		+= $(PX4_BASE)../muorb_krait
