#
# Makefile for the EAGLE QuRT *default* configuration
#

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= modules/sensors
#MODULES		+= platforms/qurt/drivers/mpu9x50
#MODULES		+= platforms/qurt/drivers/uart_esc

#
# System commands
#
MODULES	+= systemcmds/param


#
# General system control
#

#
# Estimation modules (EKF/ SO3 / other filters)
#
MODULES		+= modules/ekf_att_pos_estimator
MODULES		+= modules/attitude_estimator_q
MODULES		+= modules/position_estimator_inav

#
# Vehicle Control
#
MODULES		+= modules/mc_att_control
MODULES		+= modules/mc_pos_control

#
# Library modules
#
MODULES		+= modules/systemlib
MODULES		+= modules/systemlib/mixer
MODULES		+= modules/uORB
#MODULES		+= modules/dataman
MODULES		+= modules/commander

#
# Libraries
#
MODULES		+= lib/mathlib
MODULES		+= lib/mathlib/math/filter
MODULES		+= lib/geo
MODULES		+= lib/geo_lookup
MODULES		+= lib/conversion

#
# QuRT port
#
MODULES		+= platforms/qurt/px4_layer

#
# Unit tests
#

#
# sources for muorb over fastrpc
#
MODULES         += modules/muorb/adsp/
