#Added configuration specific flags here. 

ifndef HEXAGON_DRIVERS_ROOT
$(error HEXAGON_DRIVERS_ROOT is not set)
endif
ifndef EAGLE_DRIVERS_SRC
$(error EAGLE_DRIVERS_SRC is not set)
endif

INCLUDE_DIRS += $(HEXAGON_DRIVERS_ROOT)/inc

# For Actual flight we need to link against the driver dynamic libraries
LDFLAGS	+= -L${HEXAGON_DRIVERS_ROOT}/libs -lmpu9x50
LDFLAGS	+= -luart_esc
LDFLAGS	+= -lcsr_gps
LDFLAGS	+= -lrc_receiver

#
# Makefile for the EAGLE QuRT *default* configuration
#

#
# Board support modules
#
MODULES		+= drivers/device
MODULES		+= modules/sensors
MODULES		+= $(EAGLE_DRIVERS_SRC)/mpu9x50
MODULES		+= $(EAGLE_DRIVERS_SRC)/uart_esc
MODULES		+= $(EAGLE_DRIVERS_SRC)/rc_receiver
MODULES		+= $(EAGLE_DRIVERS_SRC)/csr_gps

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
# QuRT port
#
MODULES		+= platforms/qurt/px4_layer
MODULES		+= platforms/posix/work_queue

#
# Unit tests
#

#
# sources for muorb over fastrpc
#
MODULES         += modules/muorb/adsp/
