#
# Makefile for the px4fmu_default configuration
#

HOST_OS		 = native

MODULES		+= examples/hello_sky

#MODULES		+= modules/ekf_att_pos_estimator
#MODULES		+= modules/position_estimator_inav

#
# Vehicle Control
#
#MODULES		+= modules/fw_pos_control_l1
#MODULES		+= modules/fw_att_control
#MODULES		+= modules/mc_att_control
#MODULES		+= modules/mc_pos_control

#
# Library modules
#
#MODULES		+= modules/systemlib
#MODULES		+= modules/systemlib/mixer
#MODULES		+= modules/controllib
#MODULES		+= modules/uORB
#MODULES		+= modules/dataman
