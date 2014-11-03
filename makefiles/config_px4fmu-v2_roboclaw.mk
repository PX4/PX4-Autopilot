#
# Makefile for the px4fmu_default configuration
#

include $(PX4_BASE)/makefiles/config_px4fmu-v2_default.mk

#
# Board support modules
#
MODULES		+= drivers/roboclaw

#
# Vehicle Control
#
MODULES		+= modules/segway
MODULES		+= modules/inv_pend
