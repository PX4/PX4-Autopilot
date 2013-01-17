#
# Makefile for the px4fmu_default configuration
#

CONFIG		 = px4fmu_default
SRCS		 = $(PX4_BASE)/platforms/empty.c
ROMFS_ROOT	 = $(PX4_BASE)/ROMFS/$(CONFIG)

include $(PX4_BASE)/makefiles/firmware.mk
