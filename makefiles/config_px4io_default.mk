#
# Makefile for the px4io_default configuration
#

CONFIG		 = px4io_default
SRCS		 = $(PX4_BASE)/platforms/empty.c

include $(PX4_BASE)/makefiles/firmware.mk
