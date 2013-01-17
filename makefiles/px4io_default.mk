#
# Makefile for the px4io_default configuration
#

PLATFORM	 = px4io
SRCS		 = $(PX4_BASE)/platforms/empty.c

include $(PX4_BASE)/makefiles/firmware.mk
