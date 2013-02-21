#
# Makefile for the px4io_default configuration
#

CONFIG		 = px4io_default
SRCS		 = $(PX4_BASE)/platforms/empty.c

include $(PX4_MK_DIR)/firmware.mk
