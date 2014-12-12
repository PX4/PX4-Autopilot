#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

LIBUAVCAN_LPC11C24_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

LIBUAVCAN_LPC11C24_SRC := $(shell find $(LIBUAVCAN_LPC11C24_DIR)/src -type f -name '*.cpp')

LIBUAVCAN_LPC11C24_INC := $(LIBUAVCAN_LPC11C24_DIR)/include/
